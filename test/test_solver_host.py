"""Unit tests for SolverHost: GPU access modes and CUDA-graph ownership.

No GPU, rclpy or curobo needed: solvers and logger are fakes.
"""

import threading
from contextlib import nullcontext

from curobo_ros.core.solver_host import SolverHost


class FakeLogger:

    def __init__(self):
        self.infos = []
        self.warns = []

    def info(self, msg, **_):
        self.infos.append(msg)

    def warn(self, msg, **_):
        self.warns.append(msg)


class FakeSeedIK:

    def __init__(self):
        self.destroyed = 0

    def destroy(self):
        self.destroyed += 1


class FakeSolver:

    def __init__(self, seed_ik=None, ik_solver=None, fail=False):
        self.resets = 0
        self.destroyed = 0
        self.seed_ik_solver = seed_ik
        self.ik_solver = ik_solver
        self._fail = fail

    def reset_cuda_graph(self):
        if self._fail:
            raise RuntimeError('boom')
        self.resets += 1

    def destroy(self):
        self.destroyed += 1


def _host(solvers=(), use_cuda_graph=True):
    host = SolverHost(FakeLogger())
    host.bind_solvers(lambda: list(solvers), use_cuda_graph)
    return host


def _hold_in_thread(host):
    """Hold exclusive access in another thread until the returned event is set."""
    held = threading.Event()
    release = threading.Event()

    def run():
        with host.exclusive():
            held.set()
            release.wait(5.0)

    t = threading.Thread(target=run)
    t.start()
    assert held.wait(5.0)
    return release, t


def _free_from_other_thread(host):
    result = []

    def run():
        with host.try_access() as acquired:
            result.append(acquired)

    t = threading.Thread(target=run)
    t.start()
    t.join(5.0)
    return result[0]


# ---------------------------------------------------------------- access modes

def test_try_access_free_lock_is_acquired_then_released():
    host = _host()
    with host.try_access() as acquired:
        assert acquired
    assert _free_from_other_thread(host)


def test_try_access_refused_while_other_thread_holds_exclusive():
    host = _host()
    release, t = _hold_in_thread(host)
    try:
        with host.try_access() as acquired:
            assert not acquired
        # Refusal must not release the holder's lock.
        assert not _free_from_other_thread(host)
    finally:
        release.set()
        t.join(5.0)
    assert _free_from_other_thread(host)


def test_try_access_is_reentrant_under_exclusive():
    host = _host()
    with host.exclusive():
        with host.try_access() as acquired:
            assert acquired
        assert not _free_from_other_thread(host)
    assert _free_from_other_thread(host)


def test_try_access_timeout_waits_for_release():
    host = _host()
    release, t = _hold_in_thread(host)
    threading.Timer(0.05, release.set).start()
    with host.try_access(timeout=2.0) as acquired:
        assert acquired
    t.join(5.0)


def test_exception_in_body_releases_access():
    host = _host()
    try:
        with host.try_access() as acquired:
            assert acquired
            raise ValueError
    except ValueError:
        pass
    assert _free_from_other_thread(host)


# ------------------------------------------------------------ graph ownership

def test_first_claim_adopts_without_release():
    solver = FakeSolver()
    host = _host([solver])
    host.claim_graphs('motion')
    assert solver.resets == 0


def test_same_owner_claim_is_noop():
    solver = FakeSolver()
    host = _host([solver])
    host.claim_graphs('motion')
    host.claim_graphs('motion')
    assert solver.resets == 0


def test_owner_change_releases_every_solver():
    a, b = FakeSolver(), FakeSolver()
    host = _host([a, None, b])
    host.claim_graphs('motion')
    host.claim_graphs('reactive:MPC')
    assert (a.resets, b.resets) == (1, 1)


def test_graphs_disabled_never_release():
    solver = FakeSolver()
    host = _host([solver], use_cuda_graph=False)
    host.claim_graphs('motion')
    host.claim_graphs('reactive:MPC')
    assert solver.resets == 0


def test_none_owner_is_ignored():
    solver = FakeSolver()
    host = _host([solver])
    host.claim_graphs('motion')
    host.claim_graphs(None)
    host.claim_graphs('motion')
    assert solver.resets == 0


# --------------------------------------------------------------- capture guard

def test_capture_guard_is_take_and_clear():
    host = _host()
    assert host.capture_guard() is host.exclusive()
    assert isinstance(host.capture_guard(), nullcontext)


def test_release_marks_capture_pending():
    host = _host([FakeSolver()])
    host.capture_guard()
    host.claim_graphs('motion')
    host.claim_graphs('reactive:MPC')
    assert host.capture_guard() is host.exclusive()


def test_mark_capture_pending():
    host = _host()
    host.capture_guard()
    host.mark_capture_pending()
    assert host.capture_guard() is host.exclusive()


# ------------------------------------------------------------- graph release

def test_release_reaches_nested_solvers_without_destroying_core():
    seed = FakeSeedIK()
    nested_seed = FakeSeedIK()
    nested = FakeSolver(seed_ik=nested_seed)
    solver = FakeSolver(seed_ik=seed, ik_solver=nested)
    host = _host([solver])
    host.release_all_graphs()
    assert solver.resets == 1 and nested.resets == 1
    assert seed.destroyed == 1 and nested_seed.destroyed == 1
    assert solver.destroyed == 0 and nested.destroyed == 0


def test_self_referencing_ik_solver_is_not_recursed():
    solver = FakeSolver()
    solver.ik_solver = solver
    host = _host([solver])
    host.release_all_graphs()
    assert solver.resets == 1


def test_release_failure_is_logged_not_raised():
    bad, good = FakeSolver(fail=True), FakeSolver()
    host = _host([bad, good])
    host.release_all_graphs()
    assert good.resets == 1
    assert len(host._logger.warns) == 1

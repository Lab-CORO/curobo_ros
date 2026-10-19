"""Unit tests for SolverHost: solver lifecycle, GPU access modes and CUDA-graph
ownership.

No GPU, rclpy or curobo needed: solvers and logger are fakes.
"""

import threading
from contextlib import nullcontext

import pytest

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


class FakeMotionPlanner:
    """MotionPlanner shape: graphs live on two sub-solvers."""

    def __init__(self):
        self.trajopt_solver = FakeSolver()
        self.ik_solver = FakeSolver()


def _motion_parts(mp):
    return (mp.trajopt_solver, mp.ik_solver)


def _own_parts(solver):
    return (solver,)


def _host(use_cuda_graph=True):
    return SolverHost(FakeLogger(), use_cuda_graph)


def _host_with(solvers, use_cuda_graph=True):
    """Host with each {key: solver} registered (graph-holding) and built."""
    host = _host(use_cuda_graph)
    for key, solver in solvers.items():
        host.register(key, lambda s=solver: s, graph_parts=_own_parts)
        host.ensure(key)
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


# ------------------------------------------------------------ solver lifecycle

def test_get_never_builds():
    builds = []
    host = _host()
    host.register('motion', lambda: builds.append(1) or FakeMotionPlanner())
    assert host.get('motion') is None
    assert host.get('unknown') is None
    assert builds == []


def test_ensure_builds_once_and_get_returns_it():
    builds = []
    host = _host()
    host.register('mpc', lambda: builds.append(1) or FakeSolver())
    solver = host.ensure('mpc')
    assert host.ensure('mpc') is solver
    assert host.get('mpc') is solver
    assert builds == [1]


def test_ensure_builds_under_exclusive_access():
    seen = []
    host = _host()

    def build():
        seen.append(_free_from_other_thread(host))
        return FakeSolver()

    host.register('mpc', build)
    host.ensure('mpc')
    assert seen == [False]


def test_failed_build_leaves_slot_empty_and_raises():
    host = _host()
    old = FakeSolver()
    builds = [lambda: old]

    def build():
        return builds.pop(0)()

    host.register('motion', build)
    host.ensure('motion')

    def boom():
        raise RuntimeError('warmup failed')

    builds.append(boom)
    with pytest.raises(RuntimeError):
        host.rebuild('motion')
    assert host.get('motion') is None


def test_hooks_see_new_solver_then_prime_runs():
    order = []
    host = _host()
    host.register('lbfgs', FakeSolver,
                  prime=lambda: order.append(('prime', host.get('lbfgs'))))
    host.on_built(lambda key, solver: order.append(('hook', key, host.get(key) is solver)))
    solver = host.ensure('lbfgs')
    assert order == [('hook', 'lbfgs', True), ('prime', solver)]


def test_hook_failure_is_logged_not_raised():
    host = _host()
    host.register('mpc', FakeSolver)

    def bad_hook(key, solver):
        raise RuntimeError('replay failed')

    host.on_built(bad_hook)
    assert host.ensure('mpc') is not None
    assert len(host._logger.warns) == 1


def test_rebuild_all_rebuilds_only_built_solvers_in_registration_order():
    built = []
    host = _host()
    for key in ('motion', 'ik', 'fk', 'mpc', 'lbfgs'):
        host.register(key, lambda k=key: built.append(k) or FakeSolver())
    host.ensure('mpc')
    host.ensure('motion')
    host.ensure('fk')
    old = {k: host.get(k) for k in ('motion', 'fk', 'mpc')}
    built.clear()
    host.rebuild_all()
    assert built == ['motion', 'fk', 'mpc']
    assert all(host.get(k) is not old[k] for k in old)
    assert host.get('ik') is None and host.get('lbfgs') is None


def test_rebuild_all_tolerates_registration_from_a_hook():
    host = _host()
    host.register('motion', FakeMotionPlanner)
    host.ensure('motion')
    host.on_built(lambda key, solver: host.register('retarget', FakeSolver))
    host.rebuild_all()
    assert host.get('retarget') is None


def test_rebuild_all_marks_capture_pending():
    host = _host_with({'mpc': FakeSolver()})
    host.capture_guard()
    host.rebuild_all()
    assert host.capture_guard() is host.exclusive()


def test_reregister_keeps_built_solver():
    host = _host()
    host.register('mpc', FakeSolver)
    solver = host.ensure('mpc')
    host.register('mpc', FakeSolver)
    assert host.get('mpc') is solver


# ------------------------------------------------------------ graph ownership

def test_first_activation_adopts_without_release():
    solver = FakeSolver()
    host = _host_with({'motion': solver})
    host.activate('motion')
    assert solver.resets == 0


def test_same_owner_activation_is_noop():
    solver = FakeSolver()
    host = _host_with({'motion': solver})
    host.activate('motion')
    host.activate('motion')
    assert solver.resets == 0


def test_owner_change_releases_every_graph_holding_solver():
    a, b = FakeSolver(), FakeSolver()
    host = _host_with({'motion': a, 'mpc': b})
    host.activate('motion')
    host.activate('mpc')
    assert (a.resets, b.resets) == (1, 1)


def test_release_reaches_motion_sub_solvers_and_skips_ik_fk():
    mp, ik, fk = FakeMotionPlanner(), FakeSolver(), FakeSolver()
    host = _host()
    host.register('motion', lambda: mp, graph_parts=_motion_parts)
    host.register('ik', lambda: ik)
    host.register('fk', lambda: fk)
    for key in ('motion', 'ik', 'fk'):
        host.ensure(key)
    host.activate('motion')
    host.activate('mpc')
    assert (mp.trajopt_solver.resets, mp.ik_solver.resets) == (1, 1)
    assert (ik.resets, fk.resets) == (0, 0)


def test_unbuilt_solver_is_not_released():
    host = _host()
    host.register('mpc', FakeSolver, graph_parts=_own_parts)
    host.activate('motion')
    host.activate('mpc')
    assert host.get('mpc') is None


def test_peeking_a_non_owner_never_releases():
    a, b = FakeSolver(), FakeSolver()
    host = _host_with({'motion': a, 'mpc': b})
    host.activate('mpc')
    for _ in range(10):
        assert host.get('motion') is a
        host.ensure('motion')
    assert (a.resets, b.resets) == (0, 0)


def test_graphs_disabled_never_release():
    solver = FakeSolver()
    host = _host_with({'motion': solver}, use_cuda_graph=False)
    host.activate('motion')
    host.activate('mpc')
    assert solver.resets == 0


def test_none_key_is_ignored():
    solver = FakeSolver()
    host = _host_with({'motion': solver})
    host.activate('motion')
    host.activate(None)
    host.activate('motion')
    assert solver.resets == 0


# --------------------------------------------------------------- capture guard

def test_capture_guard_is_take_and_clear():
    host = _host()
    assert host.capture_guard() is host.exclusive()
    assert isinstance(host.capture_guard(), nullcontext)


def test_owner_change_marks_capture_pending():
    host = _host_with({'motion': FakeSolver()})
    host.capture_guard()
    host.activate('motion')
    host.activate('mpc')
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
    host = _host_with({'mpc': solver})
    host.activate('motion')
    host.activate('mpc')
    assert solver.resets == 1 and nested.resets == 1
    assert seed.destroyed == 1 and nested_seed.destroyed == 1
    assert solver.destroyed == 0 and nested.destroyed == 0


def test_self_referencing_ik_solver_is_not_recursed():
    solver = FakeSolver()
    solver.ik_solver = solver
    host = _host_with({'mpc': solver})
    host.activate('motion')
    host.activate('mpc')
    assert solver.resets == 1


def test_release_failure_is_logged_not_raised():
    bad, good = FakeSolver(fail=True), FakeSolver()
    host = _host_with({'a': bad, 'b': good})
    host.activate('a')
    host.activate('b')
    assert good.resets == 1
    assert len(host._logger.warns) == 1

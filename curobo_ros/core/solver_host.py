"""
SolverHost: the single owner of GPU access and CUDA-graph ownership.

Enforces the invariants of docs/adr/0001-gpu-access-invariants.md in one place:
one live CUDA graph, capture is process-global, GPU lock is the outermost lock.
Callers never see the lock; they ask for one of two access modes:

- ``exclusive()``: blocking, re-entrant. Anything that may capture a CUDA graph
  (plan, warmup, rebuild, priming) or must not interleave with a capture.
- ``try_access(timeout)``: non-blocking (or short-timeout). Periodic/best-effort
  GPU work; when refused, the caller skips its work this cycle.

Deliberately free of rclpy / torch / curobo imports so it can be unit-tested
without a GPU. Solvers are duck-typed (``reset_cuda_graph``, ``seed_ik_solver``,
``ik_solver``).

Transitional (step A of the refactor, see GLOSSARY.md): the solvers still live
on the node, so the node binds a provider that lists them and computes the graph
owner key itself. ``bind_solvers``, ``claim_graphs``, ``mark_capture_pending`` and
``capture_guard`` become internal once SolverHost owns the solvers (step B).
"""

import threading
from contextlib import contextmanager, nullcontext


class SolverHost:
    """Owns the GPU lock and decides which solver holds the live CUDA graph."""

    def __init__(self, logger):
        self._logger = logger
        # RLock: a thread holding exclusive access may re-enter either mode.
        self._lock = threading.RLock()
        self._use_cuda_graph = False
        self._solvers_provider = list
        # Which solver currently owns the single live CUDA graph. None = no
        # owner yet: the first claim adopts without releasing (startup warms
        # exactly one solver, so it is already the sole owner).
        self._graph_owner = None
        # True whenever the next reactive solver call may CAPTURE a graph
        # (fresh solver, or right after a release) rather than just replay.
        self._capture_pending = True
        self._capture_pending_lock = threading.Lock()

    def bind_solvers(self, solvers_provider, use_cuda_graph: bool) -> None:
        """Transitional: callable returning every live solver object."""
        self._solvers_provider = solvers_provider
        self._use_cuda_graph = bool(use_cuda_graph)

    # ------------------------------------------------------------------
    # Access modes
    # ------------------------------------------------------------------

    def exclusive(self):
        """Blocking, re-entrant exclusive GPU access (context manager)."""
        return self._lock

    @contextmanager
    def try_access(self, timeout: float = 0.0):
        """Yield True if GPU access was obtained, False if it was refused.

        ``timeout`` <= 0 never waits. Only releases what it acquired.
        """
        if timeout > 0.0:
            acquired = self._lock.acquire(timeout=timeout)
        else:
            acquired = self._lock.acquire(blocking=False)
        try:
            yield acquired
        finally:
            if acquired:
                self._lock.release()

    # ------------------------------------------------------------------
    # CUDA graph ownership (only the active solver holds a live graph)
    # ------------------------------------------------------------------

    def claim_graphs(self, owner_key) -> None:
        """Make owner_key the sole live-graph owner, releasing all others.

        No-op if graphs are disabled, owner_key is None or unchanged. Two
        captured graphs cannot coexist in one CUDA context: the inactive one's
        baked device addresses get invalidated by the active one's allocator
        activity, so replaying it segfaults in cuGraphLaunch.
        """
        if not self._use_cuda_graph or owner_key is None:
            return
        if owner_key == self._graph_owner:
            return
        if self._graph_owner is None:
            self._graph_owner = owner_key
            return
        self._logger.info(
            f"CUDA graph owner {self._graph_owner} -> {owner_key}: releasing "
            f"other captured graphs so only the active solver holds one")
        self.release_all_graphs()
        self._graph_owner = owner_key

    def release_all_graphs(self) -> None:
        """Free every solver's captured CUDA graph (frees graph + its pool).

        The solvers themselves and their shared collision world are untouched;
        each re-captures on next use.
        """
        with self._lock:
            for solver in self._solvers_provider():
                self._safe_reset_graph(solver)
            self.mark_capture_pending()

    def mark_capture_pending(self) -> None:
        """Transitional: the next reactive solver call may capture a graph."""
        with self._capture_pending_lock:
            self._capture_pending = True

    def capture_guard(self):
        """Transitional belt: exclusive access for ONE step that may capture.

        Take-and-clear, so exactly one caller gets the lock per release.
        Priming (ReactiveController.prime_cuda_graphs) is the real guarantee.
        """
        with self._capture_pending_lock:
            pending = self._capture_pending
            self._capture_pending = False
        return self._lock if pending else nullcontext()

    def _safe_reset_graph(self, solver) -> None:
        """Release a solver's captured CUDA graph(s); never raise into callers.

        Uses reset_cuda_graph(), NOT destroy(): SolverCore.destroy() drops
        reset_cuda_graph()'s guards and orphans the metrics rollout's
        constraint tensors (con_scene_collision froze, debug 2026-07-25).

        reset_cuda_graph() alone misses the nested SeedIKSolver's private
        Levenberg-Marquardt GraphExecutor (cause of the original cuGraphLaunch
        segfault on Classic after MPC), so reach seed_ik_solver explicitly:
        SeedIKSolver.destroy() only resets its own two GraphExecutors. Also
        recurse into a nested ik_solver (MpcSolver's goal-state IK, which
        MpcSolver.reset_cuda_graph() never releases).
        """
        if solver is None:
            return
        try:
            if hasattr(solver, 'reset_cuda_graph'):
                solver.reset_cuda_graph()
            seed_ik_solver = getattr(solver, 'seed_ik_solver', None)
            if seed_ik_solver is not None:
                seed_ik_solver.destroy()
            nested_ik_solver = getattr(solver, 'ik_solver', None)
            if nested_ik_solver is not None and nested_ik_solver is not solver:
                self._safe_reset_graph(nested_ik_solver)
        except Exception as e:
            self._logger.warn(
                f"CUDA graph release failed on {type(solver).__name__}: {e}")

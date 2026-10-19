"""
SolverHost: sole owner of every cuRobo solver, of GPU access and of CUDA-graph
ownership.

Enforces the invariants of docs/adr/0001-gpu-access-invariants.md in one place:
one live CUDA graph, capture is process-global, GPU lock is the outermost lock.

Solvers. Each solver is identified by its key ('motion', 'ik', 'fk', 'mpc',
'lbfgs', 'retarget') and registered with a builder. The host decides WHEN to
build (lazily on ``ensure``, again on ``rebuild``); the builder decides HOW and
keeps its own parameter reads. Callers never keep a reference: they ask
``get(key)`` on each use, so a rebuild is seen everywhere at once. Classic,
Multi-Point and Joint-Space all use the one 'motion' solver.

GPU access. Two modes, the lock itself is never exposed:

- ``exclusive()``: blocking, re-entrant. Anything that may capture a CUDA graph
  (plan, build, rebuild, priming) or must not interleave with a capture.
- ``try_access(timeout)``: non-blocking (or short-timeout). Periodic/best-effort
  GPU work; when refused, the caller skips its work this cycle.

Deliberately free of rclpy / torch / curobo imports so it can be unit-tested
without a GPU. Solvers are duck-typed (``reset_cuda_graph``, ``seed_ik_solver``,
``ik_solver``).
"""

import threading
from contextlib import contextmanager, nullcontext


class _Entry:
    __slots__ = ('build', 'graph_parts', 'prime', 'solver')

    def __init__(self, build, graph_parts, prime):
        self.build = build
        self.graph_parts = graph_parts
        self.prime = prime
        self.solver = None


class SolverHost:
    """Owns the solvers, the GPU lock and the single live CUDA graph."""

    def __init__(self, logger, use_cuda_graph: bool = False):
        self._logger = logger
        self._use_cuda_graph = bool(use_cuda_graph)
        # RLock: a thread holding exclusive access may re-enter either mode.
        self._lock = threading.RLock()
        # Registration order is the rebuild order (shared solvers first).
        self._entries = {}
        self._built_hooks = []
        # Which solver currently owns the single live CUDA graph. None = no
        # owner yet: the first activation adopts without releasing (startup
        # warms exactly one solver, so it is already the sole owner).
        self._graph_owner = None
        # True whenever the next reactive solver call may CAPTURE a graph
        # (fresh solver, or right after a release) rather than just replay.
        self._capture_pending = True
        self._capture_pending_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Solvers
    # ------------------------------------------------------------------

    def register(self, key, build, graph_parts=None, prime=None) -> None:
        """Declare how solver ``key`` is built. Does not build it.

        build:       ``() -> solver``. May raise; the slot then stays empty.
        graph_parts: ``solver -> iterable`` of the objects whose CUDA graphs
                     are released when another solver takes the graph. None =
                     this solver never holds a graph (IK, FK).
        prime:       ``() -> None``, run after the built hooks (so it captures
                     with the attached payload already written).

        Re-registering a key replaces its builder and keeps a built solver.
        """
        entry = self._entries.get(key)
        if entry is None:
            self._entries[key] = _Entry(build, graph_parts, prime)
        else:
            entry.build, entry.graph_parts, entry.prime = build, graph_parts, prime

    def on_built(self, callback) -> None:
        """Subscribe ``callback(key, solver)``, run after every (re)build."""
        self._built_hooks.append(callback)

    def get(self, key):
        """The live solver for ``key``, or None. No side effects."""
        entry = self._entries.get(key)
        return entry.solver if entry is not None else None

    def ensure(self, key):
        """The solver for ``key``, built first if it does not exist yet."""
        entry = self._entries[key]
        if entry.solver is None:
            with self._lock:
                if entry.solver is None:
                    self._build(key, entry)
        return entry.solver

    def rebuild(self, key):
        """Recreate solver ``key`` from scratch, whether or not it exists."""
        entry = self._entries[key]
        with self._lock:
            entry.solver = None
            self._build(key, entry)
            self.mark_capture_pending()
        return entry.solver

    def rebuild_all(self) -> None:
        """Recreate every solver that exists (e.g. collision-cache change).

        Never-built solvers stay unbuilt. Rebuilt solvers hold no graph yet, so
        the next reactive step may capture: mark it pending.
        """
        with self._lock:
            # Snapshot: a built hook may create a planner wrapper, which
            # registers a new (unbuilt) key mid-loop.
            for key, entry in list(self._entries.items()):
                if entry.solver is not None:
                    entry.solver = None
                    self._build(key, entry)
            self.mark_capture_pending()

    def _build(self, key, entry) -> None:
        # Caller holds the lock. The slot is filled before the hooks run, so a
        # hook (or prime) reading get(key) sees the new solver.
        entry.solver = entry.build()
        for hook in self._built_hooks:
            try:
                hook(key, entry.solver)
            except Exception as e:
                self._logger.warn(f"solver '{key}' built-hook failed: {e}")
        if entry.prime is not None:
            entry.prime()

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

    def activate(self, key) -> None:
        """Make solver ``key`` the sole live-graph owner, releasing all others.

        Only the planner-activation path calls this; ``get`` never does. No-op
        if graphs are disabled, key is None or unchanged. Two captured graphs
        cannot coexist in one CUDA context: the inactive one's baked device
        addresses get invalidated by the active one's allocator activity, so
        replaying it segfaults in cuGraphLaunch.
        """
        if not self._use_cuda_graph or key is None:
            return
        if key == self._graph_owner:
            return
        if self._graph_owner is None:
            self._graph_owner = key
            return
        self._logger.info(
            f"CUDA graph owner {self._graph_owner} -> {key}: releasing "
            f"other captured graphs so only the active solver holds one")
        self._release_all_graphs()
        self._graph_owner = key

    def _release_all_graphs(self) -> None:
        """Free every solver's captured CUDA graph (frees graph + its pool).

        The solvers themselves and their shared collision world are untouched;
        each re-captures on next use.
        """
        with self._lock:
            for entry in list(self._entries.values()):
                if entry.solver is None or entry.graph_parts is None:
                    continue
                for part in entry.graph_parts(entry.solver):
                    self._safe_reset_graph(part)
            self.mark_capture_pending()

    def mark_capture_pending(self) -> None:
        """The next reactive solver call may capture a graph."""
        with self._capture_pending_lock:
            self._capture_pending = True

    def capture_guard(self):
        """Belt: exclusive access for ONE step that may capture.

        Take-and-clear, so exactly one caller gets the lock per release.
        Priming (ReactiveController.prime_cuda_graphs) is the real guarantee;
        this goes away once priming is trusted alone.
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

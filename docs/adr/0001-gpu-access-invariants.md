# ADR-0001: GPU access invariants

- Status: accepted
- Date: 2026-10-18

## Context

curobo_ros runs several cuRobo solvers in one process and one CUDA context,
next to periodic GPU work (depth integration, voxel publishing,
collision-sphere viz) on other executor threads. cuRobo captures CUDA graphs
lazily, on several different calls per solver. The following incidents each
cost a node restart or a crashed goal:

- **2026-07-25**: calling `destroy()` instead of `reset_cuda_graph()` on a
  solver's rollout orphaned its constraint tensors. `con_scene_collision` froze
  at a constant.
- **2026-07-28**: a reactive step captured a graph while the depth callback ran
  `mapper.integrate()`, which raised `cudaErrorStreamCaptureInvalidated`.
- **2026-08-08**: switching MPC -> Classic with live cameras captured the
  seed-IK graph without holding the lock. The aborted capture poisoned the CUDA
  context (106 consecutive CUDA failures).
- **2026-09-14**: a reactive solver captured on its *second* step, which was
  unguarded, and the 7 Hz voxel publisher invalidated the capture.
- Earlier: replaying a `MotionPlanner` graph after MPC activity segfaulted in
  `cuGraphLaunch`, because two live graphs cannot coexist.

## Decision

These invariants hold, and are enforced in one place, `SolverHost`:

1. **One live CUDA graph.** Only the current graph owner may hold captured
   graphs. Changing owner releases every other solver's graphs, including the
   nested seed-IK and goal-IK solvers that `reset_cuda_graph()` does not reach.
   Release uses `reset_cuda_graph()`, never `destroy()` on a solver core.
   Building a graph-holding solver other than the owner (lazy build, single
   rebuild, collision-cache `rebuild_all`) captures during its warmup, so it
   is followed by the same release, and only the owner is primed again.
2. **Capture is process-global.** While any graph is being captured, no other
   thread may issue a CUDA op. Every capturing path holds exclusive GPU access.
   Every other GPU path takes try-or-skip access and skips its work when refused.
   On-demand services (IK, FK) run their whole callback under exclusive access
   (`SolverHost.exclusive_callback`).
3. **Capture happens before the servo loop.** Reactive controllers prime every
   lazily-captured graph under exclusive access before stepping, so the loop
   only replays.
4. **Lock order.** The GPU lock is the outermost lock:
   `GPU lock > RobotContext.strategy_lock > JointCommandStrategy.buffer_lock`.
   `plan()` runs under the GPU lock and calls `RobotContext.set_command()`.
   Nothing below it may acquire the GPU lock.
5. **One re-entrant lock.** Both access modes use the same `RLock`, so a thread
   that already has exclusive access always gets try-or-skip access.

## Consequences

- Callers never see the lock or the graph-owner bookkeeping. They ask
  `SolverHost` for an access mode.
- Periodic GPU work can drop frames or cycles during warmup or rebuild. This is
  by design.
- The servo loop steps without the GPU lock: invariant 3 is its only
  protection. The per-step capture check that once backed it up was removed
  after `ReactiveController.execute()` reported no capture inside the loop
  for MPPI, LBFGS and retarget; that report stays, and a capture it flags
  must be fixed in priming, not by locking the loop again. Proposals to
  merge solvers into one graph, or to let threads run CUDA work during a
  capture, must revisit this ADR.

# Glossary

Domain terms used across curobo_ros. Code, docs and reviews should use these
names; when a term is sharpened or a new concept gets a name, update it here.

**Solver**
: A cuRobo optimisation object that owns GPU state and may capture CUDA graphs:
  `MotionPlanner`, the MPPI and LBFGS `ModelPredictiveControl` instances, the
  `MotionRetargeter`, the IK solver and the FK collision checker.

**SolverHost** (`curobo_ros/core/solver_host.py`)
: The single owner of every solver and of GPU access. It decides *when* a
  solver is built, warmed up, rebuilt, given a new Scene or has its CUDA graphs
  released, and it is the only thing that hands out GPU access. It does not
  depend on rclpy. See [ADR-0001](docs/adr/0001-gpu-access-invariants.md).

**Builder**
: The planner-provided recipe that decides *how* a solver is constructed
  (reads its own ROS parameters, creates the cuRobo config). A solver's identity
  is its builder: planners that declare the same builder share one solver.

**Graph owner**
: The one solver currently allowed to hold a live captured CUDA graph. Changing
  owner releases every other solver's graphs. All open-loop planners share one
  owner (they share the `MotionPlanner`); each reactive controller is its own.

**Exclusive GPU access**
: Blocking, re-entrant access to the GPU, required for anything that may
  capture a CUDA graph (plan, warmup, rebuild, priming) or that must not be
  interleaved with a capture (ESDF refresh, world push, attach/detach).

**Try-or-skip GPU access**
: Non-blocking (optionally short-timeout) access for periodic or best-effort GPU
  work: depth integration, voxel grid publish/query, collision-sphere viz, live
  goal updates. When access is refused the caller skips its work this cycle.

**Open-loop planner**
: A planner that produces a full trajectory, then executes it (`SinglePlanner`
  subclasses: Classic, Multi-Point, Joint-Space).

**Reactive controller**
: A planner that runs a closed control loop until cancelled
  (`ReactiveController` subclasses: MPPI, LBFGS, Retarget).

**Scene**
: The single collision world (primitives + perception ESDF) owned by
  `ObstacleManager` and pushed into every solver.

# Unified Planner

The **unified planner** is the central node of `curobo_ros` (node name `unified_planner`, executable `curobo_trajectory_planner`). One node hosts every planning algorithm, the IK/FK services, obstacle management, and robot execution, behind a single set of interfaces. Clients never talk to a specific algorithm — they call `generate_trajectory` / `execute_trajectory`, and the *active planner* decides how the motion is computed.

## Available planners

| Key | Enum ID | Mode | What it does |
|---|---|---|---|
| `classic` | 0 | Open-loop | Single Cartesian goal → collision-free trajectory (default) |
| `mpc` | 1 | Closed-loop | Model Predictive Control (MPPI): continuous re-optimization while executing |
| `lbfgs` | 2 | Closed-loop | Model Predictive Control (LBFGS + B-spline), built on cuRobo's native `optimize_next_action()` pacing instead of MPC's hand-rolled horizon consumption |
| `multi_point` | 4 | Open-loop | Sequence of Cartesian waypoints, planned segment by segment |
| `joint_space` | 5 | Open-loop | Goal expressed directly in joint space |
| `retarget` | 6 | Closed-loop | IK-based pose-stream follower for teleoperation |

Enum ID 3 has no constant in `SetPlanner.srv` and no catalog entry — switching to it fails. Orientation/position constraints are available on the classic planner through the `trajectory_constraints` request field instead.

The catalog lives in one place, `PlannerFactory._PLANNER_CATALOG` (`curobo_ros/planners/planner_factory.py`); `GetPlanners` reflects it at runtime, so the service is always authoritative:

```bash
ros2 service call /unified_planner/get_planners curobo_msgs/srv/GetPlanners
```

## Open-loop vs closed-loop

Every planner declares an `ExecutionMode` that changes the behavior of the `execute_trajectory` action:

- **`OPEN_LOOP`** (`classic`, `multi_point`, `joint_space`): plan once, stream the interpolated trajectory to the robot, succeed when it ends. Feedback carries `step_progression` (0→1).
- **`CLOSED_LOOP`** (`mpc`, `lbfgs`, `retarget`): the action starts a servo loop that re-solves continuously, tracks `position_error`, and **keeps running after reaching the goal** (feedback `on_target: true`, state `ON_TARGET`). Retarget the goal live by publishing to `/unified_planner/mpc_goal`; stop by cancelling the goal. See [MPC Implementation](mpc-implementation.md).

## Two base classes

Open-loop planners subclass `SinglePlanner`; all three share **one cuRobo `MotionPlanner`**, held by `SolverHost` under the key `'motion'`, and its warmup.

Closed-loop planners subclass `ReactiveController`, which owns the whole servo loop (goal admission, live-goal plumbing, periodic perception refresh, command pacing); a concrete controller only implements five hooks: `build_solver()`, `setup()`, `step()`, `apply_live_goal()`, `is_converged()`.

Adding a planner means subclassing one of the two bases and adding one line to the factory catalog — the `set_planner`/`get_planners` services, the action, and the shared context all work unchanged.

## Switching planners at runtime

```bash
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
```

What happens on a switch:

- Planner instances and their solvers are **cached** (`SolverHost`, one entry per solver key): only the first switch to a planner **builds** its solver. Later switches never rebuild.
- The switch is refused while an execution goal is active.
- With `use_cuda_graph: true` the node keeps **at most one live CUDA graph** ([ADR-0001](../adr/0001-gpu-access-invariants.md)): a switch to another solver releases every other captured graph, and the new reactive solver **re-captures its graphs inside `set_planner`** ("priming"), before the servo loop starts. A closed-loop goal therefore starts in well under a second; the cost is paid by the switch call itself.
- The open-loop planners are not primed: their first plan after a switch re-captures the `MotionPlanner` graphs.

### Measured switch cost

Emulator, both cameras live (perception ESDF active, `voxel_size: 0.05`), Jetson AGX Orin, `use_cuda_graph: true`. "1st switch" includes the solver build; "later switch" is a return to an already built planner.

| Target planner | 1st switch (`set_planner`) | Later switch (`set_planner`) | Goal start → 1st feedback |
|---|---|---|---|
| `mpc` (MPPI) | ~2.0 s | ~0.85 s | ~0.35 s |
| `lbfgs` | ~2.7 s | ~2.2–2.4 s | ~0.38 s |
| `retarget` | ~3.3 s | ~1.5–3.0 s | ~0.08 s |
| `classic` | ~0.01 s | ~0.01 s | first plan ~4.1 s, then ~0.6 s |

Later switches are dominated by CUDA-graph re-capture, not by building: the L-BFGS controller captures its initial-state, optimizer-iteration and metrics graphs (~2 s); the retargeter captures the seed-IK graphs of its global and local IK (~1.5 s each). With `use_cuda_graph: false` a switch costs only the first build, but every solve is slower.

The node logs one line per priming solve (`prime solve N captured K graph(s) in T ms: [...]`). A warning `CUDA graph(s) captured INSIDE the servo loop` means priming missed a path: report it, since that capture ran without exclusive GPU access.

The startup planner is set by the `planner_type` parameter (default `classic`).

## One shared GPU context

All solvers — `MotionPlanner`, `ModelPredictiveControl`, `InverseKinematics` — are built from the same `ConfigWrapperMotion` context: same robot model, same `Scene`, same collision caches, same perception ESDF. Consequences:

- An obstacle added via `add_object` is immediately seen by planning, MPC, and IK alike.
- `set_link_collision` affects every solver in one call.
- There is no separate "MPC world" to keep in sync.

## IK and FK services

IK and FK are hosted by the same node but initialized **lazily**: call `warmup_ik` / `warmup_fk` (with a batch size) before the first `ik` / `fk` call. IK is collision-aware and shares the scene; FK is purely kinematic. Details and examples: [Tutorial 6](../tutorials/06-ik-fk-services.md), field reference: [ROS Interfaces](ros-interfaces.md).

## Trajectory caching

`generate_trajectory` caches its result for `trajectory_cache_ttl` seconds (default 30). A subsequent `execute_trajectory` goal with `allow_cached: true` (the default) and a matching target reuses it instead of re-planning — this is what makes the RViz "Generate" then "Send" workflow cheap. `clear_trajectory` empties the cache and the preview.

## Source files

```
curobo_ros/planners/
├── trajectory_planner.py    # TrajectoryPlanner ABC, ExecutionMode, PlannerResult
├── single_planner.py        # open-loop base (shared MotionPlanner)
├── classic_planner.py
├── multi_point_planner.py
├── joint_space_planner.py
├── reactive_controller.py   # closed-loop base (servo loop)
├── mppi_planner.py          # MPPIController      (planner key `mpc`)
├── lbfgs_planner.py         # LBFGSController     (planner key `lbfgs`)
├── retarget_controller.py   # RetargetController
└── planner_factory.py       # catalog + PlannerManager
```

## Related pages

- [Architecture](architecture.md) — how the planner layer fits the whole node
- [MPC Implementation](mpc-implementation.md) — the closed-loop path in depth
- [Parameters](parameters.md) — `planner_type`, `mpc_*`, `retarget_*`, `trajectory_cache_ttl`
- [Tutorial 5](../tutorials/05-mpc-planner.md) — hands-on MPC

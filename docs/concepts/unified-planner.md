# Unified Planner

The **unified planner** is the central node of `curobo_ros` (node name `unified_planner`, executable `curobo_trajectory_planner`). One node hosts every planning algorithm, the IK/FK services, obstacle management, and robot execution, behind a single set of interfaces. Clients never talk to a specific algorithm — they call `generate_trajectory` / `execute_trajectory`, and the *active planner* decides how the motion is computed.

## Available planners

| Key | Enum ID | Mode | What it does |
|---|---|---|---|
| `classic` | 0 | Open-loop | Single Cartesian goal → collision-free trajectory (default) |
| `mpc` | 1 | Closed-loop | Model Predictive Control: continuous re-optimization while executing |
| `multi_point` | 4 | Open-loop | Sequence of Cartesian waypoints, planned segment by segment |
| `joint_space` | 5 | Open-loop | Goal expressed directly in joint space |
| `retarget` | 6 | Closed-loop | IK-based pose-stream follower for teleoperation |

Enum IDs 2 (`BATCH`) and 3 (`CONSTRAINED`) exist in `SetPlanner.srv` but are **not implemented** — switching to them fails. Orientation/position constraints are available on the classic planner through the `trajectory_constraints` request field instead.

The catalog lives in one place, `PlannerFactory._PLANNER_CATALOG` (`curobo_ros/planners/planner_factory.py`); `GetPlanners` reflects it at runtime, so the service is always authoritative:

```bash
ros2 service call /unified_planner/get_planners curobo_msgs/srv/GetPlanners
```

## Open-loop vs closed-loop

Every planner declares an `ExecutionMode` that changes the behavior of the `execute_trajectory` action:

- **`OPEN_LOOP`** (`classic`, `multi_point`, `joint_space`): plan once, stream the interpolated trajectory to the robot, succeed when it ends. Feedback carries `step_progression` (0→1).
- **`CLOSED_LOOP`** (`mpc`, `retarget`): the action starts a servo loop that re-solves continuously, tracks `position_error`, and **keeps running after reaching the goal** (feedback `on_target: true`, state `ON_TARGET`). Retarget the goal live by publishing to `/unified_planner/mpc_goal`; stop by cancelling the goal. See [MPC Implementation](mpc-implementation.md).

## Two base classes

Open-loop planners subclass `SinglePlanner`, which owns a **single, class-level cuRobo `MotionPlanner`** — all three open-loop planners share the same solver instance and its warmup.

Closed-loop planners subclass `ReactiveController`, which owns the whole servo loop (goal admission, live-goal plumbing, periodic perception refresh, command pacing); a concrete controller only implements five hooks: `build_solver()`, `setup()`, `step()`, `apply_live_goal()`, `is_converged()`.

Adding a planner means subclassing one of the two bases and adding one line to the factory catalog — the `set_planner`/`get_planners` services, the action, and the shared context all work unchanged.

## Switching planners at runtime

```bash
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
```

What happens on a switch:

- Planner instances are **cached**: the first switch to a planner builds and warms up its solver (for MPC this takes several seconds); subsequent switches are fast.
- The switch is refused while an execution goal is active.
- With `use_cuda_graph: true` the node keeps **at most one live CUDA graph**: switching between solver families releases the previous graph and re-captures on the next solve. This is why the first plan after a switch can be slower.

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
├── mpc_planner.py           # MPCController
├── retarget_controller.py   # RetargetController
└── planner_factory.py       # catalog + PlannerManager
```

## Related pages

- [Architecture](architecture.md) — how the planner layer fits the whole node
- [MPC Implementation](mpc-implementation.md) — the closed-loop path in depth
- [Parameters](parameters.md) — `planner_type`, `mpc_*`, `retarget_*`, `trajectory_cache_ttl`
- [Tutorial 5](../tutorials/05-mpc-planner.md) — hands-on MPC

# Reactive Control (MPC) — Implementation Guide

> **For developers.** How reactive (closed-loop) control is wired in curobo_ros,
> on top of cuRobo v2. Companion to the [Unified Planner](unified-planner.md)
> architecture and the [MPC Planner tutorial](../tutorials/05-mpc-planner.md).

## Design principle

curobo_ros is a **thin ROS interface to cuRobo**. cuRobo models reactive control
and motion generation as **different wrappers over the same core** (a shared robot
model + world collision model; see
<https://nvlabs.github.io/curobo/latest/concepts/index.html>). They differ only by
horizon and update frequency, not by structure.

We mirror that with two planner families that share **one switch** (`SetPlanner` /
`GetPlanners`) and **one context**:

```
TrajectoryPlanner (abstract Strategy)
├── SinglePlanner            # open-loop: plan a full trajectory, then execute
│   ├── ClassicPlanner
│   ├── MultiPointPlanner
│   └── JointSpacePlanner
└── ReactiveController       # closed-loop: a control loop, no precomputed traj
    ├── MPCController         # cuRobo ModelPredictiveControl
    └── RetargetController    # cuRobo MotionRetargeter (IK teleop follower)
```

## Single shared context

Every solver (MotionPlanner, IK, **and** the MPC solver) is built from the **same**
context owned by `ConfigWrapperMotion`:

- robot YAML (`robot_config_file`),
- the shared `Scene` (`obstacle_manager.get_scene()`),
- the shared `collision_cache`,
- device/dtype.

There is **no** separate world/manager stack for MPC. Switching to/from MPC keeps
the exact same robot and obstacles, and a `set_collision_cache` change rebuilds all
solvers from the one cache (no manual copy). `node.mpc` is the single reference to
the built solver, used by `update_all_solvers_world()` and the collision-distance
service.

## `ReactiveController` — the shared loop

`curobo_ros/planners/reactive_controller.py` implements **all** the ROS / robot /
perception plumbing of the control loop once:

- cancellation via the action goal handle,
- throttled perception refresh (`node.refresh_perception_world`, every N steps),
- live goal consumption (`latest_goal`, fed by the `mpc_goal` topic),
- read robot state → `step()` → send action → publish feedback → convergence test,
- robot stop + per-step timing.

A concrete controller implements only the cuRobo-specific hooks:

| Hook | Responsibility |
|---|---|
| `build_solver()` | create the cuRobo solver from the shared context; publish `node.mpc` |
| `setup(state, goal)` | set the initial goal on the solver |
| `step(state)` | one optimization step → next action; update `_last_position_error` |
| `apply_live_goal(raw)` | retarget the goal during execution |
| `is_converged()` | stop condition (default: position error < threshold) |

## `MPCController` — cuRobo lifecycle

`curobo_ros/planners/mpc_planner.py` is a thin wrapper over cuRobo's canonical
reactive lifecycle:

```python
cfg = ModelPredictiveControlCfg.create(
    robot=robot_yaml, scene_model=scene, collision_cache=cache,
    optimization_dt=mpc_step_dt, num_control_points=mpc_horizon_steps,
)
mpc = ModelPredictiveControl(cfg)
mpc.setup(current_state)
mpc.update_goal_tool_poses({ee_link: pose})     # + Cartesian-only fallback on IK fail
# loop:
result = mpc.optimize_next_action(current_state) # result.next_action / .position_error
mpc.update_world(scene)                           # dynamic obstacles (driven by the node)
```

### Two solver recipes

`mpc_solver_type` selects how the underlying optimizer is configured:

- **`mppi_acceleration`** (default) — a hand-tuned MPPI configuration in
  ACCELERATION control space (`_build_mppi_optimizer_config`), validated on the
  real Doosan M1013 (2026-07). Tuned companions: `mpc_warm_start_iters: 5`,
  `mpc_cold_start_iters: 10`, `mpc_mppi_num_particles: 400`.
- **`lbfgs_bspline`** — cuRobo's stock L-BFGS + B-spline MPC config. If you use
  it, iteration counts must be multiples of 25 (e.g. 25/100).

### Command pacing and stability

Two mechanisms keep the real robot stable:

- `mpc_command_interval` (default `0.24` s) paces the loop in fixed windows:
  each command window fully executes on the robot before the next solve, and the
  robot state is read *after* execution (fresh and velocity-consistent). `0`
  reverts to free-running solves.
- A velocity boundary-continuity cap (`_VBC_CAP_DPS = 5.0` deg/s in
  `mpc_planner.py`) limits the velocity discontinuity at window boundaries to
  avoid ratcheting/runaway.

### Goal state

The Cartesian goal is converted to a joint-space goal with a lazy,
self-collision-only IK solve (`_solve_goal_state`), seeded from the current
configuration; on IK failure the controller falls back to a Cartesian-only goal.

## Monitoring and diagnostics

While MPC (or retarget) is active the node publishes:

| Topic | Type | Content |
|---|---|---|
| `/mpc_predicted_path` | `nav_msgs/Path` | Near-term predicted end-effector path |
| `/mpc_goal_marker` | `visualization_msgs/Marker` | Current goal (color indicates whether it was applied) |
| `/mpc_costs` | `curobo_msgs/msg/MpcCosts` | FK error, solver pose error, per-term cost and constraint breakdown |

Set `mpc_debug: true` to also write per-step CSVs to the ROS log directory
(analyze offline with `scripts/plot_mpc_diag.py`).

## `RetargetController` — teleoperation

`curobo_ros/planners/retarget_controller.py` wraps cuRobo's `MotionRetargeter`:
a pose-stream follower for teleoperation. The first frame runs a global IK
(64 seeds); subsequent frames run warm-started local IK for low latency. Weights:
`retarget_position_weight` / `retarget_orientation_weight`; `retarget_use_mpc`
routes commands through the MPC solver instead. Select it with `set_planner`
enum `6` and stream poses to `/unified_planner/mpc_goal`.

## ROS interface

Reactive control reuses the **unified** interface — no dedicated action:

- `set_planner` → switch to `mpc` (enum `1`).
- `generate_trajectory` → sets the reactive goal (`plan()`); returns no trajectory.
- `execute_trajectory` (`SendTrajectory` action) → drives the control loop with
  feedback / cancellation.
- `mpc_goal` topic (`geometry_msgs/Pose`) → live retargeting during execution; this
  is the ROS mapping of cuRobo's continuous `update_goal_tool_poses`.

## ROS parameters

| Parameter | Default | Use |
|---|---|---|
| `convergence_threshold` | `0.01` | stop servoing to "on target" when position error (m) is below this |
| `max_mpc_iterations` | `1000` | safety cap on loop iterations |
| `mpc_solver_type` | `'mppi_acceleration'` | solver recipe (see above) |
| `mpc_step_dt` | `0.03` | cuRobo `optimization_dt` |
| `mpc_horizon_steps` | `30` | cuRobo `num_control_points` |
| `mpc_command_interval` | `0.24` | fixed command-window pacing (s); `0` = free-running |

The full family (`mpc_warm_start_iters`, `mpc_mppi_num_particles`,
`mpc_vel_feedback_alpha`, `retarget_*`, …) is listed in
[Parameters](parameters.md). MPC parameters are read when the solver is built —
set them before the first switch to `mpc`.

## Adding another reactive control

1. Subclass `ReactiveController`, implement the five hooks above.
2. Add one entry to `PlannerFactory._PLANNER_CATALOG`.

It then works with the same `SetPlanner` switch, the same shared context, and the
same `execute_trajectory` / `mpc_goal` interface — no node changes required.

## References

- [cuRobo concepts](https://nvlabs.github.io/curobo/latest/concepts/index.html)
- [Unified Planner](unified-planner.md) · [MPC tutorial](../tutorials/05-mpc-planner.md)
- Canonical code: `curobo_ros/planners/reactive_controller.py`,
  `curobo_ros/planners/mpc_planner.py`

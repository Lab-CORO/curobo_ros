# Tutorial 5: MPC and Reactive Control

**Difficulty**: Advanced · **Time**: ~40 min · **Prerequisites**: [Tutorial 1](01-first-trajectory.md), ideally [Tutorial 4](04-robot-execution.md)

The open-loop planners compute a trajectory once and replay it. **MPC (Model Predictive Control)** closes the loop: at every step it re-optimizes a short receding-horizon plan from the *current* robot state, executes the first bit, and repeats. The result: the robot tracks a goal you can move **live**, and reacts to obstacles that appear mid-motion.

## What you get with MPC in curobo_ros

- A servo loop driven by the same `execute_trajectory` action as everything else — no dedicated interface to learn.
- Live goal retargeting through the `/unified_planner/mpc_goal` topic.
- Continuous perception refresh: camera obstacles ([Tutorial 7](07-pointcloud-detection.md)) are re-read during motion.
- Monitoring topics: predicted path, goal marker, cost breakdown.

## 1. Start and switch to MPC

```bash
ros2 launch curobo_ros gen_traj.launch.py robot:=emulator
# wait for node_is_available == True, then:
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
```

The **first** switch to MPC builds and warms up its solver from the shared context — expect several seconds. Later switches are fast (instances are cached).

MPC parameters are read when the solver is built; if you want to change them (e.g. `mpc_solver_type`), set them **before** the first switch. Defaults are the validated `mppi_acceleration` recipe — see the [Parameters Guide](../concepts/parameters.md).

## 2. Send a goal

```bash
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}" --feedback
```

Watch the feedback: `state` goes `PLANNING` → `TRACKING`, `position_error` shrinks, and when it drops under `convergence_threshold` (default 1 cm) you get `on_target: true` with state `ON_TARGET`. Unlike the open-loop planners, **the action does not finish there** — the controller keeps servoing on the target until you cancel the goal (Ctrl-C on the CLI, or `stop robot` in RViz).

## 3. Retarget the goal live

While the action is running, publish a new pose:

```bash
ros2 topic pub --once /unified_planner/mpc_goal geometry_msgs/msg/Pose \
  "{position: {x: 0.5, y: -0.3, z: 0.5}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}"
```

The robot bends its current motion toward the new goal without stopping. Publish continuously (e.g. from a perception node tracking an object) and the arm follows. Messages on `mpc_goal` are ignored (with a warning) when no reactive planner is active.

The most intuitive way to feel this is the RViz plugin: with MPC selected, sending a goal makes the panel stream the interactive arrow's pose to `mpc_goal` at 10 Hz — drag the arrow and the robot follows. See [RViz Plugin](../concepts/rviz-plugin.md).

## 4. React to obstacles

While the robot is tracking, add an obstacle in its path ([Tutorial 3](03-collision-objects.md)):

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 1, name: 'intruder', pose: {position: {x: 0.45, y: 0.0, z: 0.45}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.12, y: 0.12, z: 0.12}, color: {r: 1.0, a: 1.0}}"
```

The predicted path deforms around it on the next solve. With a depth camera configured, the same happens for anything entering the field of view — no service call needed.

## 5. Monitor

| Topic | Type | Shows |
|---|---|---|
| `/mpc_predicted_path` | `nav_msgs/Path` | The near-term predicted end-effector path (add it in RViz) |
| `/mpc_goal_marker` | `visualization_msgs/Marker` | The current goal (colored by whether it was accepted) |
| `/mpc_costs` | `curobo_msgs/msg/MpcCosts` | FK error, solver pose error, cost/constraint terms per step |

```bash
ros2 topic echo /mpc_costs
```

Rising `con_scene_collision` while tracking means the goal region conflicts with an obstacle. For deep debugging, set `mpc_debug: true` to write per-step CSVs to the ROS log directory (`scripts/plot_mpc_diag.py` plots them).

## 6. The other reactive planner: `retarget`

For teleoperation-style *pose following* without the MPC optimization, switch to the retargeting controller:

```bash
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 6}"
```

Then run the action and stream poses to `/unified_planner/mpc_goal` exactly as above. It solves warm-started IK per frame — lower latency, but no predictive obstacle avoidance along the horizon. Use MPC when the scene is dynamic, retarget when you need snappy manual guidance.

## MPC vs Classic — when to use which

| | Classic (open-loop) | MPC (closed-loop) |
|---|---|---|
| Static scene, repeatable motion | ✔ best trajectory quality | works, but unnecessary |
| Moving goal | ✘ re-plan each time | ✔ tracks continuously |
| Obstacles appearing mid-motion | ✘ collision or abort | ✔ deforms around them |
| Compute load | one solve | continuous GPU load |

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Switch to MPC takes long | First-time solver build — expected |
| `mpc_goal` messages ignored | No reactive planner active, or no action goal running |
| Sluggish tracking | Check `mpc_command_interval` (0.24 s pacing by default); see [MPC Implementation](../concepts/mpc-implementation.md) |
| Arm oscillates near target on hardware | Keep the validated `mppi_acceleration` defaults; check the driver executes velocities faithfully |

## Next steps

- [MPC Implementation](../concepts/mpc-implementation.md) — how the loop works inside
- [Tutorial 7: Camera-Based Obstacle Detection](07-pointcloud-detection.md) — make the world dynamic

[← Tutorial 4](04-robot-execution.md) | [Tutorial 6 →](06-ik-fk-services.md)

# Tutorial 4: Robot Execution

**Difficulty**: Intermediate · **Time**: ~30 min · **Prerequisites**: [Tutorial 1](01-first-trajectory.md)

curobo_ros separates *planning* from *execution*, and separates **which robot** you plan for from **how commands reach it**. This tutorial explains the two axes, the built-in emulator, and how the real Doosan M1013 is wired.

## Two orthogonal axes

| Axis | Parameter | Values | Selects |
|---|---|---|---|
| Which robot | `robot` (launch arg) | `doosan_m1013`, `emulator`, your own | The descriptor `robots/<name>.yaml`: URDF, cuRobo config, driver topics |
| How to command it | `control_strategy` | `emulator`, `joint_speed`, `joint_pose` | The `JointCommandStrategy` implementation |

The descriptor sets the *default* strategy (`strategy:` key), and you can switch strategies at runtime without restarting.

## The control strategies

### `emulator` — no hardware

Simulates execution: replays the planned trajectory on a thread and publishes `/emulator/joint_states` (`sensor_msgs/JointState`), which the launch file feeds into `robot_state_publisher` — the RViz robot moves as if real. Always start here.

### `joint_speed` — velocity streaming (the real M1013 path)

Streams `trajectory_msgs/JointTrajectory` chunks *with velocities* to a driver bridge, applies a hard acceleration clamp, and reads back real joint states and progress. Topics come from the descriptor's `strategy_params` — for the lab's M1013 (robot name `leeloo`):

| Topic | Type | Direction |
|---|---|---|
| `/leeloo/execute_trajectory` | `trajectory_msgs/JointTrajectory` | → driver bridge |
| `/leeloo/trajectory_state` | `std_msgs/Float32` (progress 0→1) | ← driver bridge |
| `/dsr01/joint_states` | `sensor_msgs/JointState` | ← Doosan driver |

This is also the strategy used by closed-loop planners (MPC/retarget) on the real robot.

### `joint_pose` — position streaming

Same wiring as `joint_speed` but streams positions only. For drivers without a velocity interface; motion is less smooth.

### The ghost preview (always on)

Independently of the active strategy, every plan is published on `/trajectory` and replayed by the translucent preview robot (namespace `preview/`) in RViz. It is not a strategy you select — it is always there, and it never moves hardware.

## Switching strategies at runtime

```bash
# What is available and active?
ros2 service call /unified_planner/get_robot_strategies curobo_msgs/srv/GetRobotStrategies
ros2 service call /unified_planner/get_robot_strategy std_srvs/srv/Trigger

# Switch (string key, from get_robot_strategies)
ros2 service call /unified_planner/set_robot_strategy curobo_msgs/srv/SetRobotStrategy \
  "{robot_strategy: 'emulator'}"
```

Typical workflow: plan and execute on `emulator`, inspect the motion, then switch to `joint_speed` and run the *same* action goal on hardware.

## Executing

Execution always goes through the action ([Tutorial 1](01-first-trajectory.md), [ROS Interfaces](../concepts/ros-interfaces.md)):

```bash
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}" --feedback
```

Feedback carries `state`, `step_progression`, and the streamed `joint_command`. Cancel the goal to stop; the strategy's `stop_robot()` halts streaming.

Safety notes for hardware:

- Keep the physical E-stop within reach; action cancellation is a software stop.
- **Robot speed is set in the robot YAML** (cspace velocity/acceleration/jerk limits), not by a ROS parameter — scale them down for first runs, then `update_motion_gen_config`. The `time_dilation_factor` parameter is only a feedback cadence in v2.
- The world the planner avoids is only what you gave it ([Tutorial 3](03-collision-objects.md), [Tutorial 7](07-pointcloud-detection.md)) — the real table is not an obstacle until it is in the scene.

## Connecting your own robot

1. Write the descriptor with your driver's topics ([Tutorial 2](02-adding-your-robot.md)):

   ```yaml
   strategy: joint_speed
   strategy_params:
     command_topic: /my_robot/trajectory_command
     state_topic: /my_robot/trajectory_progress
     joint_states_topic: /my_robot/joint_states
   ```

2. Your driver side must consume `JointTrajectory` chunks and report progress as a `Float32` (0→1) — the `leeloo` bridge is the reference implementation.
3. Joint *ordering* between your driver and the cuRobo cspace must match; remap in your bridge if the driver uses a different order.
4. Validate on `emulator` first, then dry-run `joint_speed` with the arm in free space.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Action succeeds but nothing moves | Active strategy is `emulator`, or the driver bridge isn't subscribed to the command topic |
| Robot moves but RViz robot doesn't | `robot_state_publisher` isn't receiving the driver's joint states |
| Jerky motion on hardware | Use `joint_speed` rather than `joint_pose`; check the driver honors velocities |
| Motion too fast | Scale down cspace velocity/acceleration limits in the robot YAML and rebuild |

## Next steps

- [Tutorial 5: MPC and Reactive Control](05-mpc-planner.md) — closed-loop execution
- [Package Architecture](../concepts/architecture.md) — how strategies fit the node

[← Tutorial 3](03-collision-objects.md) | [Tutorial 5 →](05-mpc-planner.md)

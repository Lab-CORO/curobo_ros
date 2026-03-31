# Tutorial 4: Robot Execution — Connecting curobo_ros to Your Robot

🟡 **Difficulty**: Intermediate
⏱️ **Estimated Time**: 20-30 minutes

## Overview

curobo_ros separates trajectory **planning** from trajectory **execution**. The planner generates a collision-free `JointTrajectory` — a robot driver then executes it. This tutorial explains how execution works, how to test with the built-in emulator, and how the Doosan M1013 driver is connected.

---

## The Execution Pipeline

```
generate_trajectory service
       │  JointTrajectory (positions, velocities, accelerations, timestamps)
       ▼
RobotContext  ←── Strategy pattern: ghost / emulator / doosan / ur5e
       │
       ├─► GhostStrategy  → /trajectory  (RViz preview, always active)
       │
       └─► Active Strategy
              ├─ EmulatorStrategy  → /emulator/joint_states  (JointState, progressive)
              └─ DoosanControl     → /leeloo/execute_trajectory  (JointTrajectory, full)
```

The active strategy is selected at runtime. All strategies share the same interface — `generate_trajectory` never needs to know which robot is connected.

---

## Available Strategies

| Strategy | Status | Output Topic | Description |
|----------|--------|--------------|-------------|
| `ghost` | Always active | `/trajectory` | RViz preview only, no execution |
| `emulator` | Implemented | `/emulator/joint_states` | Software emulator, progressive execution |
| `doosan_m1013` | Implemented | `/leeloo/execute_trajectory` | Real Doosan M1013 via driver |
| `ur5e` | Placeholder | — | Not yet implemented |

---

## Section 1: Ghost Mode (Visualization Only)

Ghost mode is the default. Trajectories are displayed in RViz as a "preview robot" but nothing executes. It is always running in parallel, regardless of the active strategy.

```bash
ros2 param set /unified_planner robot_type "ghost"
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger

ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0}}}"
```

The preview robot in RViz (`/preview/` namespace) shows the planned trajectory immediately.

---

## Section 2: Emulator (Test Without Hardware)

The emulator replays the trajectory by publishing `JointState` messages progressively at the planned timestep. The main robot in RViz moves step by step, simulating real execution.

### Switch to Emulator

```bash
ros2 param set /unified_planner robot_type "emulator"
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger
```

**Response:**
```yaml
success: true
message: "Strategy switched from 'ghost' to 'emulator'"
```

### Generate and Execute

```bash
# 1. Plan
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0}}}"

# 2. Execute
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```

**Feedback** during execution:
```yaml
step_progression: 0.25   # 25% complete
```

**Result:**
```yaml
success: true
message: 'Trajectory executed successfully'
```

### What the Emulator Publishes

| Topic | Type | Description |
|-------|------|-------------|
| `/emulator/joint_states` | `sensor_msgs/JointState` | Current simulated joint positions, updated at each trajectory timestep |

The emulator reads back its own joint states as the "current robot position" for the next planning cycle.

---

## Section 3: Real Robot — Doosan M1013

### How the Doosan Driver Works

The Doosan strategy connects curobo_ros to the Doosan ROS driver (`dsr_robot` package). The interface is **trajectory-based**:

1. curobo_ros generates a `JointTrajectory` with positions, velocities, accelerations, and timestamps for every waypoint
2. The strategy publishes the full trajectory to `/leeloo/execute_trajectory`
3. The Doosan driver executes it via **velocity control** — it follows each waypoint with the specified velocity and acceleration profile
4. The driver publishes execution progress on `/leeloo/trajectory_state` (a `Float32` from 0.0 to 1.0)
5. curobo_ros reads this progress to report feedback through the `execute_trajectory` action

```
curobo_ros
  └─ DoosanControl
       ├── publishes → /leeloo/execute_trajectory  (JointTrajectory)
       └── subscribes ← /leeloo/trajectory_state   (Float32, 0.0→1.0)

Doosan Driver (dsr_robot)
  ├── subscribes ← /leeloo/execute_trajectory
  ├── executes via velocity control
  └── publishes → /dsr01/joint_states  (JointState, current positions)
```

> **Note**: The topic prefix `leeloo` is the name of the specific Doosan M1013 in the Lab-CORO. If your robot uses a different prefix, the driver configuration must be updated accordingly.

### Joint State Remapping

The Doosan driver publishes joints in a non-standard order. The `DoosanControl` strategy handles this remapping internally:

```
Driver order:  [j1, j2, j3, j4, j5, j6]  (indices 0,1,4,2,3,5)
cuRobo order:  [j1, j2, j5, j3, j4, j6]
```

This is transparent — you never need to handle it manually.

### Prerequisites

Before using the Doosan strategy:

1. The `dsr_robot` ROS2 driver must be running and connected to the robot
2. The driver must be publishing to `/dsr01/joint_states`
3. The driver must be subscribed to `/leeloo/execute_trajectory`

```bash
# Verify driver is running
ros2 topic hz /dsr01/joint_states   # Should show ~100 Hz

# Verify execution topic exists
ros2 topic list | grep execute_trajectory
```

### Switch to Doosan

```bash
ros2 param set /unified_planner robot_type "doosan_m1013"
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger
```

### Generate and Execute

```bash
# 1. Plan
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"

# 2. Execute
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```

> **Safety**: Always verify the trajectory in ghost or emulator mode before executing on the real robot. Make sure the workspace is clear.

---

## Section 4: Switching Strategies

All switches follow the same pattern:

```bash
# Check current strategy
ros2 service call /unified_planner/get_robot_strategy std_srvs/srv/Trigger

# Switch
ros2 param set /unified_planner robot_type "<strategy>"
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger
```

**On switch:**
- The current robot is stopped immediately
- Any trajectory in progress is cancelled
- The new strategy initializes its topics and subscriptions

---

## Section 5: Adding a New Robot (UR5e Pattern)

The UR5e driver is not yet implemented, but the architecture follows the same pattern as Doosan. Any new robot strategy must:

1. Subscribe to the robot's joint state topic to read current positions
2. Publish a `JointTrajectory` (or equivalent) to the robot driver
3. Track execution progress and report it via `get_progression()`

The control mode depends on the robot driver — the Doosan uses **position + velocity + acceleration** per waypoint with `time_from_start`. A UR5e would likely use the same `JointTrajectory` format via the `ur_robot_driver` package.

---

## Troubleshooting

### Strategy switch fails with `ur5e`

`ur5e` is a placeholder. Use `emulator` for testing without hardware.

### Robot doesn't move after `execute_trajectory`

1. Verify the strategy is active: `ros2 service call /unified_planner/get_robot_strategy std_srvs/srv/Trigger`
2. For emulator: check `/emulator/joint_states` is publishing — `ros2 topic hz /emulator/joint_states`
3. For Doosan: check the driver is running and `/leeloo/execute_trajectory` has a subscriber — `ros2 topic info /leeloo/execute_trajectory`

### Doosan executes but position is wrong

Check that `robot_config_file` matches the physical robot (joint limits, kinematics). Verify `/dsr01/joint_states` matches the robot's actual pose before planning.

---

## Related Documentation

- [Architecture — Robot Strategy Pattern](../concepts/manager-architecture.md#62-robot-strategy)
- [Tutorial 1: Your First Trajectory](01-first-trajectory.md)
- [Tutorial 5: MPC Planner](05-mpc-planner.md)

---

[← Tutorial 3: Collision Objects](03-collision-objects.md) | [Tutorial 5: MPC Planner →](05-mpc-planner.md)

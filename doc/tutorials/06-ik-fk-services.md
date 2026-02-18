# Tutorial 6: Inverse and Forward Kinematics Services

🟢 **Difficulty**: Beginner
⏱️ **Estimated Time**: 20-30 minutes

## Overview

This tutorial covers how to use the Forward Kinematics (FK) and Inverse Kinematics (IK) services in curobo_ros. These services are essential tools for:
- **FK**: Computing end-effector pose from joint positions
- **IK**: Computing joint configurations that achieve a desired end-effector pose

Unlike full trajectory planning, these services provide quick kinematic solutions without generating collision-free paths.

**For detailed information about cuRobo's kinematics capabilities and underlying algorithms, visit [curobo.org](https://curobo.org) - the official cuRobo documentation.**

## When to Use IK/FK vs Motion Planning

| Use Case | FK Service | IK Service | Motion Planning |
|----------|-----------|-----------|-----------------|
| Verify current robot pose | ✅ | ❌ | ❌ |
| Check if pose is reachable | ❌ | ✅ | ❌ |
| Find joint configuration for target | ❌ | ✅ | ❌ |
| Generate collision-free trajectory | ❌ | ❌ | ✅ |
| Plan motion from A to B | ❌ | ❌ | ✅ |

**Key Difference**: IK/FK services perform kinematic calculations only. Use `generate_trajectory` service for collision-free motion planning.

## Prerequisites

Before starting this tutorial, ensure you have:
- Completed [Tutorial 1: Your First Trajectory](01-first-trajectory.md)
- curobo_ros installed and running
- Robot configured (if using your own robot, see [Tutorial 2](02-adding-your-robot.md))
- Basic understanding of ROS 2 services

## Section 1: Forward Kinematics (FK)

### 1.1 What is Forward Kinematics?

Forward Kinematics computes the end-effector pose (position + orientation) given a set of joint positions.

**Input**: Joint angles [j1, j2, j3, j4, j5, j6]
**Output**: End-effector pose (x, y, z, qx, qy, qz, qw)

**Use Cases**:
- Verify robot's current pose after motion
- Validate joint configurations before execution
- Compute workspace reach from joint limits

### 1.2 Using the FK Service

**Service Interface:**
- **Service Name**: `<node_name>/fk_compute`
- **Service Type**: `curobo_msgs/srv/Fk`
- **Request**: `sensor_msgs/JointState` (joint positions)
- **Response**: `geometry_msgs/Pose` (end-effector pose)

### 1.3 FK Service Examples

#### Example 1: Compute FK for Home Position

```bash
# FK for home position (all zeros for Doosan M1013)
ros2 service call /unified_planner/fk_compute curobo_msgs/srv/Fk \
  "{joint_state: {position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}"
```

**Expected Output:**
```yaml
success: true
message: 'Forward kinematics computed successfully'
end_effector_pose:
  position:
    x: 0.796
    y: 0.0
    z: 0.413
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

#### Example 2: Compute FK for Specific Configuration

```bash
# FK for a bent-arm configuration
ros2 service call /unified_planner/fk_compute curobo_msgs/srv/Fk \
  "{joint_state: {position: [0.0, -0.785, 1.57, 0.0, 1.57, 0.0]}}"
```

**Expected Output:**
```yaml
success: true
message: 'Forward kinematics computed successfully'
end_effector_pose:
  position:
    x: 0.452
    y: 0.0
    z: 0.627
  orientation:
    x: 0.0
    y: 0.707
    z: 0.0
    w: 0.707
```

### 1.4 Understanding FK Response

The response contains:
- **success** (bool): `true` if FK computation succeeded
- **message** (string): Status message
- **end_effector_pose** (Pose):
  - **position** (x, y, z): Cartesian coordinates in meters (base frame)
  - **orientation** (x, y, z, w): Quaternion representation

**Coordinate Frame**: All poses are in the robot's base frame (typically `base_0` or `base_link`).

---

## Section 2: Inverse Kinematics (IK)

### 2.1 What is Inverse Kinematics?

Inverse Kinematics computes joint configurations that achieve a desired end-effector pose.

**Input**: Target pose (x, y, z, qx, qy, qz, qw)
**Output**: Joint configuration

**Use Cases**:
- Check if target pose is reachable
- Find joint configuration for specific end-effector pose
- Validate poses before planning

### 2.2 IK Service Interface

**Service Interface:**
- **Service Name**: `<node_name>/ik_batch_poses`
- **Service Type**: `curobo_msgs/srv/Ik`
- **Request**:
  - `pose` (Pose): Target end-effector pose
- **Response**:
  - `success` (bool): True if solution found
  - `joint_states` (JointState): IK solution
  - `joint_states_valid` (Bool): Validity flag
  - `error_msg` (String): Error message if failed

### 2.3 IK Service Examples

#### Example 1: Single Pose IK

```bash
# IK for a target pose
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

**Expected Output:**
```yaml
success: true
joint_states:
  position: [0.523, -0.234, 1.245, -0.523, 1.456, 0.234]
  velocity: []
  effort: []
joint_states_valid:
  data: true
error_msg:
  data: ''
```

**Interpretation**:
- **success**: `true` means a solution was found
- **joint_states**: The joint configuration that achieves the target pose
- **joint_states_valid**: `true` confirms the solution is valid

#### Example 2: Check if Pose is Reachable

```bash
# Try IK for a potentially unreachable pose (far from robot)
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{pose: {position: {x: 2.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"
```

**Expected Output (if unreachable):**
```yaml
success: false
joint_states:
  position: []
joint_states_valid:
  data: false
error_msg:
  data: 'IK failed: Pose is outside workspace'
```

**Interpretation**: `success: false` and `joint_states_valid: false` indicate no valid solution found - the pose is unreachable.

#### Example 3: IK for Different Orientations

```bash
# IK with pointing down orientation (end-effector pointing downward)
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
```

**Expected Output:**
```yaml
success: true
joint_states:
  position: [0.234, -0.567, 1.456, -0.345, 1.678, 0.123]
joint_states_valid:
  data: true
error_msg:
  data: ''
```

---

## Section 3: Troubleshooting

### Issue 1: IK Fails to Find Solution

**Symptoms**: `success: false`, `joint_states_valid: false`

**Possible Causes**:
1. **Pose is outside workspace** → Reduce distance from base, check z-height
2. **Invalid orientation** → Check quaternion is normalized (x²+y²+z²+w²=1)
3. **Collision with obstacles** → Remove obstacles or adjust pose

**Solution**:
```bash
# Verify pose is within reasonable workspace bounds
# For typical 6-DOF arm: reach is typically 0.3m to 1.0m from base

# Try a known reachable pose
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
```

### Issue 2: FK Returns Unexpected Pose

**Symptoms**: End-effector pose doesn't match expected position

**Possible Causes**:
1. **Joint order mismatch** → Verify joint positions match robot's joint order
2. **Wrong coordinate frame** → Check base_link parameter
3. **Joint values out of range** → Check joint limits in robot config

**Solution**:
```bash
# Verify joint names and order
ros2 topic echo /joint_states --once

# Use exact joint order from topic
ros2 service call /unified_planner/fk_compute curobo_msgs/srv/Fk \
  "{joint_state: {position: [j1, j2, j3, j4, j5, j6]}}"
```

### Issue 3: Service Not Available

**Symptoms**: `Service not available: /unified_planner/fk_compute`

**Solution**:
```bash
# Check if unified_planner node is running
ros2 node list | grep unified_planner

# List available services
ros2 service list | grep -E "(fk_compute|ik_batch)"

# Restart the planner
ros2 launch curobo_ros gen_traj.launch.py
```

### Issue 4: Quaternion Normalization Error

**Symptoms**: IK returns error about invalid orientation

**Solution**:
```bash
# Ensure quaternion is normalized
# For identity orientation (no rotation), use: {x: 0, y: 0, z: 0, w: 1}
# For 180° rotation around Y: {x: 0, y: 1, z: 0, w: 0}

ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

---

## Section 4: Comparison with Motion Planning

| Feature | IK/FK Services | Motion Planning |
|---------|---------------|-----------------|
| **Speed** | < 10 ms | 50-500 ms |
| **Output** | Joint config or pose | Full trajectory |
| **Collision checking** | No | Always included |
| **Path planning** | No | Yes |
| **Use for execution** | No (config only) | Yes (trajectory) |
| **Typical use** | Validation, calibration | Robot motion |

**Rule of Thumb**: Use IK/FK for quick kinematic checks. Use `generate_trajectory` for actual robot motion.

---

## Next Steps

Now that you understand IK/FK services, you can:

- **Integrate with motion planning**: Use IK to validate poses before calling `generate_trajectory`
- **Implement pose validation**: Check reachability before attempting robot motion
- **Add collision detection**: Combine IK with obstacle management ([Tutorial 3](03-collision-objects.md))
- **Explore point clouds**: Use camera integration for dynamic obstacles ([Tutorial 7](07-pointcloud-detection.md))
- **Learn more about cuRobo**: Visit [curobo.org](https://curobo.org) for in-depth kinematics documentation

---

## Related Documentation

- [Tutorial 1: Your First Trajectory](01-first-trajectory.md) - Basic motion planning
- [Tutorial 3: Collision Objects](03-collision-objects.md) - Obstacle management
- [ROS Interfaces Reference](../concepts/ros-interfaces.md) - Complete service API
- [curobo_msgs Package](https://github.com/Lab-CORO/curobo_msgs) - Message definitions
- [cuRobo Official Documentation](https://curobo.org) - Detailed kinematics algorithms

---

[← Back to Tutorials](README.md) | [Continue to Tutorial 7: Point Cloud Detection →](07-pointcloud-detection.md)

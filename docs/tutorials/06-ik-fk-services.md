# Tutorial 6: Inverse and Forward Kinematics Services

🟢 **Difficulty**: Beginner
⏱️ **Estimated Time**: 20-30 minutes

## Overview

This tutorial covers how to use the Inverse Kinematics (IK) and Forward Kinematics (FK) services on the unified planner node.

- **IK**: Compute joint configurations that achieve a desired end-effector pose
- **FK**: Compute end-effector pose from a set of joint positions

Both solvers are **lazy-initialized** — they are not created at startup. You must call the warmup service before using IK or FK. This allows the node to start quickly and avoids allocating GPU memory for solvers you don't need.

**For detailed information about cuRobo's kinematics capabilities and underlying algorithms, visit [curobo.org](https://curobo.org).**

---

## When to Use IK/FK vs Motion Planning

| Use Case | FK | IK | Motion Planning |
|----------|----|----|-----------------|
| Get end-effector pose from joint state | ✅ | ❌ | ❌ |
| Check if a pose is reachable | ❌ | ✅ | ❌ |
| Find joint config for a target pose | ❌ | ✅ | ❌ |
| Generate collision-free trajectory | ❌ | ❌ | ✅ |

**Key rule**: Use IK/FK for kinematic queries. Use `generate_trajectory` for actual robot motion.

---

## Prerequisites

- Completed [Tutorial 1: Your First Trajectory](01-first-trajectory.md)
- curobo_ros running with the unified planner
- Basic understanding of ROS 2 services

---

## Section 1: Forward Kinematics (FK)

### 1.1 What is Forward Kinematics?

Forward kinematics computes the end-effector pose from a given set of joint positions. It is a purely geometric computation — no collision checking, no obstacle dependency.

**Input**: Joint positions `[j1, j2, j3, j4, j5, j6]`
**Output**: End-effector pose `(x, y, z, qx, qy, qz, qw)` in the robot base frame

### 1.2 Warmup FK

Before calling `fk`, initialize the FK model:

```bash
ros2 service call /unified_planner/warmup_fk curobo_msgs/srv/WarmupFK "{batch_size: 1}"
```

The `batch_size` parameter tells the model how many joint states to expect per call, allowing CUDA kernels to be pre-compiled for that size. Unlike IK, FK does not require reinitialization when the batch size changes.

**Expected response:**
```yaml
success: true
message: 'FK model initialized with batch_size=1'
```

### 1.3 FK Service Examples

#### Example 1: Single joint configuration

```bash
ros2 service call /unified_planner/fk curobo_msgs/srv/Fk \
  "{joint_states: [{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}"
```

#### Example 2: Multiple joint configurations in one call

Warmup with the batch size you intend to use, then call `fk` with that many joint states:

```bash
ros2 service call /unified_planner/warmup_fk curobo_msgs/srv/WarmupFK "{batch_size: 3}"

ros2 service call /unified_planner/fk curobo_msgs/srv/Fk \
  "{joint_states: [
    {position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    {position: [0.0, -0.785, 1.57, 0.0, 1.57, 0.0]},
    {position: [0.5, -0.5, 0.5, 0.0, 1.0, 0.0]}
  ]}"
```

**Response:**
```yaml
poses:
  - position: {x: 0.796, y: 0.0, z: 0.413}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - position: {x: 0.452, y: 0.0, z: 0.627}
    orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
  - position: {x: ..., y: ..., z: ...}
    orientation: {...}
```

All poses are in the robot's base frame.

---

## Section 2: Inverse Kinematics (IK)

### 2.1 What is Inverse Kinematics?

Inverse kinematics computes a joint configuration that places the end-effector at a given pose. The IK solver in curobo_ros uses cuRobo's `IkSolver`, which:
- Performs **collision checking** (self-collision + obstacle avoidance)
- Shares the **same obstacle world** as the trajectory planners — obstacle changes are propagated automatically

**Input**: Target pose `(x, y, z, qx, qy, qz, qw)`
**Output**: Joint configuration or failure if no valid solution exists

### 2.2 Warmup IK

Before calling `ik` or `ik_batch`, initialize the IK solver:

```bash
ros2 service call /unified_planner/warmup_ik curobo_msgs/srv/WarmupIK "{batch_size: 1}"
```

The `batch_size` parameter pre-allocates GPU memory for that number of poses. If you later call `ik_batch` with a different number of poses, the solver reinitializes automatically (the first call will be slower). For best performance, warmup with the batch size you intend to use.

**Expected response:**
```yaml
success: true
message: 'IK solver initialized with batch_size=1'
```

### 2.3 Single Pose IK

Use the `ik` service for a single target pose:

```bash
ros2 service call /unified_planner/ik curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**Response (success):**
```yaml
success: true
joint_states:
  position: [0.523, -0.234, 1.245, -0.523, 1.456, 0.234]
joint_states_valid:
  data: true
error_msg:
  data: ''
```

**Response (failure — pose unreachable):**
```yaml
success: false
joint_states:
  position: []
joint_states_valid:
  data: false
error_msg:
  data: 'IK failed: no valid solution found'
```

### 2.4 Batch IK

Use `ik_batch` to solve multiple poses in one call. Warmup with the matching batch size first:

```bash
ros2 service call /unified_planner/warmup_ik curobo_msgs/srv/WarmupIK "{batch_size: 2}"

ros2 service call /unified_planner/ik_batch curobo_msgs/srv/IkBatch \
  "{poses: [
    {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}},
    {position: {x: 0.4, y: 0.2, z: 0.5}, orientation: {w: 1.0}}
  ]}"
```

**Response:**
```yaml
success: true
joint_states:
  - position: [0.523, -0.234, 1.245, -0.523, 1.456, 0.234]
  - position: [0.312, -0.445, 1.102, -0.234, 1.234, 0.456]
joint_states_valid:
  - data: true
  - data: true
```

### 2.5 IK with Obstacles

Because the IK solver shares the obstacle world with the trajectory planners, any obstacle you add via `add_object` is automatically reflected in IK solutions. No manual synchronization is needed.

For example, after adding a box obstacle that blocks a target pose, IK will return `success: false` for poses that collide with that obstacle.

---

## Section 3: Troubleshooting

### IK returns no solution

1. **Pose outside workspace** — check that the target is within the robot's reach (typically 0.3–1.0 m from base for a 6-DOF arm)
2. **Collision** — the IK solver checks collisions; an obstacle at the target pose will cause failure
3. **Invalid quaternion** — ensure the quaternion is normalized (`x²+y²+z²+w² = 1`)

Verify with a known reachable pose:
```bash
ros2 service call /unified_planner/ik curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
```

### Service not available

IK and FK solvers are not initialized until warmup is called. If you see `Service not available`, check that the node is running and that you have called the warmup service:

```bash
ros2 node list | grep unified_planner
ros2 service list | grep unified_planner
```

### Batch size mismatch (slower first call)

If you call `ik_batch` with a different number of poses than the warmup batch size, the solver reinitializes automatically. The first call will be slower. To avoid this, warmup with the batch size you intend to use most often.

---

## Section 4: Comparison

| Feature | FK | IK | Motion Planning |
|---------|----|-----|-----------------|
| Speed | < 5 ms | < 10 ms | 50–500 ms |
| Output | End-effector pose | Joint configuration | Full trajectory |
| Collision checking | No | Yes (self + obstacles) | Yes |
| Path planning | No | No | Yes |
| Suitable for execution | No | No | Yes |

**Rule of thumb**: Use IK/FK for validation and workspace checks. Use `generate_trajectory` for motion execution.

---

## Next Steps

- **Combine IK with planning**: Use IK to validate a pose is reachable before calling `generate_trajectory`
- **Add obstacles**: See [Tutorial 3: Collision Objects](03-collision-objects.md) — obstacles added there are automatically visible to the IK solver
- **Explore point clouds**: See [Tutorial 7: Point Cloud Detection](07-pointcloud-detection.md)

---

## Related Documentation

- [Unified Planner — IK and FK Services](../concepts/unified-planner.md#ik-and-fk-services)
- [ROS Interfaces — Kinematics Services](../concepts/ros-interfaces.md#kinematics-services)
- [Tutorial 3: Collision Objects](03-collision-objects.md)
- [cuRobo Official Documentation](https://curobo.org)

---

[← Tutorial 5: MPC Planner](05-mpc-planner.md) | [Tutorial 7: Point Cloud Detection →](07-pointcloud-detection.md)

# Tutorial: Your First Trajectory

This tutorial walks you through generating your first motion plan with curobo_ros. You'll learn how to:
- Request trajectory generation
- Understand the response
- Add obstacles
- Tune parameters for better performance
- Execute trajectories

---

## Prerequisites

- Completed [Getting Started Guide](../getting-started/installation.md)
- unified_planner node is running
- RViz is open

If not, start the system:
```bash
ros2 launch curobo_ros gen_traj.launch.py
```

---

## Step 1: Understanding the Workspace

Before planning, it's helpful to understand your robot's workspace.

### Check Robot Configuration

```bash
# See which robot is loaded
ros2 param get /unified_planner robot_config_file

# Example output: m1013.yml (Doosan M1013)
```

### Typical Workspace

For the default Doosan M1013 robot:
- **Reach**: ~1.3 m radius
- **Safe zone**: x: [0.3, 0.9], y: [-0.5, 0.5], z: [0.1, 0.8]
- **Base frame**: `base_0`

**Tip**: Look at the robot in RViz to get a sense of its reachable space.

---

## Step 2: Generate Your First Trajectory

Let's plan a simple trajectory to a target position!

### Open a New Terminal

```bash
# If using VSCode: Terminal → New Terminal (already inside container)
# If not: docker exec -it curobo_ampere_dev bash  # adapt name to your GPU

# Source the workspace
source /home/ros2_ws/install/setup.bash
```

### Call the Trajectory Service

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"
```

**What this does:**
- **Target position**: x=0.5m, y=0.0m, z=0.3m (in front of robot)
- **Target orientation**: Identity quaternion (no rotation)

### Watch in RViz

You should see:
1. **Main robot** (current state) - stays still
2. **Ghost robot** (namespace `/preview`) - shows the planned trajectory

### Expected Response

```yaml
success: true
message: "Trajectory generated successfully"
trajectory:
  # ... joint trajectory data ...
```

**Success!** 🎉 You've planned your first trajectory!

---

## Step 3: Understanding the Response

Let's break down what just happened:

### The Planning Process

```
1. Service call received with target pose
2. cuRobo solves IK: Finds joint angles for target
3. cuRobo plans trajectory: Smooth path from current to target
4. Trajectory interpolated: Dense waypoints for smooth motion
5. Ghost robot updated: Preview in RViz
6. Response sent: Success/failure + trajectory data
```

### What's in the Response

The `trajectory` field contains a `JointTrajectory` message:

```yaml
trajectory:
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      accelerations: [...]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.1, 0.05, ...]
      velocities: [0.5, 0.3, ...]
      time_from_start: {sec: 0, nanosec: 50000000}
    # ... more waypoints ...
```

**Each waypoint** includes:
- Joint positions (radians)
- Joint velocities (rad/s)
- Joint accelerations (rad/s²)
- Timestamp

---

## Step 4: Trying Different Targets

Let's explore the workspace by trying different positions.

### Target 1: Higher Position

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.6}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

### Target 2: To the Side

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.32, y: -0.44, z: 0.13}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

### Target 3: Unreachable (Should Fail)

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 2.0, y: 0.0, z: 0.5}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

**Expected**: `success: false`, `message: "IK solution not found"` or "Target unreachable"

---

## Step 5: Understanding Orientation

So far we've used identity orientation `(w=1, x=0, y=0, z=0)`. Let's try different orientations.

### Quaternion Basics

Quaternions represent 3D rotations:
- `w, x, y, z` with `w² + x² + y² + z² = 1`
- Identity: `(1, 0, 0, 0)` = no rotation
- 180° around Z: `(0, 0, 0, 1)`

**Tip**: Use online quaternion calculators or Python libraries:
```python
from scipy.spatial.transform import Rotation as R
# 90 degrees around Z
quat = R.from_euler('z', 90, degrees=True).as_quat()  # [x, y, z, w]
# Note: ROS uses [w, x, y, z] format!
```

### Example: 45° Rotation

```bash
# 45 degrees around Z axis
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 0.9239, x: 0.0, y: 0.0, z: 0.3827}}}"
```

---

## Step 6: Adding Obstacles

Now let's make it interesting by adding obstacles!

### Add a Box Obstacle

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{
  name: 'table',
  type: 0,
  pose: {position: {x: 0.4, y: 0.0, z: 0.15}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 0.4, y: 0.4, z: 0.4},
  color: {r: 0.0, g: 0.0, b: 0.0, a: 0.0}
}"
```

**What this creates:**
- A box representing a table
- Position: 0.4m in front, ground level
- Dimensions: 80cm x 100cm x 5cm


**Object types**:
- `0`: CUBOID
- `1`: SPHERE
- `2`: CYLINDER
- `3`: CAPSULE
- `4`: MESH (from file)

### Generate Trajectory (Will Avoid Obstacle)

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.32, y: -0.44, z: 0.13}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

Watch in RViz - the robot now avoids the table!

### Add More Obstacles

```bash
# Add a sphere
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{
  name: 'ball',
  type: 1,
  pose: {position: {x: 0.6, y: 0.2, z: 0.5}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 0.1, y: 0.1, z: 0.1},
  color: {r: 1.0, g: 0.0, b: 0.0, a: 0.8}
}"
```

**Note**: For spheres, set all dimensions equal (x=y=z=radius).

### List All Obstacles

```bash
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger
```

**Response**:
```yaml
success: true
message: "table, ball"  # List of obstacle names
```

### Remove Specific Obstacle

```bash
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject "{name: 'ball'}"
```

### Clear All Obstacles

```bash
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger
```

---

## Step 7: Tuning Parameters

Let's experiment with parameters to see their effect.

### Adjust Robot Speed

```bash
# Slow motion (30% speed)
ros2 param set /unified_planner time_dilation_factor 0.3

# Generate trajectory
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"

# Watch in RViz - ghost robot moves slower
```

```bash
# Fast motion (80% speed)
ros2 param set /unified_planner time_dilation_factor 0.8

# Generate again - much faster!
```

### Increase Planning Attempts

```bash
# Try 3 times if first attempt fails
ros2 param set /unified_planner max_attempts 3

# This increases success rate for difficult targets
```

### Adjust Timeout

```bash
# Allow more time for planning
ros2 param set /unified_planner timeout 10.0
```
### Adjusting Voxel Size for Precision

Some parameters require special handling because they are initialized during cuRobo's warmup phase. If you modify these parameters at runtime, you must trigger a reinitialization for the changes to take effect.

#### Modifying Voxel Size

The voxel size parameter controls the resolution of the collision grid used for obstacle detection:

```bash
# Set voxel size to 8cm (good balance between precision and performance)
ros2 param set /unified_planner voxel_size 0.08
```

**Resolution Capabilities:**
- **High-end GPUs:** Can handle resolutions down to 0.01m (1cm)
- **Recommended default:** 0.05m (5cm) for most applications
- **Large environments:** 0.1m (10cm) for better performance

**Important:** After changing voxel size, you must update the motion generation configuration:

```bash
# Trigger configuration update to apply voxel size changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

#### Performance vs Precision Trade-off

| Voxel Size | Use Case | GPU Memory | Planning Speed |
|------------|----------|------------|----------------|
| 0.01m | Fine manipulation | ⚠️ High | ⚠️ Slow |
| 0.05m | General purpose | ✅ Medium | ✅ Fast |
| 0.10m | Large environments | ✅ Low | ✅ Very Fast |

**Tip:** Start with the default (0.05m) and adjust based on your specific application needs and GPU capabilities.

---

## Step 8: Executing Trajectories (Optional)

If you have a real robot or emulator configured, you can execute the trajectory.

### Using the Action Interface

```bash
# Switch the strategy to use the emulator
ros2 service call /unified_planner/set_robot_strategy curobo_msgs/srv/SetRobotStrategy "{robot_strategy: 1}"


# Generate a trajectory first

ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
 

# Execute it
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory {}
```

**What happens:**
1. The robot starts following the trajectory
2. You receive feedback: `step_progression: 0.25` (25% complete)
3. Robot completes the motion
4. Result: `success: true`

**Safety**: Make sure the workspace is clear and the robot can move safely!

---


## Common Issues

### "IK solution not found"

**Cause**: Target pose is unreachable (out of workspace or impossible orientation)

**Solutions**:
- Try a target closer to the robot
- Check orientation is valid
- Increase `max_attempts`: `ros2 param set /unified_planner max_attempts 5`

### "Collision detected" / "Planning failed"

**Cause**: Path to target is blocked by obstacles

**Solutions**:
- Remove or move obstacles
- Try a different target
- Increase `collision_activation_distance` to allow tighter clearances

### Trajectory looks jerky in RViz

**Cause**: Low `time_dilation_factor` or visualization artifacts

**Solutions**:
- Increase `time_dilation_factor`: `ros2 param set /unified_planner time_dilation_factor 0.6`
- This is just visualization - actual robot motion will be smooth

### Planning takes too long

**Cause**: Voxel size is too small, or environment is complex

**Solutions**:
- Increase voxel size (launch parameter - requires restart)
- Reduce `timeout` to fail faster
- Check CPU/GPU usage

---

## Summary

**Congratulations!** You've learned how to:

- ✅ Generate trajectories to target poses
- ✅ Understand orientation with quaternions
- ✅ Add and manage obstacles
- ✅ Tune parameters (`time_dilation_factor`, `max_attempts`, `timeout`)
- ✅ Use the trajectory service from Python
- ✅ Execute trajectories (optional)

---

## Next Steps

- **[Adding Your Robot](02-adding-your-robot.md)** - Integrate your own robot (Doosan M1013 example)
- **[Managing Obstacles](03-collision-objects.md)** - Advanced obstacle management
- **[Parameters Guide](../concepts/parameters.md)** - Deep dive into all parameters
- **[Dynamic Strategy Switching](04-strategy-switching.md)** - Switch between robot modes



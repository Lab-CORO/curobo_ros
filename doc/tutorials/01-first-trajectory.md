# Tutorial: Your First Trajectory

This tutorial walks you through generating your first motion plan with curobo_ros. You'll learn how to:
- Request trajectory generation
- Understand the response
- Add obstacles
- Tune parameters for better performance
- Execute trajectories

---

## Prerequisites

- Completed [Getting Started Guide](../getting_started.md)
- unified_planner node is running
- RViz is open

If not, start the system:
```bash
ros2 launch curobo_ros unified_planner.launch.py
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
# If not: docker exec -it x86docker bash

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
  "{target_pose: {position: {x: 0.6, y: 0.3, z: 0.4}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
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
  dimensions: {x: 0.8, y: 1.0, z: 0.05},
  color: {r: 0.6, g: 0.4, b: 0.2, a: 0.9}
}"
```

**What this creates:**
- A box representing a table
- Position: 0.4m in front, ground level
- Dimensions: 80cm x 100cm x 5cm
- Color: Brown (RGBA)

**Object types**:
- `0`: CUBOID
- `1`: SPHERE
- `2`: CYLINDER
- `3`: CAPSULE
- `4`: MESH (from file)

### Generate Trajectory (Will Avoid Obstacle)

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
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

---

## Step 8: Executing Trajectories (Optional)

If you have a real robot or emulator configured, you can execute the trajectory.

### Using the Action Interface

```bash
# Generate a trajectory first
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"

# Execute it
ros2 action send_goal /unified_planner/send_trajectrory curobo_msgs/action/SendTrajectory "{}"
```

**What happens:**
1. The robot starts following the trajectory
2. You receive feedback: `step_progression: 0.25` (25% complete)
3. Robot completes the motion
4. Result: `success: true`

**Safety**: Make sure the workspace is clear and the robot can move safely!

---

## Step 9: Python Example

Here's a complete Python script to generate trajectories:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from curobo_msgs.srv import TrajectoryGeneration
from geometry_msgs.msg import Pose
import sys


class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        self.client = self.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )

        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory service...')

    def generate_trajectory(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """Generate trajectory to target pose"""
        request = TrajectoryGeneration.Request()
        request.target_pose.position.x = x
        request.target_pose.position.y = y
        request.target_pose.position.z = z
        request.target_pose.orientation.w = qw
        request.target_pose.orientation.x = qx
        request.target_pose.orientation.y = qy
        request.target_pose.orientation.z = qz

        self.get_logger().info(f'Planning to ({x}, {y}, {z})...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Success! {response.message}')
                self.get_logger().info(f'   Trajectory has {len(response.trajectory.points)} waypoints')
                return True
            else:
                self.get_logger().error(f'❌ Failed: {response.message}')
                return False
        else:
            self.get_logger().error('❌ Service call failed')
            return False


def main():
    rclpy.init()
    client = TrajectoryClient()

    # Generate trajectory to multiple targets
    targets = [
        (0.5, 0.0, 0.3),
        (0.6, 0.2, 0.4),
        (0.5, -0.2, 0.5),
    ]

    for target in targets:
        client.generate_trajectory(*target)
        input("Press Enter to plan to next target...")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Save as** `my_trajectory_client.py` and run:
```bash
python3 my_trajectory_client.py
```

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

- **[Adding Your Robot](2_adding_your_robot.md)** - Integrate your own robot (Doosan M1013 example)
- **[Managing Obstacles](3_adding_obstacles.md)** - Advanced obstacle management
- **[Parameters Guide](../concepts/parameters.md)** - Deep dive into all parameters
- **[Dynamic Strategy Switching](4_dynamic_strategy_switching.md)** - Switch between robot modes

---

## Reference

### Trajectory Generation Service

**Service**: `/unified_planner/generate_trajectory`
**Type**: `curobo_msgs/srv/TrajectoryGeneration`

**Request**:
```yaml
target_pose:
  position: {x: float, y: float, z: float}
  orientation: {w: float, x: float, y: float, z: float}
```

**Response**:
```yaml
success: bool
message: string
trajectory: trajectory_msgs/JointTrajectory
```

### Useful Commands

```bash
# Check node status
ros2 node info /unified_planner

# Monitor planning time
ros2 topic hz /trajectory

# View collision spheres
ros2 topic echo /unified_planner/collision_spheres

# List all parameters
ros2 param list | grep unified_planner
```

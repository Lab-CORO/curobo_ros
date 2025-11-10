# Tutorial: Using the MPC Planner
 
Model Predictive Control (MPC) enables **real-time reactive trajectory planning** that can adapt to dynamic obstacles and disturbances.
 
---
 
## What is MPC?
 
**MPC (Model Predictive Control)** is a closed-loop control strategy that:
- **Predicts** future states over a time horizon
- **Optimizes** control actions to reach the goal
- **Executes** only the first control step
- **Repeats** continuously until goal is reached
 
### MPC vs Classic Planning
 
| Aspect | Classic Planner | MPC Planner |
|--------|----------------|-------------|
| **Loop** | Open-loop | Closed-loop |
| **Planning** | Once before execution | Continuous during execution |
| **Adaptability** | Fixed trajectory | Real-time adaptation |
| **Obstacles** | Static only | Static + dynamic |
| **Disturbances** | Requires replanning | Automatic correction |
| **GPU Usage** | Burst | Sustained |
| **Latency** | 50-200ms | 1-10ms per iteration |
 
---
 
## Prerequisites
 
Before starting this tutorial:
 
1. Complete [Tutorial 1: First Trajectory](1_first_trajectory.md)
2. Ensure you have:
   - NVIDIA GPU with CUDA support
   - curobo_ros installed and configured
   - Robot simulator or real robot
 
---
 
## Step 1: Launch with MPC Planner
 
### Using Launch File
 
Create a launch file that uses MPC by default:
 
```python
# mpc_demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curobo_ros',
            executable='unified_planner',
            name='trajectory_planner',
            parameters=[{
                'default_planner': 'mpc',  # Use MPC planner
                'robot_type': 'doosan_m1013',
                'time_dilation_factor': 0.5,
 
                # MPC-specific parameters
                'mpc_convergence_threshold': 0.01,  # meters
                'mpc_max_iterations': 1000,
                'mpc_horizon': 10,
                'mpc_control_frequency': 100  # Hz
            }],
            output='screen'
        )
    ])
```
 
Launch it:
```bash
ros2 launch curobo_ros mpc_demo.launch.py
```
 
### Using Command Line
 
Alternatively, launch with default settings and switch to MPC:
 
```bash
# Terminal 1: Launch unified planner
ros2 run curobo_ros unified_planner
 
# Terminal 2: Switch to MPC
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
```
 
---
 
## Step 2: Configure MPC Parameters
 
MPC has several tunable parameters:
 
### Convergence Threshold
 
How close to the goal before stopping (in meters):
 
```bash
ros2 param set /trajectory_planner mpc_convergence_threshold 0.01
```
 
- **Smaller values** (0.001): Higher precision, longer execution
- **Larger values** (0.05): Faster completion, lower precision
- **Recommended**: 0.01-0.02 for most tasks
 
### Max Iterations
 
Maximum number of MPC iterations:
 
```bash
ros2 param set /trajectory_planner mpc_max_iterations 1000
```
 
- **More iterations**: Can handle complex scenes, slower
- **Fewer iterations**: Faster but may not converge
- **Recommended**: 500-1000 for dynamic environments
 
### MPC Horizon
 
Planning horizon length (number of future steps):
 
```bash
ros2 param set /trajectory_planner mpc_horizon 10
```
 
- **Longer horizon**: Better long-term planning, more computation
- **Shorter horizon**: Reactive, less computational cost
- **Recommended**: 5-15 steps
 
### Control Frequency
 
How often MPC replans (Hz):
 
```bash
ros2 param set /trajectory_planner mpc_control_frequency 100
```
 
- **Higher frequency**: More reactive, more GPU usage
- **Lower frequency**: Less smooth, more efficient
- **Recommended**: 50-100 Hz for real-time control
 
---
 
## Step 3: Generate and Execute with MPC
 
### Using Python API
 
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from curobo_ros.planners import PlannerFactory
from geometry_msgs.msg import Pose
import numpy as np
 
class MPCDemoNode(Node):
    def __init__(self):
        super().__init__('mpc_demo_node')
 
        # Create MPC planner
        self.planner = PlannerFactory.create_planner(
            'mpc',
            self,
            config_wrapper
        )
 
        self.get_logger().info('MPC planner initialized')
 
    def plan_and_execute(self):
        # Define start state (current robot position)
        start_state = self.get_current_joint_state()
 
        # Define goal pose
        goal_pose = Pose()
        goal_pose.position.x = 0.5
        goal_pose.position.y = 0.3
        goal_pose.position.z = 0.4
        goal_pose.orientation.w = 1.0
 
        # Plan with MPC
        config = {
            'convergence_threshold': 0.01,
            'max_iterations': 1000
        }
 
        result = self.planner.plan(start_state, goal_pose, config)
 
        if result.success:
            self.get_logger().info('MPC planning succeeded')
 
            # Execute (MPC will continuously replan)
            success = self.planner.execute(robot_context, goal_handle)
 
            if success:
                self.get_logger().info('MPC execution completed')
        else:
            self.get_logger().error(f'MPC planning failed: {result.message}')
 
def main():
    rclpy.init()
    node = MPCDemoNode()
    node.plan_and_execute()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
```
 
### Using ROS 2 Services/Actions
 
```bash
# 1. Generate trajectory with MPC
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{
    goal_pose: {
      position: {x: 0.5, y: 0.3, z: 0.4},
      orientation: {w: 1.0}
    },
    use_current_state: true
  }"
 
# 2. Execute with MPC (will continuously replan)
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```
 
---
 
## Step 4: Observe MPC Behavior
 
### In RViz
 
1. **Visualization**:
   - Ghost robot shows predicted trajectory
   - Trajectory updates in real-time
   - Can see MPC adapting to obstacles
 
2. **Add dynamic obstacle** while robot is moving:
   ```bash
   ros2 service call /unified_planner/add_collision_object curobo_msgs/srv/ManageCollisionObject \
     "{
       object: {
         name: 'dynamic_box',
         primitive_type: 1,
         dimensions: [0.2, 0.2, 0.2],
         pose: {position: {x: 0.4, y: 0.2, z: 0.3}}
       }
     }"
   ```
 
3. **Observe**: MPC automatically avoids new obstacle without stopping!
 
### Monitor MPC Performance
 
```bash
# Monitor MPC iterations
ros2 topic echo /trajectory_planner/mpc_stats
 
# Expected output:
# iterations: 87
# convergence: 0.009  # meters from goal
# computation_time: 8.5  # ms
# success: true
```
 
---
 
## Step 5: Advanced MPC Usage
 
### Dynamic Obstacle Avoidance
 
MPC excels at avoiding obstacles that appear during execution:
 
```python
def demo_dynamic_avoidance():
    # Start MPC execution
    planner.execute(robot_context, goal_handle)
 
    # Simulate dynamic obstacle appearing
    time.sleep(2.0)  # Let robot start moving
 
    # Add obstacle in robot's path
    add_collision_object("moving_obstacle", [0.5, 0.3, 0.4])
 
    # MPC automatically avoids it without stopping!
```
 
### Disturbance Rejection
 
MPC can correct for external disturbances:
 
```python
# Start MPC execution
planner.execute(robot_context, goal_handle)
 
# Simulate disturbance (push robot)
def apply_disturbance():
    # MPC detects deviation and corrects automatically
    pass
```
 
### Tracking Moving Targets
 
MPC can track moving goal positions:
 
```python
class MovingTargetMPC:
    def __init__(self):
        self.planner = PlannerFactory.create_planner('mpc', node, config)
        self.timer = node.create_timer(0.1, self.update_goal)
 
    def update_goal(self):
        # Update goal position (e.g., tracking moving object)
        new_goal = self.get_moving_target_pose()
 
        # MPC will automatically adjust trajectory
        self.planner.update_goal(new_goal)
```
 
---
 
## Step 6: Tuning MPC for Your Application
 
### High-Speed Reactive Control
 
For fast, reactive movements:
 
```python
config = {
    'convergence_threshold': 0.02,  # Slightly relaxed
    'max_iterations': 500,          # Fewer iterations
    'control_frequency': 200,       # Very high frequency
    'horizon': 5                    # Short horizon
}
```
 
### High-Precision Tasks
 
For precise positioning:
 
```python
config = {
    'convergence_threshold': 0.005,  # Tight tolerance
    'max_iterations': 2000,          # More iterations
    'control_frequency': 50,         # Lower frequency OK
    'horizon': 15                    # Longer horizon
}
```
 
### Constrained Environments
 
For cluttered spaces with many obstacles:
 
```python
config = {
    'convergence_threshold': 0.01,
    'max_iterations': 1500,
    'voxel_size': 0.005,  # Fine collision checking
    'horizon': 10
}
```
 
---
 
## Common Issues and Solutions
 
### Issue 1: MPC Not Converging
 
**Symptom:**
```
Warning: MPC failed to converge after 1000 iterations
```
 
**Solutions:**
1. Increase `max_iterations`:
   ```bash
   ros2 param set /trajectory_planner mpc_max_iterations 2000
   ```
 
2. Check goal is reachable:
   ```bash
   # Test with classic planner first
   ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"
   ```
 
3. Relax convergence threshold:
   ```bash
   ros2 param set /trajectory_planner mpc_convergence_threshold 0.02
   ```
 
### Issue 2: Jerky Motion
 
**Symptom:** Robot movement is not smooth, oscillates
 
**Solutions:**
1. Decrease control frequency:
   ```bash
   ros2 param set /trajectory_planner mpc_control_frequency 50
   ```
 
2. Increase horizon length:
   ```bash
   ros2 param set /trajectory_planner mpc_horizon 15
   ```
 
3. Tune time_dilation_factor:
   ```bash
   ros2 param set /trajectory_planner time_dilation_factor 0.3  # Slower, smoother
   ```
 
### Issue 3: High GPU Usage
 
**Symptom:** GPU at 100%, system slow
 
**Solutions:**
1. Reduce control frequency:
   ```bash
   ros2 param set /trajectory_planner mpc_control_frequency 30
   ```
 
2. Shorten horizon:
   ```bash
   ros2 param set /trajectory_planner mpc_horizon 5
   ```
 
3. Use classic planner for static environments:
   ```bash
   ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"
   ```
 
### Issue 4: Slow Reaction to Obstacles
 
**Symptom:** MPC doesn't avoid obstacles quickly enough
 
**Solutions:**
1. Increase control frequency:
   ```bash
   ros2 param set /trajectory_planner mpc_control_frequency 150
   ```
 
2. Reduce collision voxel size (finer detection):
   ```bash
   ros2 param set /trajectory_planner voxel_size 0.005
   ```
 
3. Adjust MPC weights (favor obstacle avoidance):
   ```python
   config['obstacle_weight'] = 10.0  # Higher priority
   ```
 
---
 
## Comparing MPC vs Classic
 
### Experiment: Static Environment
 
```bash
# Test 1: Classic planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"
ros2 service call /unified_planner/generate_trajectory ...
# Observe: Fast planning, smooth execution, predictable
 
# Test 2: MPC planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
ros2 service call /unified_planner/generate_trajectory ...
# Observe: Continuous replanning, slightly slower, adaptive
```
 
**Result:** In static environments, Classic is more efficient
 
### Experiment: Dynamic Environment
 
```bash
# Add obstacle mid-execution
 
# With Classic planner:
# - Robot must stop
# - Replan required
# - Execution resumes
 
# With MPC planner:
# - No stop needed
# - Automatic avoidance
# - Continuous execution
```
 
**Result:** In dynamic environments, MPC is superior
 
---
 
## Best Practices
 
### 1. Choose the Right Planner
 
- **Use Classic when:**
  - Environment is static
  - Speed/efficiency is critical
  - Trajectory must be reproducible
  - Limited GPU resources
 
- **Use MPC when:**
  - Environment has moving obstacles
  - Real-time adaptation needed
  - Disturbance rejection required
  - High-precision tracking
 
### 2. Start with Default Parameters
 
```python
# Good defaults for most applications
config = {
    'convergence_threshold': 0.01,
    'max_iterations': 1000,
    'control_frequency': 100,
    'horizon': 10
}
```
 
Tune only if needed!
 
### 3. Monitor Performance
 
Always monitor MPC performance:
 
```bash
ros2 topic echo /trajectory_planner/mpc_stats
```
 
Look for:
- **Iterations**: Should be < max_iterations
- **Convergence**: Should approach threshold
- **Computation time**: Should be < 1/frequency
 
### 4. Warm-Up the Planner
 
Pre-warm MPC for faster first execution:
 
```python
# At startup
planner = PlannerFactory.create_planner('mpc', node, config)
planner.warmup()  # Initializes GPU kernels
```
 
### 5. Fallback to Classic
 
If MPC fails, fallback to Classic:
 
```python
result = mpc_planner.plan(start, goal, config)
if not result.success:
    self.get_logger().warn('MPC failed, falling back to Classic')
    classic_planner = PlannerFactory.create_planner('classic', node, config)
    result = classic_planner.plan(start, goal, config)
```
 
---
 
## Next Steps
 
- [Batch Planner Tutorial](6_batch_planner.md) - Multiple trajectory generation
- [Constrained Planning](7_constrained_planning.md) - Custom constraints
- [Unified Planner Concepts](../concepts/unified_planner.md) - Deep dive into architecture
- [Performance Optimization](../concepts/warmup_async.md) - GPU optimization
 
---
 
## References
 
- [MPC Overview](https://en.wikipedia.org/wiki/Model_predictive_control)
- [cuRobo Documentation](https://curobo.org/)
- [Real-time Trajectory Optimization](https://doi.org/10.1109/ICRA.2021.XXXXXX)
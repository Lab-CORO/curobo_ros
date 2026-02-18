# Grasp MPC Planner - Integration Guide

## Overview

This guide explains how to integrate the new **GraspMPCPlanner** into the `unified_planner_node` for reactive grasping using Model Predictive Control.

## Files Created

### Core Implementation
1. **`curobo_ros/planners/grasp_mpc_planner.py`** - Main MPC grasp planner
2. **`curobo_ros/robot/gripper_interface.py`** - Abstract gripper interface
3. **`curobo_ros/robot/doosan_gripper.py`** - Doosan M1013 gripper implementation

### ROS2 Messages
4. **`curobo_msgs/srv/GraspObject.srv`** - Grasp service definition
5. **`curobo_msgs/action/GraspMPC.action`** - Grasp action definition (optional)

### Modified Files
6. **`curobo_ros/planners/planner_factory.py`** - Added GraspMPCPlanner registration
7. **`curobo_ros/core/obstacle_manager.py`** - Added `get_object()` method

### Test & Examples
8. **`curobo_ros/examples/grasp_mpc_test.py`** - Test script

---

## Integration Steps

### Step 1: Update CMakeLists.txt for Message Generation

Add the new message files to `curobo_msgs/CMakeLists.txt`:

```cmake
# In the rosidl_generate_interfaces section, add:
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing messages ...
  "srv/GraspObject.srv"
  "action/GraspMPC.action"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)
```

### Step 2: Rebuild Messages

```bash
cd /home/ros2_ws
colcon build --packages-select curobo_msgs
source install/setup.bash
```

### Step 3: Add Parameters to unified_planner_node.py

In `unified_planner_node.py`, add grasp-specific parameters in `__init__()`:

```python
# Add after existing parameter declarations (around line 67):
# Grasp MPC parameters
self.declare_parameter('grasp_pre_grasp_offset_z', 0.10)    # 10cm above object
self.declare_parameter('grasp_post_grasp_offset_z', 0.15)   # 15cm lift
self.declare_parameter('grasp_approach_convergence', 0.02)  # Looser for approach
self.declare_parameter('grasp_grasp_convergence', 0.005)    # Tight for grasp
self.declare_parameter('grasp_retreat_convergence', 0.015)  # Moderate for retreat
self.declare_parameter('grasp_gripper_close_time', 1.0)     # 1s gripper wait
```

### Step 4: Import GraspObject Service

Add to imports at the top of `unified_planner_node.py`:

```python
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, GraspObject
```

### Step 5: Initialize Gripper Interface

In `__init__()`, after robot_context initialization:

```python
# Initialize gripper interface (after line 56)
from curobo_ros.robot.doosan_gripper import DoosanGripper
self.gripper = DoosanGripper(self)
```

### Step 6: Create Grasp Service

Add service creation in `__init__()` after existing services (around line 118):

```python
# Grasp service
self.grasp_service = self.create_service(
    GraspObject,
    f'{self.get_name()}/grasp_object',
    self.grasp_object_callback,
    callback_group=MutuallyExclusiveCallbackGroup()
)
```

### Step 7: Implement Grasp Callback

Add this method to `UnifiedPlannerNode` class (around line 500, after `list_planners_callback`):

```python
def grasp_object_callback(self, request: GraspObject.Request, response: GraspObject.Response):
    """
    Handle grasp object service request.

    This service executes a complete grasp sequence:
    1. Switch to grasp_mpc planner
    2. Plan and execute grasp with MPC reactive control
    3. Return success/failure status
    """
    import time
    start_time = time.time()

    try:
        self.get_logger().info(f"Grasp request received for object: {request.object_name}")

        # Switch to grasp_mpc planner
        self.planner_manager.set_current_planner('grasp_mpc')
        planner = self.planner_manager.get_current_planner()

        # Set gripper interface
        from curobo_ros.planners.grasp_mpc_planner import GraspMPCPlanner
        if isinstance(planner, GraspMPCPlanner):
            planner.set_gripper_interface(self.gripper)

            # Set MPC solver if not already set
            if planner.mpc is None:
                # Ensure MPC config wrapper is initialized
                if self.config_wrapper_mpc is None:
                    self._ensure_mpc_warmup()

                # Set MPC solver from wrapper
                planner.set_mpc_solver(self.mpc)

        # Get current joint state
        current_joint_pose = self.robot_context.get_joint_pose()
        start_state = JointState.from_position(
            torch.Tensor([current_joint_pose]).to(device=self.tensor_args.device)
        )

        # Prepare config
        config = {
            'convergence_threshold': self.get_parameter('convergence_threshold').value,
            'max_iterations': self.get_parameter('max_mpc_iterations').value,
        }

        # Plan grasp sequence
        self.get_logger().info("Planning grasp sequence...")
        plan_result = planner.plan(start_state, request, config, self.robot_context)

        if not plan_result.success:
            response.success = False
            response.message = f"Grasp planning failed: {plan_result.message}"
            response.execution_time = time.time() - start_time
            return response

        # Execute grasp sequence
        self.get_logger().info("Executing grasp sequence...")
        execution_success = planner.execute(self.robot_context, goal_handle=None)

        # Prepare response
        response.success = execution_success
        response.execution_time = time.time() - start_time

        if execution_success:
            response.message = f"Grasp successful! (time: {response.execution_time:.2f}s)"
            self.get_logger().info(response.message)
        else:
            response.message = "Grasp execution failed"
            self.get_logger().error(response.message)

        return response

    except Exception as e:
        self.get_logger().error(f"Grasp service error: {e}")
        import traceback
        self.get_logger().error(traceback.format_exc())

        response.success = False
        response.message = f"Grasp error: {str(e)}"
        response.execution_time = time.time() - start_time
        return response
```

### Step 8: Add MPC Warmup Helper

Add this helper method to ensure MPC is warmed up (around line 200, after `_warmup_initial_planner`):

```python
def _ensure_mpc_warmup(self):
    """Ensure MPC solver is warmed up (lazy initialization)."""
    if self.mpc is None:
        self.get_logger().info("Warming up MPC solver...")

        # Create MPC config wrapper if needed
        if self.config_wrapper_mpc is None:
            self.config_wrapper_mpc = ConfigWrapperMPC(self, self.robot_context)

        # Create and warmup MPC solver
        self.config_wrapper_mpc.set_mpc_solver_config(self)
        self.mpc = self.config_wrapper_mpc.mpc

        self.get_logger().info("MPC solver ready")
```

### Step 9: Rebuild curobo_ros Package

```bash
cd /home/ros2_ws
colcon build --packages-select curobo_ros
source install/setup.bash
```

---

## Usage

### 1. Launch Unified Planner

```bash
ros2 launch curobo_ros unified_planner.launch.py
```

### 2. Add an Object to Grasp

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 0, name: 'test_cube', pose: {position: {x: 0.5, y: 0.0, z: 0.3}, \
   orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, \
   dimensions: {x: 0.05, y: 0.05, z: 0.05}, \
   color: {r: 0.2, g: 0.5, b: 0.8, a: 1.0}}"
```

### 3. Execute Grasp

```bash
ros2 service call /unified_planner/grasp_object curobo_msgs/srv/GraspObject \
  "{object_name: 'test_cube', \
   target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, \
                 orientation: {w: 0.0, x: 0.0, y: -1.0, z: 0.0}}, \
   pre_grasp_offset: {x: 0.0, y: 0.0, z: 0.1}, \
   post_grasp_offset: {x: 0.0, y: 0.0, z: 0.15}, \
   use_current_state: true}"
```

### 4. Run Test Script

```bash
ros2 run curobo_ros grasp_mpc_test.py
```

---

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grasp_pre_grasp_offset_z` | 0.10 | Height above object for approach (meters) |
| `grasp_post_grasp_offset_z` | 0.15 | Height to lift after grasping (meters) |
| `grasp_approach_convergence` | 0.02 | MPC convergence threshold for approach phase (meters) |
| `grasp_grasp_convergence` | 0.005 | MPC convergence threshold for grasp phase (meters) |
| `grasp_retreat_convergence` | 0.015 | MPC convergence threshold for retreat phase (meters) |
| `grasp_gripper_close_time` | 1.0 | Time to wait for gripper operation (seconds) |

### Adjusting Parameters

```bash
ros2 param set /unified_planner grasp_grasp_convergence 0.003  # Tighter precision
ros2 param set /unified_planner grasp_gripper_close_time 0.5   # Faster gripper
```

---

## Gripper Configuration

### Doosan M1013 (Default)

The default implementation uses `DoosanGripper` which publishes to `/doosan/gripper/command` (Float64):
- 0.0 = fully closed
- 1.0 = fully open

### Custom Gripper

To use a different gripper:

1. Create a new gripper class inheriting from `GripperInterface`
2. Implement `open()`, `close()`, and `get_state()` methods
3. Modify unified_planner_node.py to use your gripper class

Example:

```python
from curobo_ros.robot.gripper_interface import GripperInterface

class MyCustomGripper(GripperInterface):
    def __init__(self, node):
        # Initialize your gripper interface
        pass

    def open(self) -> bool:
        # Send open command
        return True

    def close(self) -> bool:
        # Send close command
        return True

    def get_state(self) -> float:
        # Return gripper position (0.0-1.0)
        return 0.5
```

---

## Troubleshooting

### Issue: "MPC solver not initialized"

**Solution**: The planner needs the MPC solver set. Ensure `_ensure_mpc_warmup()` is called before using the grasp planner.

### Issue: "Object not found in obstacle manager"

**Solution**: Make sure the object exists in the world. Use `/unified_planner/add_object` to add it first.

### Issue: "Grasp convergence failed"

**Solution**:
- Check if the target pose is reachable
- Adjust convergence thresholds (looser thresholds converge faster)
- Increase `max_mpc_iterations` parameter

### Issue: Gripper not responding

**Solution**:
- Verify gripper topic is correct (`/doosan/gripper/command`)
- Check gripper ROS2 driver is running
- Adjust `grasp_gripper_close_time` if gripper is slow

---

## Next Steps

### TODO: Sphere Attachment to Robot Kinematics

The current implementation generates collision spheres but doesn't attach them to the robot model. To complete this:

1. Access the MPC solver's kinematics model
2. Dynamically add spheres to the gripper link
3. Update the world model after attachment

### Future Enhancements

1. **Grasp + Place**: Extend to full pick-and-place sequence
2. **Vision Integration**: Get grasp pose from perception
3. **Grasp Quality**: Evaluate grasp stability before execution
4. **Force Feedback**: Adaptive grasping based on gripper force sensors
5. **Multi-object**: Batch grasp multiple objects

---

## References

- [CuRobo MPC Documentation](https://curobo.org)
- [Original Plan](/root/.claude/plans/glowing-pondering-meteor.md)
- Block Stacking Example: https://curobo.org/advanced_examples/2_block_stacking_example.html

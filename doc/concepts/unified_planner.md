# Unified Planner Architecture
 
The **Unified Planner** is a flexible, extensible architecture for trajectory planning in curobo_ros that supports multiple planning algorithms through a unified interface.
 
---
 
## Overview
 
The unified planner architecture uses the **Strategy Pattern** to encapsulate different trajectory planning algorithms, making it easy to switch between planning methods at runtime without modifying your code.
 
### Available Planners
 
| Planner | Mode | Description | Use Case |
|---------|------|-------------|----------|
| **ClassicPlanner** | Open-loop | Complete trajectory generation then execution | Simple navigation, pre-computed paths |
| **MPCPlanner** | Closed-loop | Real-time recalculation at each step | Reactive control, dynamic environments |
| **BatchPlanner** | Open-loop | Generate multiple alternative trajectories | Multi-task optimization |
| **ConstrainedPlanner** | Open-loop | Planning with custom constraints | Specific constraints (orientation, velocity) |
 
---
 
## Architecture
 
```
TrajectoryPlanner (Abstract Base Class)
├── plan(start, goal, config) → PlannerResult
├── execute(robot_context, goal_handle) → bool
└── get_execution_mode() → ExecutionMode
 
ClassicPlanner
├── Mode: OPEN_LOOP
├── Algorithm: MotionGen
└── Execution: Complete trajectory at once
 
MPCPlanner
├── Mode: CLOSED_LOOP
├── Algorithm: MPC Solver
└── Execution: Iterative real-time loop
```
 
### Execution Modes
 
- **OPEN_LOOP**: Generate complete trajectory, then execute
  - More predictable and reproducible
  - Efficient for static environments
  - Requires replanning if disturbed
 
- **CLOSED_LOOP**: Continuous replanning during execution
  - Reactive to perturbations
  - Adaptive in real-time
  - Handles dynamic environments
 
---
 
## Using the Unified Planner
 
### 1. ROS 2 Command Line
 
```bash
# Launch the unified planner node
ros2 run curobo_ros unified_planner
 
# List available planners
ros2 service call /unified_planner/list_planners std_srvs/srv/Trigger
 
# Switch planners (using enum type-safe values)
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"  # CLASSIC
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"  # MPC
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 2}"  # BATCH
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 3}"  # CONSTRAINED
 
# Generate a trajectory
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "..."
 
# Execute trajectory
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```
 
### 2. Python API
 
```python
from curobo_ros.planners import PlannerFactory, PlannerManager
 
# Create a planner directly
planner = PlannerFactory.create_planner('classic', node, config_wrapper)
 
# Or use the manager to handle multiple planners
manager = PlannerManager(node, config_wrapper)
manager.set_current_planner('classic')
 
# Plan
result = planner.plan(start_state, goal_pose, config)
 
if result.success:
    # Execute
    success = planner.execute(robot_context, goal_handle)
```
 
### 3. Launch File Configuration
 
You can specify the default planner in your launch file:
 
```python
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curobo_ros',
            executable='unified_planner',
            parameters=[{
                'default_planner': 'classic',  # or 'mpc', 'batch', 'constrained'
                'robot_type': 'doosan_m1013',
                'time_dilation_factor': 0.5
            }]
        )
    ])
```
 
---
 
## Planner Comparison
 
### ClassicPlanner (Open-loop)
 
**Advantages:**
- ✅ Complete optimized trajectory
- ✅ Predictable and reproducible
- ✅ Efficient for static environments
- ✅ Lower computational cost
 
**Disadvantages:**
- ❌ No reaction to changes
- ❌ Requires replanning if disturbed
- ❌ Not suitable for dynamic obstacles
 
**Configuration:**
```python
config = {
    'max_attempts': 1,
    'timeout': 5.0,
    'time_dilation_factor': 0.5,
    'enable_graph': False,
    'use_cuda_graph': True
}
```
 
**When to use:**
- Static, predictable environments
- Pre-computed trajectories
- Maximum efficiency needed
- Reproducible results required
 
---
 
### MPCPlanner (Closed-loop)
 
**Advantages:**
- ✅ Reactive to perturbations
- ✅ Real-time adaptive
- ✅ Handles dynamic environments
- ✅ Better tracking performance
 
**Disadvantages:**
- ❌ Higher computational cost
- ❌ Requires GPU for real-time
- ❌ Less predictable
- ❌ May oscillate near goal
 
**Configuration:**
```python
config = {
    'convergence_threshold': 0.01,  # meters
    'max_iterations': 1000,
    'horizon': 10,  # MPC horizon
    'control_frequency': 100  # Hz
}
```
 
**When to use:**
- Dynamic, unpredictable environments
- Real-time obstacle avoidance
- High-precision tracking needed
- Disturbance rejection required
 
---
 
### BatchPlanner (Open-loop)
 
**Purpose:** Generate multiple alternative trajectories simultaneously for task optimization or failure recovery.
 
**Advantages:**
- ✅ Multiple solutions at once
- ✅ Can select best trajectory
- ✅ Failure recovery options
 
**When to use:**
- Need backup plans
- Multi-task optimization
- Exploring solution space
 
---
 
### ConstrainedPlanner (Open-loop)
 
**Purpose:** Planning with custom constraints beyond standard collision avoidance.
 
**Advantages:**
- ✅ Custom constraint support
- ✅ Task-specific requirements
- ✅ Orientation/velocity constraints
 
**When to use:**
- Specific end-effector orientation required
- Velocity/acceleration limits
- Custom geometric constraints
 
---
 
## Factory Pattern
 
The `PlannerFactory` centralizes planner creation:
 
```python
class PlannerFactory:
    @staticmethod
    def create_planner(planner_type: str, node, config_wrapper):
        """Create a planner instance"""
 
    @staticmethod
    def register_planner(name: str, planner_class):
        """Register a custom planner"""
 
    @staticmethod
    def list_planners() -> list:
        """List all registered planners"""
```
 
### Creating Custom Planners
 
```python
from curobo_ros.planners import TrajectoryPlanner, ExecutionMode, PlannerResult
 
class MyCustomPlanner(TrajectoryPlanner):
    def _get_execution_mode(self) -> ExecutionMode:
        return ExecutionMode.OPEN_LOOP
 
    def get_planner_name(self) -> str:
        return "My Custom Planner"
 
    def plan(self, start_state, goal_pose, config) -> PlannerResult:
        # Your planning algorithm
        trajectory = my_planning_algorithm(start_state, goal_pose)
 
        return PlannerResult(
            success=True,
            message="Planning succeeded",
            trajectory=trajectory
        )
 
    def execute(self, robot_context, goal_handle=None) -> bool:
        # Your execution logic
        robot_context.set_command(...)
        robot_context.send_trajectory()
        return True
 
# Register your planner
PlannerFactory.register_planner('custom', MyCustomPlanner)
 
# Use it
planner = PlannerFactory.create_planner('custom', node, config_wrapper)
```
 
---
 
## Planner Manager
 
The `PlannerManager` manages multiple planner instances:
 
```python
class PlannerManager:
    def __init__(self, node, config_wrapper):
        """Initialize with available planners"""
 
    def set_current_planner(self, planner_name: str) -> bool:
        """Switch to a different planner"""
 
    def get_current_planner(self) -> TrajectoryPlanner:
        """Get the active planner"""
 
    def list_available_planners(self) -> list:
        """List all available planners"""
```
 
### Example Usage
 
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
 
        # Initialize planner manager
        self.planner_manager = PlannerManager(self, config_wrapper)
        self.planner_manager.set_current_planner('classic')
 
        # Create service to switch planners
        self.create_service(
            SetPlanner,
            'set_planner',
            self.set_planner_callback
        )
 
    def set_planner_callback(self, request, response):
        success = self.planner_manager.set_current_planner(request.planner_type)
        response.success = success
        return response
 
    def plan_and_execute(self, goal):
        planner = self.planner_manager.get_current_planner()
        result = planner.plan(start, goal, config)
        if result.success:
            planner.execute(robot_context, goal_handle)
```
 
---
 
## Integration with Existing Code
 
### Migrating from Old API
 
**Before:**
```python
# Old trajectory generation
result = self.motion_gen.plan_single(start, goal, config)
traj = result.get_interpolated_plan()
robot_context.set_command(traj.joint_names, ...)
robot_context.send_trajectory()
```
 
**After:**
```python
# New unified planner
planner = PlannerFactory.create_planner('classic', self, config_wrapper)
planner.set_motion_gen(self.motion_gen)
result = planner.plan(start, goal, config)
if result.success:
    planner.execute(robot_context, goal_handle)
```
 
### Supporting Multiple Planners
 
```python
class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
 
        # Initialize planner manager
        self.planner_manager = PlannerManager(self, config_wrapper)
 
        # Set default planner
        default_planner = self.declare_parameter('default_planner', 'classic').value
        self.planner_manager.set_current_planner(default_planner)
 
    def generate_trajectory_callback(self, request, response):
        # Get current planner
        planner = self.planner_manager.get_current_planner()
 
        # Plan and execute
        result = planner.plan(start, goal, config)
        if result.success:
            success = planner.execute(robot_context, goal_handle)
            response.success = success
 
        return response
```
 
---
 
## File Structure
 
```
curobo_ros/planners/
├── __init__.py                  # Public exports
├── trajectory_planner.py        # Abstract base class
├── classic_planner.py           # Classic implementation
├── mpc_planner.py               # MPC implementation
├── batch_planner.py             # Batch implementation (future)
├── constrained_planner.py       # Constrained implementation (future)
├── planner_factory.py           # Factory and Manager
└── README.md                    # Implementation documentation
```
 
---
 
## Design Patterns
 
The unified planner architecture uses several design patterns:
 
1. **Strategy Pattern**: Encapsulates different planning algorithms
   - Makes algorithms interchangeable
   - Easy to add new planners
   - Client code doesn't need to change
 
2. **Factory Pattern**: Centralized planner creation
   - Consistent planner instantiation
   - Easy registration of custom planners
   - Decouples client from concrete classes
 
3. **Template Method**: Common methods in base class
   - Defines skeleton algorithm
   - Subclasses implement specific steps
   - Ensures consistent interface
 
---
 
## ROS 2 Services and Actions
 
### Services
 
| Service | Type | Description |
|---------|------|-------------|
| `~/set_planner` | `curobo_msgs/srv/SetPlanner` | Switch to a different planner |
| `~/list_planners` | `std_srvs/srv/Trigger` | List available planners |
| `~/generate_trajectory` | `curobo_msgs/srv/TrajectoryGeneration` | Generate trajectory with current planner |
 
### Actions
 
| Action | Type | Description |
|--------|------|-------------|
| `~/execute_trajectory` | `curobo_msgs/action/SendTrajectory` | Execute generated trajectory |
 
---
 
## Performance Considerations
 
### ClassicPlanner
- **Planning time**: 10-100ms (depends on complexity)
- **Execution**: Single trajectory send
- **GPU usage**: Burst during planning
- **Best for**: Static environments, predictable paths
 
### MPCPlanner
- **Planning time**: 1-10ms per iteration
- **Execution**: Continuous replanning loop
- **GPU usage**: Sustained during execution
- **Best for**: Dynamic environments, real-time adaptation
 
### Memory Usage
- Each planner instance: ~100-500MB GPU memory
- Planner manager overhead: Minimal (<10MB)
- Recommended: Pre-warm planners at startup
 
---
 
## Troubleshooting
 
### Planner Not Found
```
Error: Planner 'mpc' not found
```
**Solution**: Check planner is registered in factory. List available planners:
```bash
ros2 service call /unified_planner/list_planners std_srvs/srv/Trigger
```
 
### MPC Not Converging
```
Warning: MPC failed to converge after 1000 iterations
```
**Solution**:
- Increase `max_iterations`
- Decrease `convergence_threshold`
- Check goal is reachable
- Verify no collision in path
 
### GPU Out of Memory
```
RuntimeError: CUDA out of memory
```
**Solution**:
- Use only one planner at a time
- Reduce batch size for BatchPlanner
- Decrease MPC horizon length
 
---
 
## Future Extensions
 
Planned planner implementations:
 
```python
# Hybrid planner
class HybridPlanner(TrajectoryPlanner):
    """Combines Classic + MPC: fast initial plan, MPC tracking"""
    pass
 
# Learning-based planner
class LearningPlanner(TrajectoryPlanner):
    """Uses learned models for planning"""
    pass
 
# Multi-agent planner
class MultiAgentPlanner(TrajectoryPlanner):
    """Coordinates multiple robots"""
    pass
```
 
---
 
## Related Documentation
 
- [MPC Planner Tutorial](../tutorials/5_mpc_planner.md) - Step-by-step MPC usage
- [Architecture](architecture.md) - Overall system architecture
- [ROS Interfaces](ros_interfaces.md) - Services and messages
- [Dynamic Strategy Switching](../tutorials/dynamic_strategy_switching.md) - Robot control strategies
 
---
 
## References
 
- [Design Patterns (Strategy)](https://refactoring.guru/design-patterns/strategy)
- [cuRobo Documentation](https://curobo.org/)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
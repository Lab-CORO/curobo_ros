# Unified Planner Architecture

> **📋 Implementation Status**: This document describes the planned unified planner architecture. The design is complete and serves as a specification for ongoing implementation. The Classic planner (open-loop) is currently functional using the existing MotionGen integration. MPC, Batch, and Constrained planners are planned for future releases.

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

### Class Hierarchy

```
TrajectoryPlanner (Abstract Base Class)
├── plan(start, goal, config) → PlannerResult
├── execute(robot_context, goal_handle) → bool
└── get_execution_mode() → ExecutionMode

├── SinglePlanner (MotionGen-based planners) ⭐ Shared warmup
│   ├── ClassicPlanner (single-shot planning)
│   ├── MultiPointPlanner (waypoint planning)
│   ├── JointSpacePlanner (joint space planning) [Future]
│   └── GraspPlanner (grasp planning) [Future]
│
└── MPCPlanner (MpcSolver-based)
    ├── Mode: CLOSED_LOOP
    └── Execution: Iterative real-time loop
```

### SinglePlanner: Shared MotionGen Architecture

**SinglePlanner** is an abstract intermediate class for all planners that use cuRobo's MotionGen as their underlying solver. It provides infrastructure for efficient planner management:

#### Key Design Decisions

1. **Shared MotionGen Instance**
   - All `SinglePlanner` children share a **single** MotionGen instance (class-level variable)
   - Warmup is performed **only once** by `ConfigWrapperMotion`
   - Switching between SinglePlanner-based planners is **instantaneous** (no re-warmup)
   - Saves significant GPU memory and initialization time

2. **Separation of Concerns**
   - `SinglePlanner` handles: execution logic, cancellation, feedback, trajectory storage
   - Child classes implement: `_plan_trajectory()` (how to plan) and optionally `_process_trajectory()` (post-processing)

3. **Open-Loop Execution**
   - All SinglePlanner children use open-loop execution
   - Trajectory fully generated in `plan()`, then executed in `execute()`
   - Different from `MPCPlanner` which uses closed-loop

#### Architecture Comparison

| Aspect | SinglePlanner Children | MPCPlanner |
|--------|----------------------|------------|
| **Solver** | MotionGen (shared) | MpcSolver (independent) |
| **Warmup** | Once for all children | Independent |
| **Execution** | Open-loop | Closed-loop |
| **Switching Cost** | Instant | Requires MPC setup |
| **Memory** | Single instance | Separate instance |

#### Example: ClassicPlanner

```python
class ClassicPlanner(SinglePlanner):
    """Simple single-goal planner using MotionGen."""

    def get_planner_name(self) -> str:
        return "Classic Motion Generation"

    def _plan_trajectory(self, start_state, goal_pose, config):
        # Only implement HOW to plan - everything else handled by SinglePlanner
        return self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(
                max_attempts=config.get('max_attempts', 1),
                timeout=config.get('timeout', 5.0),
                time_dilation_factor=config.get('time_dilation_factor', 0.5),
            )
        )

    # execute(), cancel(), plan() all inherited from SinglePlanner!
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

#### Option 1: MotionGen-based Planner (Recommended)

If your planner uses cuRobo's MotionGen, **inherit from `SinglePlanner`** to automatically get warmup sharing:

```python
from curobo_ros.planners import SinglePlanner
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, MotionGenResult

class MyMotionGenPlanner(SinglePlanner):
    """Benefits: warmup sharing, execution/cancel/feedback already implemented"""

    def get_planner_name(self) -> str:
        return "My MotionGen Planner"

    def _plan_trajectory(self, start_state, goal_pose, config) -> MotionGenResult:
        """Define HOW to generate the trajectory - everything else is inherited"""
        return self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(
                max_attempts=config.get('max_attempts', 1),
                timeout=config.get('timeout', 5.0),
            )
        )

    # Optional: override to post-process trajectory
    def _process_trajectory(self, trajectory, config):
        # Add custom modifications here
        return trajectory

# No need to call set_motion_gen() - already shared!
PlannerFactory.register_planner('my_custom', MyMotionGenPlanner)
```

**When to use SinglePlanner:**
- ✅ Uses cuRobo MotionGen
- ✅ Wants automatic warmup sharing
- ✅ Open-loop execution
- ✅ Examples: ClassicPlanner, MultiPointPlanner

---

#### Option 2: Fully Custom Planner

If not using MotionGen, inherit from `TrajectoryPlanner`:

```python
from curobo_ros.planners import TrajectoryPlanner, ExecutionMode, PlannerResult

class MyCustomPlanner(TrajectoryPlanner):
    def _get_execution_mode(self) -> ExecutionMode:
        return ExecutionMode.OPEN_LOOP

    def get_planner_name(self) -> str:
        return "My Custom Planner"

    def plan(self, start_state, goal_pose, config, robot_context=None) -> PlannerResult:
        # Your custom planning algorithm
        trajectory = my_algorithm(start_state, goal_pose)
        return PlannerResult(success=True, trajectory=trajectory)

    def execute(self, robot_context, goal_handle=None) -> bool:
        # Your execution logic
        robot_context.set_command(...)
        return True

PlannerFactory.register_planner('custom', MyCustomPlanner)
```

**When to use TrajectoryPlanner directly:**
- ✅ Uses different solver (not MotionGen)
- ✅ Needs full control
- ✅ Example: MPCPlanner uses MpcSolver
 
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

## Implementation Roadmap

### Current Status (Phase 1: ✅ Complete)

**What's Working:**
- ✅ **Classic Planner** - Fully functional using existing `MotionGen` integration
- ✅ **Robot Strategy Pattern** - Dynamic switching between real/emulator/visualization modes
- ✅ **Configuration Framework** - `ConfigWrapper` and parameter management
- ✅ **Collision Detection** - Voxel-based BLOX integration with camera support
- ✅ **ROS 2 Integration** - Services, actions, and topics for trajectory generation

**Current Limitations:**
- Only Classic (open-loop) planner is implemented
- No runtime planner switching (UnifiedPlannerNode not yet implemented)
- MPC, Batch, and Constrained planners exist as specifications only

### Phase 2: Unified Planner Framework (🚧 In Progress)

**To Implement:**
```
curobo_ros/planners/
├── __init__.py
├── base_planner.py           # TrajectoryPlanner abstract base class
├── planner_factory.py        # PlannerFactory for creating planners
├── planner_manager.py        # PlannerManager for runtime switching
├── classic_planner.py        # Wrapper around existing MotionGen
└── execution_modes.py        # ExecutionMode enum
```

**Services to Add:**
- `~/set_planner` - Switch between planners at runtime
- `~/list_planners` - Query available planners
- `~/get_current_planner` - Get active planner name

**Key Implementation Tasks:**
1. Create `TrajectoryPlanner` abstract base class with `plan()` and `execute()` methods
2. Implement `PlannerFactory` with registration system
3. Build `PlannerManager` for runtime switching
4. Wrap existing trajectory generation in `ClassicPlanner`
5. Create `UnifiedPlannerNode` to expose new services
6. Add planner type enum to `curobo_msgs`

### Phase 3: MPC Planner (📋 Planned)

**To Implement:**
```python
curobo_ros/planners/mpc_planner.py
```

**Key Features:**
- Closed-loop control with continuous replanning
- Integration with cuRobo's MPC solver
- Configurable horizon, frequency, convergence thresholds
- Real-time obstacle adaptation
- Performance monitoring via `/mpc_stats` topic

**Dependencies:**
- cuRobo MPC solver API
- Real-time control loop framework
- GPU resource management for sustained computation

### Phase 4: Batch & Constrained Planners (📋 Future)

**Batch Planner:**
- Generate multiple alternative trajectories
- Parallel GPU computation
- Cost-based trajectory selection

**Constrained Planner:**
- Custom constraint specification API
- Orientation constraints
- Velocity/acceleration limits
- Task-space constraints

### Developer Notes

**For Contributors Implementing Phase 2:**

1. **Start with base_planner.py**:
   ```python
   from abc import ABC, abstractmethod
   from enum import Enum

   class ExecutionMode(Enum):
       OPEN_LOOP = 0
       CLOSED_LOOP = 1

   class TrajectoryPlanner(ABC):
       @abstractmethod
       def plan(self, start_state, goal_pose, config) -> PlannerResult:
           pass

       @abstractmethod
       def execute(self, robot_context, goal_handle) -> bool:
           pass
   ```

2. **Preserve Existing Functionality**: The Classic planner should wrap existing code without breaking changes

3. **Use Existing Patterns**: Follow the Robot Strategy pattern already in `curobo_ros/robot/`

4. **Testing**: Write unit tests for each planner and integration tests for switching

5. **Documentation**: Update this doc with actual implementation details as you code

**Reference Implementations:**
- Robot strategies: `curobo_ros/robot/robot_strategy.py`
- Config management: `curobo_ros/core/config_wrapper.py`
- Service patterns: Look at existing `set_robot_strategy` service

**Detailed Implementation Guide:**
- See [MPC Implementation Guide](mpc_implementation_guide.md) for complete technical specifications, code templates, and testing strategies

---

## Future Extensions

Additional planner types being considered:
 
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
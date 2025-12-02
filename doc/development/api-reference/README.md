# API Reference

Technical reference for curobo_ros Python APIs.

## 📚 API Documentation

### Core APIs
- **[Planners API](planners.md)** - Planning algorithm interfaces and implementations
- **[Robot Strategies API](robot-strategies.md)** - Robot control strategy patterns
- **[ROS Nodes API](ros-nodes.md)** - ROS 2 node implementations

## 🎯 Quick Reference

### Planners Module (`curobo_ros.planners`)

```python
from curobo_ros.planners import (
    TrajectoryPlanner,      # Abstract base class
    ClassicPlanner,         # Open-loop planning
    MPCPlanner,             # Closed-loop MPC
    BatchPlanner,           # Batch processing
    PlannerFactory,         # Factory for creating planners
    PlannerManager,         # Manager for caching/switching
)
```

See [Planners API](planners.md) for detailed documentation.

### Robot Strategies Module (`curobo_ros.robot`)

```python
from curobo_ros.robot import (
    RobotStrategy,          # Abstract base class
    RealRobotStrategy,      # Real hardware control
    EmulatorStrategy,       # Software simulation
    GhostStrategy,          # Visualization only
)
```

See [Robot Strategies API](robot-strategies.md) for detailed documentation.

### ROS Nodes Module (`curobo_ros.core`)

```python
# Main planning node
from curobo_ros.core import UnifiedPlannerNode

# IK/FK nodes
from curobo_ros.core import IKSolverNode, FKSolverNode
```

See [ROS Nodes API](ros-nodes.md) for detailed documentation.

## 📖 API Conventions

### Type Hints
All APIs use Python type hints:
```python
def plan(
    self,
    start_state: JointState,
    goal_pose: Pose,
    config: PlanConfig
) -> PlannerResult:
    ...
```

### Abstract Base Classes
Use `abc.ABC` for interfaces:
```python
from abc import ABC, abstractmethod

class TrajectoryPlanner(ABC):
    @abstractmethod
    def plan(self, ...):
        pass
```

### Error Handling
Use custom exceptions:
```python
class PlanningError(Exception):
    """Raised when planning fails"""
    pass
```

### Documentation
All public APIs use docstrings:
```python
def plan(self, start_state, goal_pose):
    """
    Plan a trajectory from start to goal.

    Args:
        start_state: Initial robot configuration
        goal_pose: Target end-effector pose

    Returns:
        PlannerResult with trajectory or error

    Raises:
        PlanningError: If planning fails
    """
```

## 🔍 Usage Examples

### Using Planner Factory

```python
from curobo_ros.planners import PlannerFactory, PlannerType

# Create classic planner
classic = PlannerFactory.create_planner(
    planner_type=PlannerType.CLASSIC,
    motion_gen=motion_gen,
    world_cfg=world_cfg,
)

# Plan trajectory
result = classic.plan(start_state, goal_pose, config)
```

### Using Planner Manager

```python
from curobo_ros.planners import PlannerManager

# Initialize manager
manager = PlannerManager(motion_gen, world_cfg)

# Switch planner
manager.set_planner(PlannerType.MPC)

# Use current planner
result = manager.plan(start_state, goal_pose)
```

### Using Robot Strategy

```python
from curobo_ros.robot import RealRobotStrategy

# Initialize strategy
strategy = RealRobotStrategy(robot_config)

# Execute trajectory
strategy.execute_trajectory(joint_trajectory)
```

## 🔗 Related Documentation

### Concepts
- [Unified Planner Architecture](../../concepts/unified-planner.md)
- [System Architecture](../../concepts/architecture.md)

### Development
- [Architecture Patterns](../architecture-patterns.md)
- [Testing Guide](../testing-guide.md)
- [Contributing Guide](../contributing.md)

### Tutorials
- [Tutorial 5: MPC Planner](../../tutorials/05-mpc-planner.md)
- [Tutorial 4: Strategy Switching](../../tutorials/04-strategy-switching.md)

## 💡 Development Tips

### Type Checking
Use mypy for static type checking:
```bash
mypy curobo_ros/
```

### Documentation Generation
Generate API docs with Sphinx:
```bash
cd docs
make html
```

### Unit Testing
Test all public APIs:
```python
def test_classic_planner_plan():
    planner = ClassicPlanner(...)
    result = planner.plan(...)
    assert result.success
```

---

[← Back to Development](../) | [Documentation Home](../../README.md)

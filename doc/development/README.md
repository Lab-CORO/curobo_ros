# Development Guide

Documentation for contributors and developers extending curobo_ros.

## 📚 Developer Documentation

### Architecture & Patterns
- **[Architecture Patterns](architecture-patterns.md)** - Strategy Pattern, Factory Pattern, and design decisions
- **[MPC Implementation Guide](mpc-implementation.md)** - Detailed guide for implementing Model Predictive Control
- **[Optimization Guide](optimization-guide.md)** - Performance optimization techniques and best practices

### Development Workflow
- **[Contributing Guide](contributing.md)** - How to contribute to curobo_ros
- **[Testing Guide](testing-guide.md)** - Unit tests, integration tests, and testing strategies
- **[Migration Guide](migration-guide.md)** - Migrating from old architecture to unified planner

### API Reference
- **[Planners API](api-reference/planners.md)** - TrajectoryPlanner interface and implementations
- **[Robot Strategies API](api-reference/robot-strategies.md)** - Robot control strategy patterns
- **[ROS Nodes API](api-reference/ros-nodes.md)** - Node implementations and interfaces

## 🎯 Developer Resources

### Quick Links

**Setting up development environment:**
1. Fork and clone the repository
2. Build Docker image for development
3. Read [Contributing Guide](contributing.md)

**Implementing new features:**
1. Review [Architecture Patterns](architecture-patterns.md)
2. Check [API Reference](api-reference/) for existing interfaces
3. Follow [Testing Guide](testing-guide.md) for test coverage

**Optimizing performance:**
1. Read [Optimization Guide](optimization-guide.md)
2. Review GPU profiling techniques
3. Check cuRobo performance best practices

## 🏗️ Architecture Overview

curobo_ros uses several design patterns:

- **Strategy Pattern**: Different planning algorithms (Classic, MPC, Batch, Constrained)
- **Factory Pattern**: Create planners dynamically based on configuration
- **Manager Pattern**: Cache and switch between planner instances
- **Observer Pattern**: ROS 2 topics for state updates and feedback

See [Architecture Patterns](architecture-patterns.md) for detailed explanation.

## 📦 Module Structure

```
curobo_ros/
├── core/                  # Core ROS 2 nodes
│   ├── unified_planner_node.py
│   ├── ik_solver_node.py
│   └── fk_solver_node.py
├── planners/              # Planning algorithms
│   ├── trajectory_planner.py     (ABC)
│   ├── classic_planner.py
│   ├── mpc_planner.py
│   ├── batch_planner.py
│   └── planner_factory.py
├── robot/                 # Robot strategies
│   ├── robot_strategy.py         (ABC)
│   ├── real_robot_strategy.py
│   ├── emulator_strategy.py
│   └── ghost_strategy.py
└── utils/                 # Utilities
    ├── collision_manager.py
    ├── camera_integration.py
    └── visualization.py
```

## 🧪 Testing Strategy

### Test Levels
1. **Unit Tests**: Test individual classes and methods
2. **Integration Tests**: Test component interactions
3. **System Tests**: End-to-end ROS 2 service/action tests
4. **Performance Tests**: Benchmark planning times

See [Testing Guide](testing-guide.md) for details.

## 📖 Implementation Guides

### Adding New Planner
1. Inherit from `TrajectoryPlanner` ABC
2. Implement `plan()` and `execute()` methods
3. Register in `PlannerFactory`
4. Add ROS 2 parameters
5. Write unit and integration tests

See [Architecture Patterns](architecture-patterns.md) for full example.

### Adding New Robot Strategy
1. Inherit from `RobotStrategy` ABC
2. Implement command execution methods
3. Add robot-specific configuration
4. Update launch files

See [Robot Strategies API](api-reference/robot-strategies.md).

## 🔗 External Resources

- **cuRobo Documentation**: https://curobo.org
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **NVIDIA Isaac Sim**: https://developer.nvidia.com/isaac-sim
- **Python Type Hints**: https://docs.python.org/3/library/typing.html

## 💡 Development Tips

- **Use type hints**: All code should use Python type hints
- **Document public APIs**: Use docstrings for all public methods
- **Write tests first**: TDD approach for new features
- **Profile before optimizing**: Use cProfile and nvprof
- **Follow ROS 2 conventions**: Node names, topic names, parameter naming

## 📞 Getting Help

- **GitHub Issues**: Report bugs and request features
- **Discussions**: Ask questions and share ideas
- **Code Review**: Submit PRs for community review

---

[← Back to Documentation Home](../README.md) | [Concepts](../concepts/) | [Robots →](../robots/)

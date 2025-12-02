# Concepts

Deep dive into curobo_ros architecture, design patterns, and technical concepts.

## 📚 Core Concepts

### Architecture
- **[System Architecture](architecture.md)** - Overall system design, components, and data flow
- **[Unified Planner Architecture](unified-planner.md)** - Flexible planning framework supporting multiple algorithms
- **[ROS 2 Interfaces](ros-interfaces.md)** - Complete reference of services, topics, actions, and messages

### Planning & Control
- **[Parameters Guide](parameters.md)** - Understand and tune all planning parameters
- **[Collision Detection](collision-detection.md)** - Voxel-based GPU collision checking (BLOX)
- **[GPU Optimization](gpu-optimization.md)** - Asynchronous warmup and GPU memory management

### Visualization
- **[RViz Plugin](rviz-plugin.md)** - Interactive trajectory preview and visualization

## 🎯 Concept Categories

### For Users
Understanding how the system works:
- [System Architecture](architecture.md) - High-level overview
- [Parameters Guide](parameters.md) - How to configure the system
- [ROS 2 Interfaces](ros-interfaces.md) - How to interact with the system

### For Advanced Users
Deep technical knowledge:
- [Unified Planner Architecture](unified-planner.md) - Multiple planning algorithms
- [Collision Detection](collision-detection.md) - How collision checking works
- [GPU Optimization](gpu-optimization.md) - Performance optimization

### For Integrators
System integration:
- [ROS 2 Interfaces](ros-interfaces.md) - Complete API reference
- [RViz Plugin](rviz-plugin.md) - Visualization setup

## 📖 Reading Order

**New to curobo_ros?** Read in this order:
1. [System Architecture](architecture.md) - Understand the big picture
2. [ROS 2 Interfaces](ros-interfaces.md) - Learn available services and topics
3. [Parameters Guide](parameters.md) - Learn how to configure planning

**Optimizing performance?** Focus on:
1. [Parameters Guide](parameters.md) - Tuning parameters
2. [GPU Optimization](gpu-optimization.md) - GPU efficiency
3. [Collision Detection](collision-detection.md) - Voxel size tuning

**Implementing advanced features?** Read:
1. [Unified Planner Architecture](unified-planner.md) - Multi-algorithm framework
2. [Development Guide](../development/) - Implementation details

## 🔗 Related Documentation

- **Tutorials**: [Step-by-step guides](../tutorials/) to learn by doing
- **Development**: [Technical implementation details](../development/) for contributors
- **Examples**: [Real-world examples](../tutorials/examples/) of complete systems

---

[← Back to Documentation Home](../README.md) | [Tutorials](../tutorials/) | [Development →](../development/)

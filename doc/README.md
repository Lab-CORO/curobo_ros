# curobo_ros Documentation

Complete documentation for curobo_ros - GPU-accelerated motion planning for ROS 2.

## 📚 Documentation Structure

Documentation is organized by audience and purpose:

### 🚀 [Getting Started](getting-started/)
**For new users** - Installation, quick start, and troubleshooting
- Installation & setup guide
- Quick start tutorial
- Introduction to ROS, Docker & cuRobo
- Docker workflow guide
- Troubleshooting common issues

### 🎓 [Tutorials](tutorials/)
**For learning** - Step-by-step guides to master curobo_ros features
1. Your first trajectory
2. Adding your robot
3. Managing collision objects
4. Dynamic strategy switching
5. MPC planner
6. IK/FK services
7. Point cloud obstacle detection

Plus complete examples: Doosan M1013, camera integration

### 📖 [Concepts](concepts/)
**For understanding** - Deep technical concepts and architecture
- System architecture
- Unified planner architecture
- ROS 2 interfaces (API reference)
- Parameters guide
- Collision detection
- GPU optimization
- RViz plugin

### 🔧 [Development](development/)
**For contributors** - Developer guides and API reference
- Architecture patterns
- MPC implementation guide
- Optimization guide
- Testing guide
- Migration guide
- Contributing guide
- API reference (Planners, Robot Strategies, ROS Nodes)

### 🤖 [Robots](robots/)
**For robot integration** - Robot-specific documentation
- Doosan M1013 (fully supported)
- Universal Robots (in progress)
- Custom robot integration guide
- Robot configuration reference

### 🗄️ [Archive](archive/)
Historical documentation and migration plans

## 🎯 Quick Navigation

### I want to...
- **Install curobo_ros** → [Getting Started: Installation](getting-started/installation.md)
- **Generate my first trajectory** → [Getting Started: Quick Start](getting-started/quick-start.md)
- **Learn step-by-step** → [Tutorials](tutorials/)
- **Integrate my robot** → [Tutorial: Adding Your Robot](tutorials/02-adding-your-robot.md)
- **Understand the architecture** → [Concepts: Architecture](concepts/architecture.md)
- **Tune parameters** → [Concepts: Parameters Guide](concepts/parameters.md)
- **Use MPC planner** → [Tutorial: MPC Planner](tutorials/05-mpc-planner.md)
- **Contribute code** → [Development: Contributing](development/contributing.md)
- **Add collision detection** → [Tutorial: Collision Objects](tutorials/03-collision-objects.md)
- **Use IK/FK services** → [Tutorial: IK/FK Services](tutorials/06-ik-fk-services.md)
- **Integrate cameras** → [Tutorial: Point Cloud Detection](tutorials/07-pointcloud-detection.md)
- **Troubleshoot issues** → [Getting Started: Troubleshooting](getting-started/troubleshooting.md)

## 📖 Learning Paths

### Path 1: New User (Beginner)
1. [Introduction to ROS, Docker & cuRobo](getting-started/introduction.md)
2. [Installation](getting-started/installation.md)
3. [Quick Start](getting-started/quick-start.md)
4. [Tutorial 1: Your First Trajectory](tutorials/01-first-trajectory.md)
5. [Tutorial 3: Collision Objects](tutorials/03-collision-objects.md)

### Path 2: Robot Integrator (Intermediate)
1. [Tutorial 1: Your First Trajectory](tutorials/01-first-trajectory.md)
2. [Tutorial 2: Adding Your Robot](tutorials/02-adding-your-robot.md)
3. [Custom Robot Guide](robots/custom-robot.md)
4. [Tutorial 4: Strategy Switching](tutorials/04-strategy-switching.md)
5. [Doosan Example](tutorials/examples/doosan-m1013.md)

### Path 3: Advanced User
1. [System Architecture](concepts/architecture.md)
2. [Unified Planner Architecture](concepts/unified-planner.md)
3. [Tutorial 5: MPC Planner](tutorials/05-mpc-planner.md)
4. [Tutorial 7: Point Cloud Detection](tutorials/07-pointcloud-detection.md)
5. [GPU Optimization](concepts/gpu-optimization.md)

### Path 4: Contributor/Developer (Advanced)
1. [Architecture Patterns](development/architecture-patterns.md)
2. [API Reference](development/api-reference/)
3. [Testing Guide](development/testing-guide.md)
4. [Contributing Guide](development/contributing.md)
5. [MPC Implementation Guide](development/mpc-implementation.md)

## 🎓 Documentation By Difficulty

### 🟢 Beginner
- [Getting Started](getting-started/) - All guides
- [Tutorial 1: First Trajectory](tutorials/01-first-trajectory.md)
- [Tutorial 3: Collision Objects](tutorials/03-collision-objects.md)
- [Tutorial 6: IK/FK Services](tutorials/06-ik-fk-services.md)

### 🟡 Intermediate
- [Tutorial 2: Adding Your Robot](tutorials/02-adding-your-robot.md)
- [Tutorial 4: Strategy Switching](tutorials/04-strategy-switching.md)
- [Tutorial 7: Point Cloud Detection](tutorials/07-pointcloud-detection.md)
- [Concepts: Parameters](concepts/parameters.md)
- [Concepts: ROS Interfaces](concepts/ros-interfaces.md)

### 🔴 Advanced
- [Tutorial 5: MPC Planner](tutorials/05-mpc-planner.md)
- [Concepts: Unified Planner](concepts/unified-planner.md)
- [Concepts: GPU Optimization](concepts/gpu-optimization.md)
- [Development Guides](development/) - All guides

## 🗺️ Documentation Map

```
curobo_ros/doc/
│
├── 🚀 getting-started/       # Start here if you're new
│   ├── installation.md
│   ├── quick-start.md
│   ├── introduction.md
│   ├── docker-workflow.md
│   └── troubleshooting.md
│
├── 🎓 tutorials/             # Learn by doing
│   ├── 01-first-trajectory.md
│   ├── 02-adding-your-robot.md
│   ├── 03-collision-objects.md
│   ├── 04-strategy-switching.md
│   ├── 05-mpc-planner.md
│   ├── 06-ik-fk-services.md
│   ├── 07-pointcloud-detection.md
│   └── examples/
│       ├── doosan-m1013.md
│       └── camera-integration.md
│
├── 📖 concepts/              # Understand the system
│   ├── architecture.md
│   ├── unified-planner.md
│   ├── ros-interfaces.md
│   ├── parameters.md
│   ├── collision-detection.md
│   ├── gpu-optimization.md
│   └── rviz-plugin.md
│
├── 🔧 development/           # Contribute and extend
│   ├── architecture-patterns.md
│   ├── mpc-implementation.md
│   ├── optimization-guide.md
│   ├── testing-guide.md
│   ├── migration-guide.md
│   ├── contributing.md
│   └── api-reference/
│       ├── planners.md
│       ├── robot-strategies.md
│       └── ros-nodes.md
│
├── 🤖 robots/                # Robot integration
│   ├── doosan-m1013.md
│   ├── universal-robots.md
│   ├── custom-robot.md
│   └── robot-configuration.md
│
└── 🗄️ archive/              # Historical docs
    ├── ARCHITECTURE_MIGRATION_PLAN.md
    └── MPC_DOCUMENTATION_CHANGELOG.md
```

## 🔍 Search Tips

- **Looking for a specific feature?** Check the "I want to..." section above
- **Not sure where to start?** Follow one of the Learning Paths
- **Need API reference?** See [ROS Interfaces](concepts/ros-interfaces.md) or [API Reference](development/api-reference/)
- **Troubleshooting?** Check [Troubleshooting Guide](getting-started/troubleshooting.md) first

## 📝 Documentation Conventions

- **File naming**: Uses kebab-case (e.g., `mpc-implementation.md`)
- **Tutorial numbering**: Sequential with zero-padding (01, 02, etc.)
- **Language**: All documentation in English
- **Code examples**: Tested and working
- **Cross-references**: Use relative links

## 🔗 External Resources

- **cuRobo**: https://curobo.org
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **NVIDIA Isaac Sim**: https://developer.nvidia.com/isaac-sim
- **Docker**: https://docs.docker.com/
- **GitHub Repository**: https://github.com/Lab-CORO/curobo_ros

## 🤝 Contributing to Documentation

Found an error or want to improve the docs?
1. Check [Contributing Guide](development/contributing.md)
2. Submit a pull request with your changes
3. Follow documentation conventions above

## 📞 Getting Help

- **Troubleshooting**: [Troubleshooting Guide](getting-started/troubleshooting.md)
- **GitHub Issues**: Report bugs or request features
- **Discussions**: Ask questions and share ideas

---

**Quick Links**: [Getting Started](getting-started/) | [Tutorials](tutorials/) | [Concepts](concepts/) | [Development](development/) | [Robots](robots/)

[← Back to Project README](../README.md)

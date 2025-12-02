# curobo_ros Documentation

Complete documentation for curobo_ros - GPU-accelerated motion planning for ROS 2.

## 📚 Documentation Structure

### 🚀 [Getting Started](getting-started/)
**For new users** - Installation, quick start, and troubleshooting
- Installation & setup guide
- Quick start tutorial
- Introduction to ROS, Docker & cuRobo
- Docker workflow guide
- Troubleshooting common issues

### 🎓 [Tutorials](tutorials/)
**Step-by-step guides** to master curobo_ros features
1. Your first trajectory
2. Adding your robot
3. Managing collision objects
4. Dynamic strategy switching
5. MPC planner
6. IK/FK services
7. Point cloud obstacle detection

Plus complete examples: Doosan M1013, camera integration

### 📖 [Concepts](concepts/)
**Technical concepts** and architecture
- System architecture
- Unified planner architecture
- ROS 2 interfaces (API reference)
- Parameters guide
- Collision detection
- GPU optimization
- RViz plugin

### 🤖 [Robots](robots/)
**Robot integration** - Robot-specific documentation
- Doosan M1013 (fully supported)
- Universal Robots (in progress)
- Custom robot integration guide
- Robot configuration reference

### 🗄️ [Archive](archive/)
Historical documentation and migration plans

## 🎯 Quick Navigation

### I want to...
- **Install curobo_ros** → [Getting Started](getting-started/)
- **Generate my first trajectory** → [Tutorial 1](tutorials/01-first-trajectory.md)
- **Integrate my robot** → [Tutorial 2](tutorials/02-adding-your-robot.md)
- **Understand the architecture** → [Concepts: Architecture](concepts/architecture.md)
- **Tune parameters** → [Concepts: Parameters](concepts/parameters.md)
- **Use MPC planner** → [Tutorial 5](tutorials/05-mpc-planner.md)
- **Add collision detection** → [Tutorial 3](tutorials/03-collision-objects.md)
- **Use IK/FK services** → [Tutorial 6](tutorials/06-ik-fk-services.md)
- **Integrate cameras** → [Tutorial 7](tutorials/07-pointcloud-detection.md)
- **Troubleshoot issues** → [Troubleshooting](getting-started/troubleshooting.md)

## 📖 Learning Paths

### Path 1: New User (Beginner)
1. [Getting Started](getting-started/)
2. [Tutorial 1: Your First Trajectory](tutorials/01-first-trajectory.md)
3. [Tutorial 3: Collision Objects](tutorials/03-collision-objects.md)

### Path 2: Robot Integrator
1. [Tutorial 1: Your First Trajectory](tutorials/01-first-trajectory.md)
2. [Tutorial 2: Adding Your Robot](tutorials/02-adding-your-robot.md)
3. [Tutorial 4: Strategy Switching](tutorials/04-strategy-switching.md)
4. [Doosan Example](tutorials/examples/doosan-m1013.md)

### Path 3: Advanced User
1. [System Architecture](concepts/architecture.md)
2. [Unified Planner](concepts/unified-planner.md)
3. [Tutorial 5: MPC Planner](tutorials/05-mpc-planner.md)
4. [GPU Optimization](concepts/gpu-optimization.md)

## 🗺️ Documentation Map

```
doc/
├── 🚀 getting-started/       # Installation & quick start
├── 🎓 tutorials/             # Step-by-step guides
│   └── examples/            # Complete integration examples
├── 📖 concepts/              # Technical concepts
├── 🤖 robots/                # Robot integration
└── 🗄️ archive/              # Historical docs
```

## 🔗 External Resources

- **cuRobo**: https://curobo.org
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **GitHub**: https://github.com/Lab-CORO/curobo_ros

---

**Quick Links**: [Getting Started](getting-started/) | [Tutorials](tutorials/) | [Concepts](concepts/) | [Robots](robots/)

[← Back to Project README](../README.md)

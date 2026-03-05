# curobo_ros

GPU-accelerated motion planning in **ROS 2**, powered by [curobo](https://curobo.org).

`curobo_ros` wraps the CUDA core of curobo with ROS 2 nodes, services, messages,
and an RViz plug-in, so you can:

- Solve IK/FK or whole-body trajectories in **milliseconds**
- Stream collision-aware paths directly to any robot controller
- Visualise everything live in RViz

A pre-built Docker image ships all dependencies (ROS 2 Humble, CUDA 12, curobo).

---

## Documentation

- **[Getting Started](getting-started/index.md)**
  - [Introduction](getting-started/introduction.md)
  - [Installation](getting-started/installation.md)
  - [Troubleshooting](getting-started/troubleshooting.md)
- **[Tutorials](tutorials/index.md)**
  - [1. Your First Trajectory](tutorials/01-first-trajectory.md)
  - [2. Adding Your Robot](tutorials/02-adding-your-robot.md)
  - [3. Managing Collision Objects](tutorials/03-collision-objects.md)
  - [4. Dynamic Strategy Switching](tutorials/04-strategy-switching.md)
  - [5. MPC Planner](tutorials/05-mpc-planner.md)
  - [6. IK/FK Services](tutorials/06-ik-fk-services.md)
  - [7. Point Cloud Obstacle Detection](tutorials/07-pointcloud-detection.md)
  - **[Examples](tutorials/examples/index.md)**
    - [Doosan M1013 Complete Integration](tutorials/examples/doosan-m1013.md)
    - [Camera Integration](tutorials/examples/camera-integration.md)
- **[Concepts](concepts/index.md)**
  - [System Architecture](concepts/architecture.md)
  - [Manager Architecture](concepts/manager-architecture.md)
  - [Unified Planner Architecture](concepts/unified-planner.md)
  - [Parameters Guide](concepts/parameters.md)
  - [MPC Implementation](concepts/mpc-implementation.md)
  - [ROS 2 Interfaces](concepts/ros-interfaces.md)
  - [RViz Plugin](concepts/rviz-plugin.md)
- **[Robots](robots/index.md)**
  - [Doosan M1013](robots/doosan-m1013.md)

---

```{toctree}
:maxdepth: 3
:caption: Getting Started
:hidden:
getting-started/index
```

```{toctree}
:maxdepth: 3
:caption: Tutorials
:hidden:
tutorials/index
```

```{toctree}
:maxdepth: 3
:caption: Concepts
:hidden:
concepts/index
```

```{toctree}
:maxdepth: 3
:caption: Robots
:hidden:
robots/index
```

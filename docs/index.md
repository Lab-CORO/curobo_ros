# curobo_ros

GPU-accelerated motion planning in **ROS 2**, powered by [cuRobo](https://curobo.org) v0.8.0 (v2 API).

`curobo_ros` wraps the CUDA core of cuRobo with a ROS 2 node, services, an action, and an RViz plugin, so you can:

- Solve IK/FK and collision-free trajectories in **milliseconds** on GPU
- Run **closed-loop reactive control** (MPC, teleoperation) with live goal retargeting
- Feed depth cameras into a shared GPU voxel world for **automatic obstacle avoidance**
- Stream trajectories to a real robot, an emulator, or an RViz preview — switchable at runtime

Docker images ship all dependencies (ROS 2 Jazzy, CUDA, cuRobo) for x86 and Jetson.

## Documentation

- **[Getting Started](getting-started/index.md)**
  - [Introduction](getting-started/introduction.md)
  - [Installation](getting-started/installation.md)
  - [Testing](getting-started/testing.md)
  - [Troubleshooting](getting-started/troubleshooting.md)
- **[Tutorials](tutorials/index.md)**
  - [1. Your First Trajectory](tutorials/01-first-trajectory.md)
  - [2. Adding Your Robot](tutorials/02-adding-your-robot.md)
  - [3. Managing Collision Objects](tutorials/03-collision-objects.md)
  - [4. Robot Execution](tutorials/04-robot-execution.md)
  - [5. MPC and Reactive Control](tutorials/05-mpc-planner.md)
  - [6. IK/FK Services](tutorials/06-ik-fk-services.md)
  - [7. Camera-Based Obstacle Detection](tutorials/07-pointcloud-detection.md)
- **[Concepts](concepts/index.md)**
  - [Package Architecture](concepts/architecture.md)
  - [Manager Architecture](concepts/manager-architecture.md)
  - [Unified Planner](concepts/unified-planner.md)
  - [MPC Implementation](concepts/mpc-implementation.md)
  - [Parameters Guide](concepts/parameters.md)
  - [ROS 2 Interfaces](concepts/ros-interfaces.md)
  - [RViz Plugin](concepts/rviz-plugin.md)
- **[Migration to cuRobo v2](MIGRATION_V2.md)** — for developers of the wrapper itself

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
:caption: Reference
:hidden:

MIGRATION_V2
```

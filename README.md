# Curobo ROS

GPU-accelerated motion planning in **ROS 2**—powered by [curobo](https://curobo.org).

`curobo_ros` wraps the CUDA core of curobo with ROS 2 nodes, services, messages
and an RViz plug-in, so you can:

* solve IK/FK or whole-body trajectories in **milliseconds**,
* stream collision-aware paths directly to any robot controller,
* visualise everything live in RViz,

A pre-built Docker image ships all dependencies (ROS 2 Humble, CUDA 12, curobo)

---

## Documentation structure

- [Getting started](doc/getting_started.md)
- Concepts
   - [Todo Architecture](doc/concepts/architecture.md)
   - [Todo Ros interfaces](doc/concepts/ros_interfaces.md)
   - [Todo Rviz plugin](doc/concepts/rviz_plugin.md)
- Tutorials
  - Todo
- [Troubleshooting](doc/troubleshooting.md) 

---

## License



# curobo_ros

GPU-accelerated motion planning in **ROS 2**, powered by [curobo](https://curobo.org).

`curobo_ros` wraps the CUDA core of curobo with ROS 2 nodes, services, messages,
and an RViz plug-in, so you can:

- Solve IK/FK or whole-body trajectories in **milliseconds**
- Stream collision-aware paths directly to any robot controller
- Visualise everything live in RViz

A pre-built Docker image ships all dependencies (ROS 2 Humble, CUDA 12, curobo).

---

```{toctree}
:maxdepth: 2
:caption: Getting Started

getting-started/index
```

```{toctree}
:maxdepth: 2
:caption: Tutorials

tutorials/index
```

```{toctree}
:maxdepth: 2
:caption: Concepts

concepts/index
```

```{toctree}
:maxdepth: 2
:caption: Robots

robots/index
```

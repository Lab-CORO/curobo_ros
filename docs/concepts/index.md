# Concepts

Design-level documentation: how `curobo_ros` is structured, how its control loops work, and the normative reference for every interface and parameter.

## Architecture

- **[Package Architecture](architecture.md)** — the three strategy families (planners, robot control, cameras) and how they compose
- **[Manager Architecture](manager-architecture.md)** — the configuration layer, concurrency invariants, and observer wiring (contributor-oriented)
- **[Unified Planner](unified-planner.md)** — the planner catalog, open-loop vs closed-loop execution, runtime switching

## Planning & control

- **[MPC Implementation](mpc-implementation.md)** — the closed-loop reactive control path (MPC and retargeting)
- **[Parameters Guide](parameters.md)** — every parameter with its real default and when changes take effect

## Interfaces

- **[ROS 2 Interfaces](ros-interfaces.md)** — the complete reference of services, action, and topics
- **[RViz Plugin](rviz-plugin.md)** — the graphical interface and what it calls underneath

```{toctree}
:maxdepth: 2
:hidden:

architecture
manager-architecture
unified-planner
mpc-implementation
parameters
ros-interfaces
rviz-plugin
```

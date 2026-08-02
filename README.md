# curobo_ros

📖 **Full documentation: [lab-coro.github.io/curobo_ros](https://lab-coro.github.io/curobo_ros/index.html)**

GPU-accelerated motion planning for **ROS 2** — powered by [cuRobo](https://curobo.org) **v0.8.0 (v2 API)**.

`curobo_ros` wraps NVIDIA's CUDA-accelerated cuRobo library with ROS 2 services, an action, and topics:

- ⚡ **Millisecond planning** — IK/FK and full trajectories solved on GPU
- 🔁 **Closed-loop reactive control** — MPC and IK-based teleoperation, with live goal retargeting
- 🚧 **Camera-based collision avoidance** — depth cameras feed a GPU voxel map (ESDF) shared by all solvers
- 🎯 **One node, one interface** — switch planners and robot backends at runtime through services
- 🎨 **RViz integration** — interactive 6-DOF goal marker, trajectory preview, obstacle management
- 🐳 **Docker images** — ROS 2 Jazzy + CUDA, for x86 and Jetson (aarch64)

## Features

- **Unified planner node** — one node (`unified_planner`) hosts every algorithm behind a single `generate_trajectory` service and `execute_trajectory` action
- **Five planners** — `classic` (single pose), `multi_point` (waypoints), `joint_space`, `mpc` (Model Predictive Control, closed-loop), `retarget` (teleoperation follower); switch at runtime with `set_planner`
- **Robot backends as strategies** — emulator, velocity streaming, or position streaming to a real robot, switchable at runtime; bring your own robot with a YAML descriptor
- **Obstacle management** — primitives and meshes, object attachment to the flange, per-link collision toggling, collision distance queries, voxel grid introspection
- **Batch IK/FK services** — GPU-parallel kinematics with collision-aware IK
- **Test suite** — auto-generated integration tests covering planning, collisions, kinematics, and planner switching

## Quick start

See [Getting Started](https://lab-coro.github.io/curobo_ros/getting-started/index.html) for the full walkthrough (Docker build, workspace setup, first trajectory). In short:

```bash
git clone --recurse-submodules https://github.com/Lab-CORO/curobo_ros.git
vcs import < curobo_ros/my.repos           # pulls curobo_msgs + curobo_rviz
# build the Docker image for your platform (x86 or Jetson), then inside it:
colcon build && source install/setup.bash
ros2 launch curobo_ros gen_traj.launch.py robot:=emulator
```

## Documentation

- [Getting Started](docs/getting-started/index.md) — introduction, installation, testing, troubleshooting
- [Tutorials](docs/tutorials/index.md) — first trajectory, adding your robot, obstacles, execution, MPC, IK/FK, cameras
- [Concepts](docs/concepts/index.md) — architecture, planners, parameters, complete ROS interface reference
- [Migration to cuRobo v2](docs/MIGRATION_V2.md) — v1 → v2 API mapping for wrapper developers

## Contributing

Contributions are welcome: fork, create a feature branch, and open a pull request. For robot integration, see the [Adding Your Robot tutorial](docs/tutorials/02-adding-your-robot.md).

## Citation

If you use curobo_ros in your research, please cite cuRobo:

```bibtex
@article{sundaralingam2023curobo,
  title={cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation},
  author={Sundaralingam, Balakumar and Hari, Siva Kumar Sastry and Fishman, Adam and Garrett, Caelan and Van Wyk, Karl and Blukis, Valts and Millane, Alexander and Oleynikova, Helen and Handa, Ankur and Ramos, Fabio and others},
  journal={arXiv preprint arXiv:2310.17274},
  year={2023}
}
```

## Support

- **Documentation**: [lab-coro.github.io/curobo_ros](https://lab-coro.github.io/curobo_ros/index.html)
- **Issues**: [GitHub Issues](https://github.com/Lab-CORO/curobo_ros/issues)
- **cuRobo**: [curobo.org](https://curobo.org)
- **ROS 2**: [docs.ros.org](https://docs.ros.org/en/jazzy/)

## License

Licensed under the [Apache License 2.0](LICENSE).

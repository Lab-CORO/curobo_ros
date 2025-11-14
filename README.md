# curobo_ros

GPU-accelerated motion planning for **ROS 2** — powered by [cuRobo](https://curobo.org).

`curobo_ros` wraps NVIDIA's CUDA-accelerated cuRobo library with ROS 2 nodes, services, actions, and messages, enabling:

- ⚡ **Millisecond motion planning** - Solve IK/FK or full trajectories in ~1-50ms on GPU
- 🚧 **Real-time collision avoidance** - Voxel-based GPU collision checking with camera integration
- 🎯 **Simple ROS 2 interface** - Services, actions, and topics for easy integration
- 🎨 **Live visualization** - RViz integration with trajectory preview
- 🐳 **Ready-to-use Docker** - Pre-built image with ROS 2 Humble, CUDA 12, and all dependencies

---

## Quick Start

**New to ROS or Docker?** Start here: [Introduction](doc/concepts/introduction.md) | [Docker Workflow](doc/concepts/docker_workflow.md)

```bash
# 1. Clone repository (requires ~30 GB disk space)
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules
cd curobo_ros/docker

# 2. Build Docker image (~20-30 minutes)
bash build_docker.sh x86

# 3. Start container
bash start_docker_x86.sh

# 4. Inside container: Launch the system
cd /home/ros2_ws
source install/setup.bash
ros2 launch curobo_ros unified_planner.launch.py

# 5. In another terminal: Generate your first trajectory
docker exec -it x86docker bash
source /home/ros2_ws/install/setup.bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

See the [Getting Started Guide](doc/getting_started.md) for detailed instructions.

---

## Documentation

### 🚀 Getting Started
- **[Introduction](doc/concepts/introduction.md)** - What is ROS, Docker, and cuRobo? (Start here if you're new!)
- **[Docker Workflow](doc/concepts/docker_workflow.md)** - How to work efficiently with Docker containers
- **[Getting Started](doc/getting_started.md)** - Complete setup guide from installation to first trajectory
- **[Troubleshooting](doc/troubleshooting.md)** - Common issues and solutions

### 📚 Step-by-Step Tutorials
1. **[Your First Trajectory](doc/tutorials/1_first_trajectory.md)** - Generate trajectories, add obstacles, tune parameters
2. **[Adding Your Robot](doc/tutorials/2_adding_your_robot.md)** - Integrate your robot (Doosan M1013 example)
3. **[Managing Obstacles](doc/tutorials/adding_collision_objects.md)** - Dynamic obstacle management and collision checking
4. **[Dynamic Strategy Switching](doc/tutorials/4_dynamic_strategy_switching.md)** - Switch between real robot, emulator, and simulation
5. **[MPC Planner](doc/tutorials/5_mpc_planner.md)** - Real-time reactive trajectory planning with Model Predictive Control
6. **[IK/FK Services](doc/tutorials/ik_fk_services.md)** - Use inverse and forward kinematics services
7. **[Point Cloud Obstacle Detection](doc/tutorials/pointcloud_obstacle_detection.md)** - Integrate cameras for dynamic obstacle detection

### 📖 Concepts & Reference
- **[Unified Planner Architecture](doc/concepts/unified_planner.md)** - Flexible planning framework supporting multiple algorithms
- **[Parameters Guide](doc/concepts/parameters.md)** - Understand `voxel_size`, `time_dilation_factor`, and all parameters
- **[Async Warmup](doc/concepts/warmup_async.md)** - GPU optimization and asynchronous initialization
- **[ROS Interfaces](doc/concepts/ros_interfaces.md)** - Complete reference of services, topics, and actions
- **[Architecture](doc/concepts/architecture.md)** - System architecture and class diagrams

### 🤖 Examples
- **[Doosan M1013 Example](doc/tutorials/doosan_example.md)** - Complete example with Doosan collaborative robot

---

## Features

- ✅ **GPU-Accelerated Planning** - CUDA-based trajectory optimization for real-time performance
- ✅ **Unified Planner Architecture** - Flexible framework supporting multiple planning algorithms (Classic, MPC, Batch, Constrained)
- 📋 **MPC Real-Time Planning** - Model Predictive Control for reactive, closed-loop trajectory execution _(specification ready, implementation planned)_
- ✅ **Collision Avoidance** - Voxel-based collision checking (BLOX) with dynamic obstacle management
- ✅ **Multiple Robot Support** - Easy integration of custom robots via YAML configuration
- ✅ **Strategy Switching** - Dynamically switch between real robot, emulator, and visualization modes
- ✅ **Camera Integration** - Point cloud and depth camera support for automatic obstacle detection
- ✅ **Batch IK/FK** - Solve multiple poses simultaneously with GPU parallelization
- ✅ **RViz Visualization** - Live trajectory preview and collision sphere visualization
- ✅ **Flexible Interfaces** - Services for planning, actions for execution, topics for streaming

---

## Requirements

| Requirement | Notes |
|-------------|-------|
| **Ubuntu 20.04 / 22.04** | Or Windows 11 + WSL2 |
| **NVIDIA GPU** | With recent drivers (check with `nvidia-smi`) |
| **~30 GB disk space** | For Docker image and dependencies |
| **Docker ≥ 24** | [Install Docker](https://docs.docker.com/engine/install/) |
| **NVIDIA Container Toolkit** | [Install Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |

---

## Supported Robots

| Robot | Status | Package |
|-------|--------|---------|
| **Doosan M1013** | ✅ Full support | [curobo_doosan](https://github.com/Lab-CORO/curobo_doosan) |
| **Universal Robots UR5e** | ⚠️ In progress | - |
| **Custom robots** | ✅ Via YAML config | [Tutorial](doc/tutorials/2_adding_your_robot.md) |

---

## Architecture Overview

```
┌─────────────────────────────────────────┐
│       Your Application / RViz            │
│         (Python, C++, GUI)              │
└──────────────┬──────────────────────────┘
               │ ROS 2 Services/Actions/Topics
               ▼
┌──────────────────────────────────────────┐
│         unified_planner Node             │
│  • Motion planning                       │
│  • Obstacle management                   │
│  • Robot strategy handling               │
└──────────────┬───────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────┐
│       cuRobo (GPU Library)               │
│  • Trajectory optimization               │
│  • IK/FK solving                         │
│  • Collision checking (BLOX)             │
└──────────────┬───────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────┐
│      Robot Control Strategy              │
│  • Real robot (Doosan, UR5e)             │
│  • Emulator (simulation)                 │
│  • Ghost (visualization only)            │
└──────────────────────────────────────────┘
```

See [Architecture](doc/concepts/architecture.md) for details.

---

## Key Services & Actions

| Service/Action | Type | Description |
|----------------|------|-------------|
| `/unified_planner/generate_trajectory` | Service | Generate collision-free trajectory |
| `/unified_planner/send_trajectrory` | Action | Execute trajectory with feedback |
| `/unified_planner/add_object` | Service | Add collision object |
| `/unified_planner/remove_all_objects` | Service | Clear all obstacles |
| `/unified_planner/set_robot_strategy` | Service | Switch robot control mode |
| `/curobo_ik/ik_pose` | Service | Solve inverse kinematics |
| `/curobo_fk/fk_poses` | Service | Solve forward kinematics |

See [ROS Interfaces](doc/concepts/ros_interfaces.md) for complete reference.

---

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

For robot integration, see [Adding Your Robot Tutorial](doc/tutorials/2_adding_your_robot.md).

---

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

---

## Support

- **Documentation**: [docs/](doc/)
- **Issues**: [GitHub Issues](https://github.com/Lab-CORO/curobo_ros/issues)
- **cuRobo**: [curobo.org](https://curobo.org)
- **ROS 2**: [docs.ros.org](https://docs.ros.org/en/humble/)

---

## License



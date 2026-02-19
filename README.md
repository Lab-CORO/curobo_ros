# curobo_ros

📖 **Full documentation: [lab-coro.github.io/curobo_ros](https://lab-coro.github.io/curobo_ros/index.html)**

GPU-accelerated motion planning for **ROS 2** — powered by [cuRobo](https://curobo.org).

`curobo_ros` wraps NVIDIA's CUDA-accelerated cuRobo library with ROS 2 nodes, services, actions, and messages, enabling:

- ⚡ **Millisecond motion planning** - Solve IK/FK or full trajectories in ~1-50ms on GPU
- 🚧 **Real-time collision avoidance** - Voxel-based GPU collision checking with camera integration
- 🎯 **Simple ROS 2 interface** - Services, actions, and topics for easy integration
- 🎨 **Live visualization** - RViz integration with trajectory preview
- 🐳 **Ready-to-use Docker** - Pre-built image with ROS 2 Humble, CUDA 12, and all dependencies


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

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

For robot integration, see [Adding Your Robot Tutorial](doc/tutorials/02-adding-your-robot.md).

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



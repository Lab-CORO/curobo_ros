# Tutorials

Step-by-step tutorials to learn curobo_ros features and workflows.

## 📚 Core Tutorials

Follow these tutorials in order to learn the fundamentals:

1. **[Your First Trajectory](01-first-trajectory.md)**
   - Generate collision-free trajectories
   - Add obstacles to the environment
   - Tune planning parameters
   - Visualize trajectories in RViz

2. **[Adding Your Robot](02-adding-your-robot.md)**
   - Integrate a custom robot (Doosan M1013 example)
   - Configure robot URDF and kinematics
   - Set up collision geometry
   - Test your robot configuration

3. **[Managing Collision Objects](03-collision-objects.md)**
   - Add and remove dynamic obstacles
   - Update obstacle positions in real-time
   - Configure collision checking parameters
   - Use voxel-based collision detection

4. **[Dynamic Strategy Switching](04-strategy-switching.md)**
   - Switch between real robot, emulator, and ghost mode
   - Configure different execution strategies
   - Use visualization mode for testing
   - Handle multiple robots

5. **[MPC Planner](05-mpc-planner.md)**
   - Real-time reactive trajectory planning
   - Model Predictive Control for closed-loop execution
   - Handle dynamic obstacles during execution
   - Tune MPC parameters

6. **[IK/FK Services](06-ik-fk-services.md)**
   - Use inverse kinematics services
   - Solve forward kinematics
   - Batch IK/FK for multiple poses
   - GPU-accelerated solving

7. **[Point Cloud Obstacle Detection](07-pointcloud-detection.md)**
   - Integrate depth cameras (RealSense, Kinect)
   - Automatic obstacle detection from point clouds
   - Real-time environment updates
   - Camera calibration and setup

## 🤖 Complete Examples

Real-world integration examples:

- **[Doosan M1013 Example](examples/doosan-m1013.md)** - Complete setup with Doosan collaborative robot
- **[Camera Integration Example](examples/camera-integration.md)** - Generate trajectories with camera-based obstacle detection

## 🎯 Tutorial Difficulty

| Tutorial | Difficulty | Time | Prerequisites |
|----------|-----------|------|---------------|
| 1. First Trajectory | 🟢 Beginner | 15 min | Installation complete |
| 2. Adding Your Robot | 🟡 Intermediate | 45 min | Tutorial 1 |
| 3. Collision Objects | 🟢 Beginner | 20 min | Tutorial 1 |
| 4. Strategy Switching | 🟡 Intermediate | 30 min | Tutorial 1, 2 |
| 5. MPC Planner | 🔴 Advanced | 60 min | Tutorial 1, 3 |
| 6. IK/FK Services | 🟢 Beginner | 15 min | Tutorial 1 |
| 7. Point Cloud Detection | 🟡 Intermediate | 45 min | Tutorial 1, 3 |

## 💡 Tips for Success

- **Work inside Docker**: All tutorials assume you're running inside the Docker container
- **Check prerequisites**: Make sure you've completed prerequisite tutorials
- **Read error messages**: The system provides helpful error messages with solutions
- **Start simple**: Test basic functionality before adding complexity
- **Use RViz**: Visualization helps understand what's happening

## 📖 What's Next?

After completing the tutorials:
- Explore [Concepts](../concepts/) for deeper technical understanding
- Check [Robot Integration](../robots/) for specific robot configurations
- Read [MPC Implementation](../concepts/mpc-implementation.md) for advanced planning

---

[← Back to Documentation Home](../README.md) | [Getting Started](../getting-started/) | [Concepts →](../concepts/)

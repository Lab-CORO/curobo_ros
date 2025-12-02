# Tutorial Examples

Complete, real-world examples of curobo_ros integration.

## 📚 Available Examples

### [Doosan M1013 Complete Integration](doosan-m1013.md)
**Difficulty**: 🟡 Intermediate | **Time**: 2-3 hours

Complete example of integrating a Doosan M1013 collaborative robot with curobo_ros:
- Hardware setup and connection
- Robot configuration with curobo_robot_setup
- Real robot control vs. emulator vs. ghost mode
- Collision avoidance configuration
- Camera integration for dynamic obstacles
- Production deployment tips

**What you'll learn**:
- Complete robot integration workflow
- Strategy switching (real/emulator/ghost)
- Safety and collision configuration
- Real-world deployment practices

**Prerequisites**:
- Completed [Tutorial 1: First Trajectory](../01-first-trajectory.md)
- Completed [Tutorial 2: Adding Your Robot](../02-adding-your-robot.md)
- Access to Doosan M1013 (or similar robot)

---

### [Camera Integration Example](camera-integration.md)
**Difficulty**: 🟡 Intermediate | **Time**: 1-2 hours

Example of integrating depth cameras for automatic obstacle detection:
- Intel RealSense D435 setup
- Point cloud processing and filtering
- Real-time obstacle updates during trajectory execution
- Camera calibration and workspace setup
- Performance optimization

**What you'll learn**:
- Camera driver configuration
- Point cloud to voxel conversion
- Dynamic environment updates
- Latency optimization

**Prerequisites**:
- Completed [Tutorial 1: First Trajectory](../01-first-trajectory.md)
- Completed [Tutorial 7: Point Cloud Detection](../07-pointcloud-detection.md)
- Depth camera hardware (RealSense, Kinect, etc.)

---

## 🎯 How to Use These Examples

### 1. Study First
Read through the entire example before starting to understand the complete workflow.

### 2. Adapt to Your Setup
These examples are based on specific hardware but can be adapted:
- Replace robot model with your own
- Use different camera models
- Adjust parameters for your workspace

### 3. Follow Step-by-Step
Work through the example sequentially:
- Setup and installation
- Configuration
- Testing
- Integration
- Production deployment

### 4. Troubleshooting
Each example includes a troubleshooting section for common issues.

## 💡 Tips for Success

- **Start with simulation**: Use ghost or emulator mode before testing with real hardware
- **Test incrementally**: Verify each component before integration
- **Keep backups**: Save working configurations before making changes
- **Monitor performance**: Use RViz visualization to verify behavior
- **Safety first**: Always have emergency stop ready when testing with real robots

## 🔗 Related Documentation

### Before Starting
- [Tutorial 2: Adding Your Robot](../02-adding-your-robot.md)
- [Robot Integration Guide](../../robots/)

### During Development
- [Parameters Guide](../../concepts/parameters.md)
- [ROS Interfaces](../../concepts/ros-interfaces.md)
- [Troubleshooting](../../getting-started/troubleshooting.md)

### After Integration
- [Optimization Guide](../../development/optimization-guide.md)
- [Testing Guide](../../development/testing-guide.md)

## 🤝 Share Your Example

Have you integrated curobo_ros with another robot or setup? We'd love to add your example!

1. Document your integration following the format of existing examples
2. Include configuration files and launch files
3. Add troubleshooting tips
4. Submit a pull request

See [Contributing Guide](../../development/contributing.md).

---

[← Back to Tutorials](../) | [Documentation Home](../../README.md)

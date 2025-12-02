# Robot Integration

Documentation for integrating robots with curobo_ros.

## 🤖 Supported Robots

### Fully Supported
- **[Doosan M1013](doosan-m1013.md)** - Doosan collaborative robot with full integration
  - Status: ✅ Production ready
  - Package: [curobo_doosan](https://github.com/Lab-CORO/curobo_doosan)
  - Features: Real robot control, emulator support, collision avoidance

### In Development
- **Universal Robots** - UR5e, UR10e, UR16e
  - Status: 🚧 In progress
  - Expected: Q1 2026

### Custom Integration
See [Tutorial: Adding Your Robot](../tutorials/02-adding-your-robot.md) for step-by-step integration guide

## 📚 Integration Documentation

### Getting Started
1. **[Adding Your Robot Tutorial](../tutorials/02-adding-your-robot.md)** - Step-by-step guide to integrate your robot
2. **[Doosan M1013 Example](doosan-m1013.md)** - Complete integration reference

## 🎯 Integration Quick Start

### 1. Prerequisites
- Robot URDF file
- Joint limits and capabilities
- End-effector TCP offset (if any)
- Collision geometry (optional but recommended)

### 2. Basic Steps
1. Prepare your robot URDF
2. Generate cuRobo configuration YAML
3. Create launch file
4. Test IK/FK
5. Generate first trajectory

See [Adding Your Robot Tutorial](../tutorials/02-adding-your-robot.md) for detailed steps.

### 3. Tools Available
- **curobo_robot_setup** - RViz plugin for interactive robot configuration
- **NVIDIA Isaac Sim** - Advanced robot configuration and testing
- **Manual YAML** - Direct configuration file editing

## 📋 Robot Configuration Requirements

### Minimum Requirements
```yaml
robot:
  urdf_path: /path/to/robot.urdf
  base_link: base_link
  ee_link: tool0

kinematics:
  cspace:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    position_bounds: [[-2.9, 2.9], [-1.7, 1.7], ...]
    velocity_bounds: [[3.14], [3.14], ...]
```

### Optional Configuration
- Collision geometry (spheres, capsules)
- Self-collision pairs
- External collision meshes
- End-effector offset
- Custom joint limits

See [Adding Your Robot Tutorial](../tutorials/02-adding-your-robot.md) for configuration details.

## 🔧 Integration Workflow

```
1. Robot URDF → 2. Configuration YAML → 3. Launch File → 4. Testing → 5. Production
```

### 1. Robot URDF
- Obtain or create accurate URDF
- Verify joint names and limits
- Include collision meshes (optional)

### 2. Configuration YAML
Use one of these methods:
- **curobo_robot_setup RViz plugin** (recommended) - Interactive visual configuration
- **NVIDIA Isaac Sim** - Advanced simulation and testing
- **Manual editing** - Direct YAML file creation

### 3. Launch File
Create a launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curobo_ros',
            executable='unified_planner',
            parameters=[{
                'robot_config': '/path/to/robot.yaml',
                'robot_strategy': 'real',  # or 'emulator', 'ghost'
            }]
        )
    ])
```

### 4. Testing
1. Test IK/FK services
2. Generate simple trajectories
3. Add collision objects
4. Test with your robot controller

### 5. Production
- Integrate with your application
- Tune parameters for your use case
- Add safety layers if needed

## 🤖 Robot Examples

### Doosan M1013
Complete production-ready integration:
- Real robot control via Doosan API
- Software emulator for testing
- Ghost mode for visualization
- Collision avoidance with cameras

[See full documentation →](doosan-m1013.md)

### Universal Robots UR5e
Work in progress:
- ROS 2 control integration
- MoveIt 2 compatibility
- Real-time control

Status: 🚧 In development (Q1 2026)

## 📖 Advanced Topics

### Multi-Robot Systems
- Configure multiple robots in one workspace
- Shared collision environment
- Coordinated planning

### Robot Strategies
Different execution modes:
- **Real Robot**: Control physical robot hardware
- **Emulator**: Software simulation for testing
- **Ghost**: Visualization only (no execution)

See [Strategy Switching Tutorial](../tutorials/04-strategy-switching.md).

### Camera Integration
Add dynamic obstacle detection:
- Intel RealSense
- Microsoft Kinect
- Generic depth cameras

See [Point Cloud Detection Tutorial](../tutorials/07-pointcloud-detection.md).

## 🔗 Related Documentation

- **Tutorial**: [Adding Your Robot](../tutorials/02-adding-your-robot.md)
- **Tutorial**: [Strategy Switching](../tutorials/04-strategy-switching.md)
- **Concepts**: [System Architecture](../concepts/architecture.md)

## 💡 Integration Tips

- Start with IK/FK testing before full trajectory planning
- Use ghost mode first to verify configuration
- Test with simple poses before complex trajectories
- Gradually add collision geometry for accuracy vs. performance
- Use curobo_robot_setup plugin for quickest setup

## 📞 Need Help?

- Check [Troubleshooting](../getting-started/troubleshooting.md)
- Review [Doosan example](doosan-m1013.md) as reference
- Open GitHub issue with `[Robot Integration]` tag
- Join discussions for community support

---

[← Back to Documentation Home](../README.md) | [Tutorials](../tutorials/) | [Concepts →](../concepts/)

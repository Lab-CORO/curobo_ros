# Tutorial: Adding Your Robot

This tutorial shows you how to integrate your own robot with curobo_ros. We'll use the **Doosan M1013** as a complete example, walking through:

- Importing the robot package
- Understanding the robot configuration file
- Creating your own configuration
- Testing with your robot

---

## Overview

To use curobo_ros with your robot, you need:

1. **URDF file** - Robot description with kinematics and geometry
2. **Configuration YAML** - cuRobo-specific parameters (joint limits, collision spheres, etc.)
3. **Robot package** (optional) - ROS package containing URDF and meshes

---

## Example: Doosan M1013

We'll use the Doosan M1013 collaborative robot as our example. The Lab-CORO has a pre-built package for this robot.

###  Step 1: Import the Robot Package

The Doosan M1013 configuration is in a separate package: `curobo_doosan`.

```bash
# Navigate to your workspace (on host machine, NOT inside Docker)
cd ~/ros2_ws/src

# Clone the package
git clone https://github.com/Lab-CORO/curobo_doosan.git

# The package is now available in the Docker container via volume mount!
```

**What's in the package:**
```
curobo_doosan/
├── config/
│   └── m1013.yml          # cuRobo configuration
├── urdf/
│   └── m1013.urdf         # Robot description
├── meshes/
│   └── *.stl              # Visual/collision meshes
└── launch/
    └── display.launch.py  # Visualization launch file
```

### Step 2: Build the Workspace

```bash
# Inside the Docker container
cd /home/ros2_ws

# Build the new package
colcon build --packages-select curobo_doosan

# Source the workspace
source install/setup.bash
```

### Step 3: Verify the URDF

Before using with cuRobo, verify the URDF loads correctly:

```bash
# Check robot description
ros2 launch curobo_doosan display.launch.py

# RViz should open showing the Doosan M1013
```

**Check:**
- All joints move correctly
- Base frame is correct
- End-effector frame exists
- Collision meshes look reasonable

### Step 4: Launch with Your Robot

```bash
# Launch curobo_ros with Doosan M1013 config
ros2 launch curobo_ros gen_traj.launch.py \
  robot_config_file:=$(ros2 pkg prefix curobo_doosan)/config/m1013.yml
```

**What this does:**
- Loads the m1013.yml configuration
- Reads the URDF from the path specified in the config
- Initializes cuRobo with Doosan kinematics
- Starts trajectory planning node

### Step 5: Test Trajectory Generation

```bash
# Open a new terminal
docker exec -it x86docker bash
source /home/ros2_ws/install/setup.bash

# Generate a trajectory
ros2 service call /curobo_gen_traj/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

**Success!** You're now planning trajectories for the Doosan M1013.

---

## Understanding the Configuration File

Let's examine what's in `m1013.yml`:

### Full Configuration Structure

```yaml
robot:
  # Robot kinematics
  kinematics:
    urdf_path: "path/to/m1013.urdf"  # Absolute or relative path
    base_link: "base_0"               # Robot base frame
    ee_link: "link6"                  # End-effector frame

  # Joint limits
  cspace:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    position_limits:
      - [-3.14, 3.14]    # Joint 1 (radians)
      - [-2.27, 2.27]    # Joint 2
      - [-1.92, 1.92]    # Joint 3
      - [-3.14, 3.14]    # Joint 4
      - [-2.09, 2.09]    # Joint 5
      - [-6.28, 6.28]    # Joint 6
    velocity_limits: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]        # rad/s
    acceleration_limits: [15.0, 15.0, 15.0, 15.0, 15.0, 15.0]  # rad/s²
    jerk_limits: [500.0, 500.0, 500.0, 500.0, 500.0, 500.0]   # rad/s³

  # Collision model (simplified robot as spheres)
  collision_spheres:
    link1:
      - center: [0.0, 0.0, 0.15]
        radius: 0.08
    link2:
      - center: [0.1, 0.0, 0.0]
        radius: 0.06
      - center: [0.2, 0.0, 0.0]
        radius: 0.06
    # ... more spheres for each link ...

# Motion planning parameters
motion_gen:
  collision_checker_type: "BLOX"      # Voxel-based checker
  trajopt_tsteps: 32                   # Trajectory optimization steps
  num_trajopt_seeds: 12                # Parallel seeds for optimization
  interpolation_dt: 0.03               # Interpolation timestep (s)
  collision_activation_distance: 0.025 # Collision margin (m)

# IK solver parameters
ik_solver:
  num_seeds: 20                        # Parallel IK seeds
  position_threshold: 0.005            # Position accuracy (m)
  rotation_threshold: 0.05             # Rotation accuracy (rad)
```

---

## Creating Configuration for Your Robot

Follow these steps to create a configuration for your own robot:

### Step 1: Prepare Your URDF

Requirements:
- Valid URDF with `<robot>` tag
- All joints have `<limit>` tags with effort, velocity
- End-effector frame defined (usually as a fixed joint)
- Collision meshes (optional but recommended)

**Example joint definition:**
```xml
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="3.0"/>
</joint>
```

### Step 2: Extract Joint Limits from URDF

You can use this Python script:

```python
#!/usr/bin/env python3
import xml.etree.ElementTree as ET

def extract_joint_limits(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = []
    for joint in root.findall('joint'):
        if joint.get('type') in ['revolute', 'prismatic']:
            name = joint.get('name')
            limit = joint.find('limit')
            if limit is not None:
                lower = float(limit.get('lower'))
                upper = float(limit.get('upper'))
                velocity = float(limit.get('velocity'))
                joints.append({
                    'name': name,
                    'limits': [lower, upper],
                    'velocity': velocity
                })

    # Print in YAML format
    print("joint_names:", [j['name'] for j in joints])
    print("position_limits:")
    for j in joints:
        print(f"  - {j['limits']}")
    print("velocity_limits:", [j['velocity'] for j in joints])

extract_joint_limits('path/to/your_robot.urdf')
```

### Step 3: Define Collision Spheres

Collision spheres approximate your robot's geometry for fast GPU collision checking.

**Guidelines:**
- Use 2-5 spheres per link
- Larger spheres for larger links
- Overlap spheres slightly for complete coverage
- Don't over-approximate (leads to false collisions)

**Tools to help:**
- Visualize in RViz with collision sphere markers
- Use `ros2 topic echo /curobo_gen_traj/collision_spheres`
- Iterate until coverage looks good

**Example approach:**
```yaml
collision_spheres:
  base_link:
    - center: [0.0, 0.0, 0.1]
      radius: 0.12
  link1:
    # Start of link
    - center: [0.0, 0.0, 0.05]
      radius: 0.08
    # Middle of link
    - center: [0.15, 0.0, 0.05]
      radius: 0.07
    # End of link
    - center: [0.3, 0.0, 0.05]
      radius: 0.08
```

**Tip**: Start with fewer, larger spheres. Refine later for better performance.

### Step 4: Set Acceleration/Jerk Limits

If your URDF doesn't specify these:

**Conservative defaults:**
```yaml
acceleration_limits: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # rad/s²
jerk_limits: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]    # rad/s³
```

**Aggressive (faster planning):**
```yaml
acceleration_limits: [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
jerk_limits: [500.0, 500.0, 500.0, 500.0, 500.0, 500.0]
```

**Too high**: Robot may not track trajectory accurately
**Too low**: Unnecessarily slow motion

### Step 5: Create the YAML File

```bash
# Create config file
cd ~/ros2_ws/src/curobo_ros/config
nano my_robot.yml
```

**Template:**
```yaml
robot:
  kinematics:
    urdf_path: "path/to/my_robot.urdf"
    base_link: "base_link"
    ee_link: "end_effector_link"

  cspace:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    position_limits:
      # Fill from URDF
      - [-3.14, 3.14]
      - [-2.0, 2.0]
      # ... etc
    velocity_limits: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    acceleration_limits: [15.0, 15.0, 15.0, 15.0, 15.0, 15.0]
    jerk_limits: [500.0, 500.0, 500.0, 500.0, 500.0, 500.0]

  collision_spheres:
    # Define spheres for each link
    link1:
      - center: [0.0, 0.0, 0.1]
        radius: 0.08
    # ... etc
```

### Step 6: Test Your Configuration

```bash
# Launch with your config
ros2 launch curobo_ros gen_traj.launch.py \
  robot_config_file:=/home/ros2_ws/src/curobo_ros/config/my_robot.yml
```

**Check logs for errors:**
- URDF parsing errors
- Missing joints
- Invalid limits

**Visual check in RViz:**
- Robot appears correctly
- Collision spheres cover robot geometry
- End-effector frame is at correct location

---

## Advanced: Using a Robot Package

For better organization, create a dedicated package:

### Create Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_curobo \
  --build-type ament_python \
  --dependencies rclpy

cd my_robot_curobo
mkdir -p config urdf meshes launch
```

### Add Files

- Put URDF in `urdf/`
- Put meshes in `meshes/`
- Put cuRobo config in `config/`
- Create launch file in `launch/`

### Update setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_curobo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='cuRobo configuration for My Robot',
    license='Apache-2.0',
)
```

### Build and Use

```bash
colcon build --packages-select my_robot_curobo
source install/setup.bash

ros2 launch curobo_ros gen_traj.launch.py \
  robot_config_file:=$(ros2 pkg prefix my_robot_curobo)/config/my_robot.yml
```

---

## Troubleshooting

### "Failed to parse URDF"

**Check:**
- URDF file path is correct (absolute or relative to config file)
- URDF is valid XML: `check_urdf my_robot.urdf`
- All mesh files exist if referenced

### "Joint limits exceed URDF limits"

**Solution**: Make sure your YAML joint limits are within URDF limits:
```yaml
# URDF: lower="-3.0" upper="3.0"
# YAML should be equal or more restrictive:
position_limits: [[-2.9, 2.9]]  # OK
position_limits: [[-3.5, 3.5]]  # ERROR - exceeds URDF
```

### Collision spheres don't cover robot

**Iterate**:
1. Launch node
2. View spheres: `ros2 topic echo /curobo_gen_traj/collision_spheres`
3. Visualize in RViz (MarkerArray display)
4. Adjust config file
5. Reload: `ros2 service call /curobo_gen_traj/update_motion_gen_config std_srvs/srv/Trigger`

### Planning is slow

**Try**:
- Reduce number of collision spheres
- Increase `voxel_size` parameter
- Reduce `trajopt_tsteps` in config
- Check GPU is being used: `nvidia-smi`

---

## Summary

**You've learned:**

- ✅ How to import a robot package (Doosan M1013 example)
- ✅ Understanding configuration file structure
- ✅ Creating configuration for your own robot
- ✅ Defining collision spheres
- ✅ Testing and troubleshooting

---

## Next Steps

- **[Managing Obstacles](3_adding_obstacles.md)** - Add obstacles to avoid collisions
- **[Dynamic Strategy Switching](4_dynamic_strategy_switching.md)** - Switch between robot control modes
- **[Parameters Guide](../concepts/parameters.md)** - Tune performance parameters

---

## Reference Files

**Example Configurations:**
- `config/m1013.yml` - Doosan M1013 (complete example)
- `config/ur5e.yml` - Universal Robots UR5e (if available)

**Useful Links:**
- cuRobo robot configuration: [curobo.org/robot_configuration.html](https://curobo.org/robot_configuration.html)
- URDF tutorials: [wiki.ros.org/urdf/Tutorials](http://wiki.ros.org/urdf/Tutorials)
- Lab-CORO robots: [github.com/Lab-CORO](https://github.com/Lab-CORO)

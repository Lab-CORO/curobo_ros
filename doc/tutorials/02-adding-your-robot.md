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

### Two Approaches for Robot Configuration

This tutorial covers **two methods** to generate the configuration YAML:

#### 🚀 Method 1: RViz Plugin (Recommended)
- **Uses**: curobo_robot_setup RViz2 plugin
- **Time**: ~15-30 minutes
- **Pros**: Interactive visual editor, drag-and-drop spheres, real-time feedback, no extra software
- **Cons**: None (RViz2 included with ROS 2)
- **Best for**: **Everyone** - fastest and easiest method

#### ✍️ Method 2: Manual
- **Uses**: Text editor + trial and error
- **Time**: ~2-4 hours
- **Pros**: No additional installation, good for understanding concepts
- **Cons**: Time-consuming, requires iteration, prone to errors
- **Best for**: Only if RViz is unavailable or for educational purposes

**Quick Decision:**
- Use Method 1 (RViz plugin) unless you have a specific reason not to!

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
ros2 launch curobo_ros unified_planner.launch.py \
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
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
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

You have **two approaches** to create a robot configuration:

1. **🚀 Automated (Recommended)**: Use cuRobo + Isaac Sim to generate collision spheres automatically
2. **✍️ Manual**: Create configuration file by hand with custom collision spheres

We'll cover both approaches below.

---

## Method 1: Using curobo_robot_setup RViz Plugin (Recommended)

This method uses the **curobo_robot_setup** RViz2 plugin to interactively create collision spheres and generate configurations directly in RViz. This is the **simplest and fastest** way to configure a new robot - no additional software required!

### Why This Method?

- ✅ **No extra software**: Works entirely in RViz2 (already included in ROS 2)
- ✅ **Interactive**: Visual feedback with draggable sphere markers
- ✅ **Fast iteration**: Adjust spheres in real-time and see results immediately
- ✅ **Integrated**: Part of your ROS 2 workspace, easy to version control
- ✅ **Lightweight**: No 30GB Isaac Sim download needed

### Prerequisites

- ROS 2 (Humble or later)
- RViz2 (included with ROS 2)
- Your robot URDF file

### Step 1: Install curobo_robot_setup Plugin

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone the curobo_robot_setup repository
git clone https://github.com/Lab-CORO/curobo_robot_setup.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup

# Source the workspace
source install/setup.bash
```

**Installation time**: ~2-3 minutes

### Step 2: Launch RViz2

```bash
# Launch RViz2
rviz2
```

### Step 3: Add the CuRobo Setup Panel

In RViz2:
1. Go to **Panels** menu (top bar)
2. Click **Add New Panel**
3. Select **CuRoboSetupPanel** from the list
4. Click **OK**

A new panel appears (usually on the right side) with three tabs.

### Step 4: Load Your Robot URDF (Tab 1)

In the **CuRoboSetupPanel**:

1. **Select Tab 1: "URDF Loading"**
2. Click **"Browse"** or **"Load URDF"** button
3. Navigate to your robot's URDF file
4. Click **"Open"**

**Result**: Your robot appears in the RViz 3D viewport!

**Visual Check**:
- ✅ Robot model loads correctly
- ✅ All links are visible
- ✅ Joints are connected properly

### Step 5: Define Collision Spheres (Tab 2)

This is where the magic happens! Interactively add collision spheres to your robot.

1. **Select Tab 2: "Collision Spheres"**

2. **For each link**:

   a. **Select link from dropdown**
      - Example: `link1`, `link2`, etc.

   b. **Click "Add Sphere"**
      - A sphere appears on the link
      - An interactive marker appears in RViz

   c. **Adjust sphere position**:
      - **Method 1 (Interactive)**: Drag the marker in RViz 3D view
      - **Method 2 (Precise)**: Enter coordinates in the panel
        - X: Forward/backward
        - Y: Left/right
        - Z: Up/down

   d. **Adjust sphere radius**:
      - Drag the radius handle, or
      - Enter value in the "Radius" field
      - Recommended: 0.05 - 0.15 meters depending on link size

   e. **Add more spheres**:
      - Click "Add Sphere" again for the same link
      - Repeat until link is well-covered
      - Guideline: 2-5 spheres per link

   f. **Visual verification**:
      - Spheres should overlap slightly (10-20%)
      - Coverage should extend along entire link
      - No large gaps between spheres

3. **Repeat for all links**

**Tips**:
- Start with larger spheres, refine later
- Use fewer spheres for better performance
- Make spheres slightly larger than geometry (safety margin)

### Step 6: Configure Robot Parameters (Tab 3)

1. **Select Tab 3: "Configuration"**

2. **Set kinematic parameters**:
   - **Base Link**: Enter name of robot base (e.g., `base_link`)
   - **End-Effector Link**: Enter name of EE (e.g., `tool0`, `ee_link`)

3. **Configure joint limits** (optional):
   - Adjust position, velocity, acceleration limits per joint
   - Default values are extracted from URDF

4. **Set retract position** (optional):
   - Default "safe" joint configuration
   - Used for warmup and reset

5. **Adjust optimization weights** (optional):
   - Default weights work well for most robots
   - Advanced users can tune for specific behaviors

### Step 7: Export Configuration

1. Click **"Export YAML"** button
2. Choose save location and filename
   - Example: `~/ros2_ws/src/my_robot_config/config/my_robot.yml`
3. Click **"Save"**

**Result**: A complete cuRobo configuration file is generated!

### Step 8: Verify Configuration File

Open the generated YAML file to verify:

```bash
cat ~/ros2_ws/src/my_robot_config/config/my_robot.yml
```

**Expected structure**:
```yaml
robot:
  kinematics:
    urdf_path: "path/to/your/robot.urdf"
    base_link: "base_link"
    ee_link: "tool0"
    collision_link_names: [link1, link2, link3, link4, link5, link6]

  cspace:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    position_limits:
      - [-3.14, 3.14]
      - [-2.0, 2.0]
      # ... extracted from URDF
    velocity_limits: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    acceleration_limits: [15.0, 15.0, 15.0, 15.0, 15.0, 15.0]
    jerk_limits: [500.0, 500.0, 500.0, 500.0, 500.0, 500.0]

  collision_spheres:
    link1:
      - center: [0.0, 0.0, 0.1]
        radius: 0.08
      - center: [0.0, 0.0, 0.2]
        radius: 0.07
    link2:
      - center: [0.15, 0.0, 0.0]
        radius: 0.06
    # ... more links
```

### Step 9: Test with curobo_ros

Now use your configuration with curobo_ros:

```bash
# Inside Docker container (or your ROS 2 environment)
ros2 launch curobo_ros unified_planner.launch.py \
  robot_config_file:=/path/to/my_robot.yml
```

**Success!** Your robot is configured and ready for motion planning.

---

### curobo_robot_setup Tips & Tricks

#### Efficient Sphere Placement

**For cylindrical links**:
- Place spheres along the cylinder axis
- Use 3-5 spheres depending on length
- Radius slightly larger than cylinder radius

**For box-like links**:
- Place spheres at corners and center
- 4-6 spheres for good coverage

**For complex shapes**:
- Use more smaller spheres
- Aim for 80-90% geometry coverage
- Don't worry about perfect coverage (some gaps OK)

#### Keyboard Shortcuts (in RViz)

- **Ctrl + Mouse**: Rotate view
- **Shift + Mouse**: Pan view
- **Scroll**: Zoom in/out
- **G**: Toggle grid
- **R**: Reset view

#### Troubleshooting curobo_robot_setup

**Panel doesn't appear**:
```bash
# Rebuild and re-source
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup --cmake-clean-cache
source install/setup.bash
rviz2
```

**URDF doesn't load**:
- Check URDF is valid: `check_urdf my_robot.urdf`
- Ensure mesh paths are correct (absolute or relative to URDF)
- Check RViz console for error messages

**Spheres not visible**:
- Check "Interactive Markers" display is enabled in RViz
- Verify correct TF frame is set
- Try zooming out (spheres might be very large/small)

**Cannot drag markers**:
- Click directly on the marker (not just near it)
- Ensure "Interact" mode is active (press `i` key)
- Check marker scale in display properties

---

## Method 2: Manual Configuration

Follow these steps to create a configuration manually (useful if you don't have Isaac Sim):

### Manual Step 1: Prepare Your URDF

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

### Manual Step 2: Extract Joint Limits from URDF

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

### Manual Step 3: Define Collision Spheres Manually

Collision spheres approximate your robot's geometry for fast GPU collision checking.

**Guidelines:**
- Use 2-5 spheres per link
- Larger spheres for larger links
- Overlap spheres slightly for complete coverage
- Don't over-approximate (leads to false collisions)

**Tools to help:**
- Visualize in RViz with collision sphere markers
- Use `ros2 topic echo /unified_planner/collision_spheres`
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

### Manual Step 4: Set Acceleration/Jerk Limits

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

### Manual Step 5: Create the YAML File

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

### Manual Step 6: Test Your Configuration

```bash
# Launch with your config
ros2 launch curobo_ros unified_planner.launch.py \
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

ros2 launch curobo_ros unified_planner.launch.py \
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
2. View spheres: `ros2 topic echo /unified_planner/collision_spheres`
3. Visualize in RViz (MarkerArray display)
4. Adjust config file
5. Reload: `ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger`

### Planning is slow

**Try**:
- Reduce number of collision spheres
- Increase `voxel_size` parameter
- Reduce `trajopt_tsteps` in config
- Check GPU is being used: `nvidia-smi`

---

## Comparison: RViz Plugin vs Manual Configuration

| Aspect | RViz Plugin (curobo_robot_setup) | Manual |
|--------|----------------------------------|--------|
| **Time** | 15-30 minutes | 2-4 hours |
| **Accuracy** | High (visual verification + draggable markers) | Medium (trial and error) |
| **Prerequisites** | RViz2 (included in ROS 2) | None (just text editor) |
| **Learning Curve** | Easy (familiar RViz interface) | Steep (needs understanding) |
| **Iteration** | Very fast (real-time adjustment) | Slow (edit, test, repeat) |
| **Coverage** | Excellent (interactive visual feedback) | Variable (depends on experience) |
| **Installation** | 2 minutes (one git clone + colcon build) | N/A |
| **Best For** | **All users** - simple, fast, integrated | Only if RViz unavailable |

**Recommendation**: Use the RViz plugin method. It's faster, easier, and provides better visual feedback than manual configuration.

---

## Summary

**You've learned:**

- ✅ How to import a robot package (Doosan M1013 example)
- ✅ Understanding configuration file structure
- ✅ **Method 1**: Interactive configuration with curobo_robot_setup RViz plugin
  - Installing the RViz plugin
  - Loading URDF in RViz
  - Adding and adjusting collision spheres interactively
  - Configuring robot parameters
  - Exporting complete configuration YAML
- ✅ **Method 2**: Manual configuration creation
  - Extracting joint limits from URDF
  - Manually defining collision spheres
  - Setting acceleration and jerk limits
- ✅ Creating a dedicated robot package
- ✅ Testing and troubleshooting

---

## Next Steps

- **[Managing Obstacles](03-collision-objects.md)** - Add obstacles to avoid collisions
- **[Dynamic Strategy Switching](04-strategy-switching.md)** - Switch between robot control modes
- **[Parameters Guide](../concepts/parameters.md)** - Tune performance parameters

---

## Reference Files

**Example Configurations:**
- `config/m1013.yml` - Doosan M1013 (complete example)
- `config/ur5e.yml` - Universal Robots UR5e (if available)

**Useful Links:**
- **curobo_robot_setup Plugin**: [github.com/Lab-CORO/curobo_robot_setup](https://github.com/Lab-CORO/curobo_robot_setup)
- **cuRobo Robot Configuration Tutorial**: [curobo.org/tutorials/1_robot_configuration.html](https://curobo.org/tutorials/1_robot_configuration.html)
- **cuRobo GitHub Repository**: [github.com/NVlabs/curobo](https://github.com/NVlabs/curobo)
- **URDF Tutorials**: [wiki.ros.org/urdf/Tutorials](http://wiki.ros.org/urdf/Tutorials)
- **Lab-CORO Robot Packages**: [github.com/Lab-CORO](https://github.com/Lab-CORO)

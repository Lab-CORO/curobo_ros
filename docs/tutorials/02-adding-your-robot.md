# Tutorial 2: Adding Your Robot

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

### Robot Configuration with curobo_robot_setup

The recommended way to generate the configuration YAML is the **curobo_robot_setup** RViz2 plugin — an interactive visual editor that lets you place and adjust collision spheres directly in RViz.

- **Time**: ~15–30 minutes
- **Prerequisites**: RViz2 (included with ROS 2) + your robot URDF on disk

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

1. **🚀 Automated (Recommended)**: Use Rviz2 to generate collision spheres automatically
2. **✍️ Manual**: Create configuration file by hand with custom collision spheres


---

## Using the curobo_robot_setup RViz Plugin

### Step 1: Install curobo_robot_setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/Lab-CORO/curobo_robot_setup.git
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup
source install/setup.bash
```

---

### Step 2: Get Your Robot URDF

The plugin requires a **URDF file on disk**. If your robot is defined via xacro or a ROS package, you first need to publish it on `/robot_description`, then save it to a file.

**Publish the robot:**

```bash
# Example — adapt to your package
ros2 launch my_robot_pkg display.launch.py
# or directly from a xacro file
ros2 run xacro xacro my_robot.urdf.xacro > /tmp/my_robot_raw.urdf
```

Once the robot is live on `/robot_description`, save the URDF using one of these two options:

#### Option A — Save manually

```bash
ros2 topic echo --once /robot_description std_msgs/msg/String \
  --field data > /path/to/my_robot.urdf
```
You may need to reformat the file.

#### Option B — urdf_extractor node (recommended)

The `urdf_extractor` node subscribes to `/robot_description`, formats the XML, and writes it to disk automatically:

```bash
ros2 run curobo_robot_setup urdf_extractor
```

Parameters (all optional, set via `--ros-args`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `topic` | `/robot_description` | Topic to subscribe to |
| `output_dir` | `.../curobo_robot_setup/example` | Directory for the output file |
| `robot_name` | (from `<robot name="...">`) | Override the output filename |

```bash
ros2 run curobo_robot_setup urdf_extractor \
  --ros-args -p output_dir:=/home/ros2_ws/src/my_robot/config \
             -p robot_name:=my_robot
```

The node saves the file as `<robot_name>.urdf` and exits once it has received the message.

---

### Step 3: Launch RViz2 and Add the Panel

Use the provided launch file — it starts RViz2 with the correct config, a `robot_state_publisher`, and a `joint_state_publisher_gui` in one command:

```bash
ros2 launch curobo_robot_setup robot_setup.launch.py \
  urdf_file:=/path/to/my_robot.urdf
```

| Argument | Default | Description |
|----------|---------|-------------|
| `urdf_file` | *(empty)* | Path to the URDF file to load |
| `use_gui` | `true` | Joint state publisher with sliders if true |
| `rviz_config` | package default | Path to a custom RViz config file |

RViz2 opens with the CuRoboSetupPanel already configured. If the panel is not visible, add it manually: **Panels** → **Add New Panel** → **CuRoboSetupPanel**.

The plugin automatically creates an **InteractiveMarkers** display pointing to `/collision_spheres` if one doesn't already exist. The URDF is loaded automatically in the panel — your robot appears in the 3D viewport immediately.

> If you need to load a different URDF after launch, use the **"Load URDF"** button in Tab 1 of the panel.

---

### Step 4: Define Collision Spheres

The plugin offers three approaches for placing collision spheres.

#### 4.1 Manual placement

For each link:
1. Select the link in the panel tree
2. Click **"Add Sphere"** — a sphere appears in the viewport with colored arrows (X/Y/Z)
3. **Drag the arrows** in RViz to reposition, or type coordinates directly in the panel
4. Adjust the **radius** field

Guidelines:
- 2–5 spheres per link depending on link size
- Spheres should overlap slightly (10–20%) for complete coverage
- Make spheres slightly larger than the actual geometry (safety margin)
- Fewer spheres = better planning performance

#### 4.2 Import spheres from an existing configuration

If your robot is an **assembly that includes a robot already configured for cuRobo** (e.g., a manipulator + gripper, where the arm already has a YAML), you can import existing spheres as a starting point.

Use **"Load Spheres from YAML"** and select the existing cuRobo YAML file. Spheres are placed on links **with matching names** between the two robots.

> **Warning**: this operation **replaces all current spheres**. Any manual adjustments made before this import are lost. Use it as a starting point and refine manually afterwards.

#### 4.3 Auto-generation (experimental)

> **This feature is experimental.** Results vary depending on mesh quality and complexity.

Auto-generation requires the `sphere_fit_service` node to be running separately. It wraps cuRobo's `fit_spheres_to_mesh` function:

```bash
# In a separate terminal
ros2 run curobo_robot_setup sphere_fit_service.py
```

Once the service is available, click **"Auto-generate spheres"** in the panel. The node fits spheres to the collision geometry of each link.

Always review and adjust the result manually — auto-generated spheres frequently need refinement on links with complex or non-convex geometry.

---

### Step 5: Configure Robot Parameters (Tab 3)

1. **Base Link**: robot base frame name (e.g. `base_link`, `base_0`)
2. **End-Effector Link**: EE frame name (e.g. `tool0`, `ee_link`)
3. Joint limits, retract position, and self-collision ignore pairs can be adjusted here — joint limits are pre-populated from the URDF.

---

### Step 6: Export the Configuration

1. Click **"Save YAML"**
2. Choose a path: e.g. `~/ros2_ws/src/my_robot_config/config/my_robot.yml`
3. Click **"Save"**

---

### Step 7: Test with curobo_ros

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  robot_config_file:=/path/to/my_robot.yml
```

---

### Step 8: Video Demonstration

{youtube} Comming soon


> Replace `YOUR_VIDEO_ID` with the YouTube video ID (the part after `?v=` in the URL).
> This requires the `sphinxcontrib-youtube` Sphinx extension.

---

### Troubleshooting curobo_robot_setup

**Panel doesn't appear**:
```bash
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup --cmake-clean-cache
source install/setup.bash
rviz2
```

**URDF doesn't load**:
- Validate the file: `check_urdf my_robot.urdf`
- Ensure mesh paths are correct (absolute or relative to the URDF file)

**Spheres not visible in RViz**:
- Check that the **InteractiveMarkers** display is enabled and pointed to `/collision_spheres`
- Verify the fixed frame matches your robot's base frame

**Auto-generation button does nothing**:
- Make sure `sphere_fit_service.py` is running (`ros2 service list | grep generate_spheres`)
- Check that `trimesh` and `curobo` are installed in the environment

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

## Summary

**You've learned:**

- ✅ How to import a robot package (Doosan M1013 example)
- ✅ Understanding configuration file structure
- ✅ Extracting a URDF from `/robot_description` (manually or via `urdf_extractor`)
- ✅ Creating collision spheres interactively with the curobo_robot_setup RViz plugin
  - Manual drag-and-drop placement
  - Importing spheres from an existing YAML (for assemblies)
  - Auto-generation (experimental)
- ✅ Configuring kinematic parameters and exporting the YAML
- ✅ Creating a dedicated robot package

---

## Next Steps

- **[Managing Obstacles](03-collision-objects.md)** - Add obstacles to avoid collisions
- **[Robot Execution](04-robot-execution.md)** - Connect to real robot or emulator
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

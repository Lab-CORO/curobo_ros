# Getting Started with **curobo_ros**

This guide shows the fastest route from cloning the repository to seeing a
GPU-accelerated trajectory in RViz.
Everything runs inside a ready-made Docker image, so your host OS stays clean.

---

## Choose Your Workflow

curobo_ros offers **two Docker workflows**:

| Workflow | Best For | Image Size | Setup |
|----------|----------|------------|-------|
| **DEV Mode** | Modifying curobo_ros internals | ~25-30 GB | Import dependencies with vcs |
| **PROD Mode** | Using curobo_ros in your projects | ~15-20 GB | Use pre-installed curobo_ros |

**Choose DEV if**: You want to contribute to or modify curobo_ros source code.  
**Choose PROD if**: You want to use curobo_ros as a dependency in your robot project.

**This guide covers DEV mode.** For PROD mode, see [Docker Workflow Guide](docker-workflow.md#workflow-prod-mode).

---

## Before You Begin

**New to ROS or Docker?** We recommend reading these guides first:
- [Introduction to curobo_ros](introduction.md) - Explains ROS, Docker, and cuRobo
- [Docker Workflow Guide](docker-workflow.md) - DEV vs PROD modes explained

---

## 1. Prerequisites

| Requirement | Notes |
|-------------|-------|
| **Ubuntu 20.04 / 22.04** (or Windows 11 + WSL 2) | Host system tested in CI. |
| **NVIDIA GPU** with recent drivers | CUDA 12 is used in the container. Check with `nvidia-smi`. |
| **~30 GB free disk space** | Docker image and dependencies are large! |
| **Docker ≥ 24** & **NVIDIA Container Toolkit** | Follow the install steps on [nvidia.github.io/nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). |

**Verify prerequisites:**
```bash
# Check NVIDIA driver
nvidia-smi

# Check Docker
docker --version

# Check NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

---

## 2. Clone the Repository (DEV Mode Only)

**Note**: Skip this step if using PROD mode (see [Docker Workflow Guide](docker-workflow.md)).

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone curobo_ros with submodules
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules
```

### Import Additional Dependencies (DEV Mode)

DEV mode requires additional packages mounted from the host:

```bash
cd ~/ros2_ws/src

# Install vcstool if not already installed
sudo apt install python3-vcstool

# Import dependencies from my.repos
vcs import < curobo_ros/my.repos
```

This imports:
- `curobo_msgs` - Custom message definitions
- `curobo_rviz` - RViz plugin for motion planning

**Your workspace structure**:
```
~/ros2_ws/
└── src/
    ├── curobo_ros/    # Cloned from GitHub
    ├── curobo_msgs/   # Imported by vcs
    └── curobo_rviz/   # Imported by vcs
```

These will be mounted as volumes in the container for live editing.

---

## 3. Build the Docker Image

**⚠️ This step takes 20-30 minutes and requires ~30 GB disk space!**

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh
```

**Interactive prompts**:

**Step 1/2: Choose GPU architecture**
```
1) Ampere (RTX 30XX series: 3060, 3070, 3080, 3090, A100)
2) Ada Lovelace (RTX 40XX series: 4060, 4070, 4080, 4090)
3) Turing (RTX 20XX series: 2060, 2070, 2080)
4) Volta (Titan V, V100)
```
Select the number matching your GPU.

**Step 2/2: Choose build mode**
```
1) DEV  - Development mode (for modifying curobo_ros internals)
2) PROD - Production mode (for using curobo_ros)
```
Select `1` for DEV mode (this guide).

**What happens during build:**
1. Downloads base Docker images (Ubuntu 22.04 + CUDA 12)
2. Installs ROS 2 Humble
3. Installs PyTorch with GPU support
4. Builds cuRobo from source
5. Installs all Python dependencies
6. Configures for your GPU architecture

**Result**: Image named `curobo_ros:ampere-dev` (or your GPU + mode).

---

## 4. Start the Container

**⚠️ Important**: Run this script to **create** the container. After creation, use `docker start` + `docker exec` (see [Docker Workflow Guide](docker-workflow.md)).

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash start_docker_x86.sh
```

**Interactive prompts**:

**Step 1/3: Choose GPU** - Select same GPU as build  
**Step 2/3: Choose mode** - Select `1` for DEV mode  
**Step 3/3: Confirm mount** - Press Enter to mount curobo packages from host

**What happens:**
1. Creates container named `curobo_ampere_dev` (or your GPU)
2. Mounts workspace directories as volumes:
   - `~/ros2_ws/src/curobo_ros` → `/home/ros2_ws/src/curobo_ros`
   - `~/ros2_ws/src/curobo_rviz` → `/home/ros2_ws/src/curobo_rviz`
   - `~/ros2_ws/src/curobo_msgs` → `/home/ros2_ws/src/curobo_msgs`
3. Enables GPU access (`--gpus all`)
4. Enables X11 forwarding for RViz (GUI)
5. Drops you into a shell **inside the container**

**You are now inside the container!** Notice your prompt changed.

### Open Additional Terminals

You'll often need multiple terminals (one for the node, one for calling services, etc.).

**Option 1: Using VSCode** (Recommended)
1. Install "Dev Containers" extension in VSCode
2. Attach VSCode to the running container
3. Open integrated terminals (Terminal → New Terminal)

**Option 2: Using docker exec**
```bash
# In a new terminal on your host
docker exec -it curobo_ampere_dev bash  # Use your actual container name
```

See the [Docker Workflow Guide](docker-workflow.md) for details.

---

## 5. Build the Workspace

**All commands in this section run INSIDE the Docker container.**

```bash
cd /home/ros2_ws

# Build all packages
colcon build --symlink-install

# Symlink-install allows Python changes without rebuilding
```

**Build time**: ~2-5 minutes on first build.

**Troubleshooting**: If you get errors, see [troubleshooting.md](troubleshooting.md#2-missing-symbol-ucm_set_global_opts).

---

## 6. Source the ROS Environment

**Important**: You must source the workspace in **every new terminal** before using ROS commands.

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source your workspace
source /home/ros2_ws/install/setup.bash
```

**Pro Tip**: Add this to your `~/.bashrc` inside the container to auto-source on every terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Verify it worked:**
```bash
# This should list all ROS packages, including curobo_ros
ros2 pkg list | grep curobo
```

Expected output:
```
curobo_msgs
curobo_ros
curobo_rviz
```

---

## 7. Launch the System

Now let's start the motion planning system!

```bash
# Make sure you sourced the workspace (step 6)
ros2 launch curobo_ros unified_planner.launch.py
```

**What launches:**
1. `robot_state_publisher` - Publishes robot URDF
2. `joint_state_publisher` - Publishes joint states
3. `unified_planner` - Main motion planning node
4. `marker_publisher` - Publishes visualization markers
5. `rviz2` - RViz visualization window

**First launch**: The node performs a **warmup** (compiling GPU kernels). This takes ~30-60 seconds. You'll see:

```
[INFO] [unified_planner]: Starting motion generation warmup...
[INFO] [unified_planner]: Warmup progress: 10%
[INFO] [unified_planner]: Warmup progress: 20%
...
[INFO] [unified_planner]: ✅ Warmup complete! Ready to generate trajectories.
```

**RViz should open** showing your robot (default: Doosan M1013).

**⚠️ RViz doesn't appear?** See [troubleshooting.md](troubleshooting.md#32-rviz--gui-windows-do-not-appear).

---

## 8. Verify the System is Working

Let's run a simple test to make sure everything works.

### Option 1: Run the Test Script (Easiest)

```bash
# In another terminal (docker exec or VSCode)
cd /home/ros2_ws/src/curobo_ros
python3 tests/test_basic.py
```

This script checks if the node is running and ready.

### Option 2: Manual Verification

```bash
# Open a new terminal (docker exec -it x86docker bash)
# Source the environment
source /home/ros2_ws/install/setup.bash

# Check if the node is running
ros2 node list
# Should show: /unified_planner

# Check if services are available
ros2 service list | grep unified_planner
# Should show multiple services including /unified_planner/generate_trajectory

# Call the availability service
ros2 service call /unified_planner/is_available std_srvs/srv/Trigger
```

**Expected response:**
```yaml
success: True
message: 'Node is ready'
```

If you see this, **everything is working!** 🎉

---


## 9. Choose Your Planner
 
curobo_ros now supports **multiple planning strategies** through the Unified Planner architecture:
 
| Planner | Mode | Best For |
|---------|------|----------|
| **Classic** | Open-loop | Static environments, pre-computed paths |
| **MPC** | Closed-loop | Dynamic obstacles, real-time adaptation |
| **Batch** | Open-loop | Multiple trajectory alternatives |
| **Constrained** | Open-loop | Custom constraints (orientation, velocity) |
 
**Launch with default (Classic) planner:**
```bash
ros2 run curobo_ros unified_planner
```
 
**Switch to MPC planner:**
```bash
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
```
 
See [Unified Planner Concepts](../concepts/unified-planner.md) for details.
 
---
 
## 10. Generate Your First Trajectory
 
**Using ROS 2 service:**
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{
    goal_pose: {
      position: {x: 0.5, y: 0.3, z: 0.4},
      orientation: {w: 1.0}
    },
    use_current_state: true
  }"
```
 
**Execute the trajectory:**
```bash
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```
 
---
 
## 11. Visualize in RViz
 
Launch RViz to see your trajectory:
```bash
ros2 launch curobo_ros rviz.launch.py
```
 
You should see:
- **Robot model** in current position
- **Ghost robot** showing planned trajectory
- **Collision spheres** for obstacle avoidance
- **Voxel grid** for environment representation
 
---
 
## 12. Next Steps
 
### Tutorials – Hands-on Guides
 
**Getting Started:**
- [First Trajectory](../tutorials/01-first-trajectory.md) - Detailed first trajectory walkthrough
- [Adding Your Robot](../tutorials/02-adding-your-robot.md) - Integrate custom robots

**Working with Obstacles:**
- [Collision Objects](../tutorials/03-collision-objects.md) - Static obstacle management
- [Point Cloud Detection](../tutorials/07-pointcloud-detection.md) - Camera-based obstacles

**Advanced Features:**
- [Strategy Switching](../tutorials/04-strategy-switching.md) - Switch between robot modes
- [MPC Planner](../tutorials/05-mpc-planner.md) - Real-time reactive planning
- [IK/FK Services](../tutorials/06-ik-fk-services.md) - Kinematics services
 
### Concepts – Deep Dives

- **[Unified Planner](../concepts/unified-planner.md)** - Multiple planning strategies
- **[Architecture](../concepts/architecture.md)** - System design patterns
- **[ROS Interfaces](../concepts/ros-interfaces.md)** - Services, topics, actions
- **[RViz Plugin](../concepts/rviz-plugin.md)** - Interactive visualization
 
### Troubleshooting
 
- [**Troubleshooting Guide**](troubleshooting.md) - Common CUDA/Docker/X-server issues
 
---
 




 
## Quick Reference
 
### Planner Selection
 
```bash
# List available planners
ros2 service call /unified_planner/list_planners std_srvs/srv/Trigger
 
# Switch planners
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"  # Classic
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"  # MPC
```
 
### Robot Strategy Selection
 
```bash
# Change robot control strategy
ros2 param set /unified_planner robot_type "emulator"  # or "doosan_m1013", "ghost"
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger
```
 
### Common Commands
 
```bash
# Generate trajectory
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "{...}"
 
# Execute trajectory
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
 
# Add collision object
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject "{...}"
 
# Remove all objects
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger
```

**Happy motion planning!** 🤖









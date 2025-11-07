# Getting Started with **curobo_ros**

This guide shows the fastest route from cloning the repository to seeing a
GPU-accelerated trajectory in RViz.
Everything runs inside a ready-made Docker image, so your host OS stays clean.

---

## Before You Begin

**New to ROS or Docker?** We recommend reading these guides first:
- [Introduction to curobo_ros](concepts/introduction.md) - Explains ROS, Docker, and cuRobo
- [Docker Workflow Guide](concepts/docker_workflow.md) - Learn to work efficiently with Docker

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

## 2. Clone the Repository

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone curobo_ros with submodules
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules
```

### Import Additional Dependencies

To set up the development environment, you need additional ROS packages:

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
- Other required packages

**Note**: This package is designed for development, so all packages are included in the Docker container through volume sharing (your changes on the host are immediately visible inside the container).

---

## 3. Build the Docker Image

**⚠️ This step takes 20-30 minutes and requires ~30 GB disk space!**

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh x86
```

**What happens during build:**
1. Downloads base Docker images (Ubuntu 22.04 + CUDA 12)
2. Installs ROS 2 Humble
3. Installs PyTorch with CUDA support
4. Builds cuRobo from source
5. Installs all Python dependencies

**GPU Selection**: The script will ask you to choose your GPU:
```
Choose your GPU card model:
1) RTX 30XX
2) RTX 40XX
3) A100
```

This optimizes CUDA compilation for your specific GPU architecture.

**Result**: An image named `curobo_docker:rtx30xxx` (or `rtx40xxx` or `A100`) depending on your choice.

---

## 4. Start the Container

**⚠️ Important**: Only run this script **once** to create the container. After that, use `docker start x86docker` (see [Docker Workflow Guide](concepts/docker_workflow.md)).

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash start_docker_x86.sh
```

**What happens:**
1. Creates a container named `x86docker`
2. Mounts your workspace directories as volumes:
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
docker exec -it x86docker bash
```

See the [Docker Workflow Guide](concepts/docker_workflow.md) for details.

---

## 5. Build the ROS Workspace

The workspace is **already compiled** in the Docker image, but you should rebuild if you make changes:

```bash
# Inside the container
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
ros2 launch curobo_ros gen_traj.launch.py
```

**What launches:**
1. `robot_state_publisher` - Publishes robot URDF
2. `joint_state_publisher` - Publishes joint states
3. `curobo_gen_traj` - Main motion planning node
4. `marker_publisher` - Publishes visualization markers
5. `rviz2` - RViz visualization window

**First launch**: The node performs a **warmup** (compiling GPU kernels). This takes ~30-60 seconds. You'll see:

```
[INFO] [curobo_gen_traj]: Starting motion generation warmup...
[INFO] [curobo_gen_traj]: Warmup progress: 10%
[INFO] [curobo_gen_traj]: Warmup progress: 20%
...
[INFO] [curobo_gen_traj]: ✅ Warmup complete! Ready to generate trajectories.
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
# Should show: /curobo_gen_traj

# Check if services are available
ros2 service list | grep curobo_gen_traj
# Should show multiple services including /curobo_gen_traj/generate_trajectory

# Call the availability service
ros2 service call /curobo_gen_traj/is_available std_srvs/srv/Trigger
```

**Expected response:**
```yaml
success: True
message: 'Node is ready'
```

If you see this, **everything is working!** 🎉

---

## 9. Generate Your First Trajectory

Now let's plan a trajectory to a target position!

```bash
# In the second terminal (make sure workspace is sourced)
ros2 service call /curobo_gen_traj/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"
```

**What happens:**
1. cuRobo solves IK to find joint angles for the target pose
2. Plans a smooth, collision-free trajectory
3. Publishes the trajectory to `/trajectory` topic
4. **RViz shows a preview** of the robot following the path

**In RViz**, you should see:
- The main robot (current position)
- A "ghost" robot (preview) showing the trajectory

**Response:**
```yaml
success: True
message: "Trajectory generated successfully"
```

**Trajectory not generated?**
- The target might be unreachable (out of workspace)
- Try a different position
- Check the logs in terminal 1 for error messages

---

## 10. Next Steps

**Congratulations!** You've successfully:
- ✅ Set up the Docker environment
- ✅ Built and launched curobo_ros
- ✅ Generated your first trajectory

**What's next?**

### Essential Tutorials
1. **[Your First Trajectory](tutorials/1_first_trajectory.md)** - Learn more about trajectory generation
2. **[Adding Your Robot](tutorials/2_adding_your_robot.md)** - Integrate your own robot (Doosan M1013 example)
3. **[Managing Obstacles](tutorials/3_adding_obstacles.md)** - Add collision objects to the environment

### Advanced Tutorials
4. **[Dynamic Strategy Switching](tutorials/4_dynamic_strategy_switching.md)** - Switch between real robot, emulator, and simulation
5. **[Camera Integration](tutorials/5_camera_pointcloud.md)** - Use depth cameras for obstacle detection
6. **[IK/FK Services](tutorials/6_ik_fk_services.md)** - Use inverse and forward kinematics

### Concepts
- **[Parameters Guide](concepts/parameters.md)** - Understand `voxel_size`, `time_dilation_factor`, and other important parameters
- **[ROS Interfaces](concepts/ros_interfaces.md)** - Complete reference of all services, topics, and actions
- **[Architecture](concepts/architecture.md)** - How curobo_ros is structured internally

### Reference
- **[Troubleshooting](troubleshooting.md)** - Solutions to common problems
- **[Doosan Example](tutorials/doosan_example.md)** - Complete example with Doosan M1013

---

**Happy motion planning!** 🤖

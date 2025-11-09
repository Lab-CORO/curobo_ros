# Docker Workflow Guide

This guide teaches you how to work efficiently with Docker containers for curobo_ros. If you're new to Docker, don't worry - we'll cover everything step by step!

---

## Two Workflows: DEV vs PROD

curobo_ros offers two Docker workflows to match your needs:

| Aspect | DEV Mode | PROD Mode |
|--------|----------|-----------|
| **Purpose** | Develop and modify curobo_ros internals | Use curobo_ros in your projects |
| **Image Size** | ~25-30 GB | ~15-20 GB (smaller) |
| **curobo_ros** | Mounted from host (live editing) | Pre-installed in container |
| **Your Workspace** | Shared from host ros2_ws | Mount your own workspace |
| **Use Case** | Contributing to curobo_ros | Using curobo_ros as a dependency |
| **Build Time** | ~25-30 minutes | ~20-25 minutes |

**Choose DEV if**:
- You want to modify curobo_ros source code
- You're contributing features or fixes
- You need development tools (debugging, profiling)

**Choose PROD if**:
- You want to use curobo_ros in your robot project
- You don't need to modify curobo_ros internals
- You want a smaller, optimized image

---

## Why Docker?

The curobo_ros project requires many dependencies:
- ROS 2 Humble
- CUDA 12 + NVIDIA drivers
- PyTorch with GPU support
- cuRobo library
- Python packages

Installing all of this manually would take hours and often fails. Docker provides a **pre-configured environment** where everything works out of the box.

**Important**: Docker images are **large (15-30 GB)**, so make sure you have enough disk space!

---

## Docker Concepts You Need to Know

| Concept | What It Means | Example |
|---------|---------------|---------|
| **Image** | A snapshot of a configured environment | `curobo_ros:ampere-dev` |
| **Container** | A running instance of an image | `curobo_ampere_dev` |
| **Volume** | A folder shared between host and container | Your workspace mounted in container |
| **Build** | Creating an image from a Dockerfile | `bash build_docker.sh` |
| **Start** | Running a container | `bash start_docker_x86.sh` |
| **Exec** | Running a command in an existing container | `docker exec -it container_name bash` |

---

## Workflow: DEV Mode

Use this workflow if you want to **modify curobo_ros** source code.

### Prerequisites

```bash
# Create ROS workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone curobo_ros
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules

# Import dependencies using vcs
sudo apt install python3-vcstool
vcs import < curobo_ros/my.repos
```

This imports `curobo_msgs` and `curobo_rviz` into your workspace.

### 1. Build DEV Image

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh
```

**Interactive prompts**:
1. **GPU**: Choose your GPU architecture (Ampere, Ada Lovelace, etc.)
2. **Mode**: Choose `1` for DEV mode

**Build time**: ~25-30 minutes

**Result**: Image `curobo_ros:ampere-dev` (or your GPU architecture)

### 2. Start DEV Container

```bash
bash start_docker_x86.sh
```

**Interactive prompts**:
1. **GPU**: Choose same GPU as build
2. **Mode**: Choose `1` for DEV mode

**What gets mounted**:
- `~/ros2_ws/src/curobo_ros` → `/home/ros2_ws/src/curobo_ros`
- `~/ros2_ws/src/curobo_rviz` → `/home/ros2_ws/src/curobo_rviz`
- `~/ros2_ws/src/curobo_msgs` → `/home/ros2_ws/src/curobo_msgs`

**Inside the container**:
```bash
# Build workspace
cd /home/ros2_ws
colcon build --symlink-install

# Source environment
source install/setup.bash

# Launch
ros2 launch curobo_ros gen_traj.launch.py
```

### 3. Edit Code on Host

**Key Advantage**: Edit files on your host machine with your favorite IDE. Changes are immediately visible in the container!

**Example workflow**:
1. Edit `~/ros2_ws/src/curobo_ros/curobo_ros/trajectory_generator.py` on host
2. Inside container: `colcon build --symlink-install --packages-select curobo_ros`
3. Test your changes immediately

---

## Workflow: PROD Mode

Use this workflow if you want to **use curobo_ros** in your projects without modifying it.

### Prerequisites

```bash
# Create your own ROS workspace
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src

# Add your packages here
# curobo_ros is already installed in the container!
```

### 1. Build PROD Image

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh
```

**Interactive prompts**:
1. **GPU**: Choose your GPU architecture
2. **Mode**: Choose `2` for PROD mode

**Build time**: ~20-25 minutes

**Result**: Smaller image `curobo_ros:ampere-prod` with curobo_ros pre-installed

### 2. Start PROD Container

```bash
bash start_docker_x86.sh
```

**Interactive prompts**:
1. **GPU**: Choose same GPU as build
2. **Mode**: Choose `2` for PROD mode
3. **Workspace path**: Enter your workspace path (e.g., `/home/username/my_robot_ws`)

**What gets mounted**:
- Your entire workspace → `/home/ros2_ws`

**Inside the container**:
```bash
# curobo_ros is pre-installed in /home/curobo_ws and auto-sourced!
ros2 pkg list | grep curobo  # Should show curobo packages

# Build your own packages
cd /home/ros2_ws
colcon build

# Source your workspace (curobo_ws is already sourced automatically)
source install/setup.bash

# Use curobo_ros in your code
ros2 launch curobo_ros gen_traj.launch.py
```

**Note**: curobo_ros is installed in `/home/curobo_ws` (separate from your workspace) and is automatically sourced on container startup via `.bashrc`.

### 3. Use curobo_ros in Your Package

**Example `package.xml`**:
```xml
<depend>curobo_msgs</depend>
<depend>curobo_ros</depend>
```

**Example Python node**:
```python
from curobo_msgs.srv import TrajectoryGeneration

# Use curobo_ros services...
```

---

## Daily Workflow with VSCode

The best way to work with Docker containers is using **Visual Studio Code**.

### Setup (One Time)

1. **Install VSCode**: [code.visualstudio.com](https://code.visualstudio.com/)
2. **Install Dev Containers extension**: Search for "Dev Containers" by Microsoft
3. **Install Docker extension** (optional): For container management UI

### Attach to Running Container

**Step 1**: Start container in terminal
```bash
# Check if running
docker ps

# Start if stopped (use your actual container name)
docker start curobo_ampere_dev  # or curobo_ampere_prod

# Optional: attach in terminal
docker exec -it curobo_ampere_dev bash
```

**Step 2**: Attach VSCode
1. Open VSCode
2. Click blue/green icon in bottom-left corner (><)
3. Select "Attach to Running Container..."
4. Choose your container (e.g., `curobo_ampere_dev`)

**You're in!** Your terminal is now inside the container. Edit files, run commands, open multiple terminals - all inside the container.

### Working in VSCode

```bash
# All terminals are inside the container automatically!

# Build workspace
cd /home/ros2_ws
colcon build --symlink-install

# Source environment
source install/setup.bash

# Launch
ros2 launch curobo_ros gen_traj.launch.py
```

**Pro Tip**: Open multiple terminals (Terminal → New Terminal) - they all run inside the container!

---

## Opening Additional Terminals

### Method 1: VSCode (Easiest)

If using VSCode attached to container:
1. Terminal → New Terminal
2. Done! Already inside container.

### Method 2: Docker Exec

Open new terminal on **host** and run:
```bash
docker exec -it curobo_ampere_dev bash  # Replace with your container name

# Inside container, source environment
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
```

---

## Container Lifecycle

### Starting and Stopping

```bash
# Check running containers
docker ps

# Check all containers (including stopped)
docker ps -a

# Start a stopped container
docker start curobo_ampere_dev

# Stop a running container
docker stop curobo_ampere_dev

# Restart
docker restart curobo_ampere_dev
```

### Important Notes

**What persists**:
- ✅ Files in mounted volumes (your code)
- ✅ Built packages in mounted workspace

**What doesn't persist**:
- ❌ Packages installed with `apt` inside container (unless you rebuild image)
- ❌ Files outside mounted volumes

### When to Rebuild

Rebuild the image when:
- cuRobo dependencies change
- ROS 2 packages need updates
- You want to switch GPU architecture
- You want to switch DEV/PROD mode

You do **NOT** need to rebuild for:
- Modifying your code (it's in mounted volume)
- Building ROS packages (builds are in mounted volume)

---

## Common Docker Commands

### Container Management

```bash
# List running containers
docker ps

# List all containers
docker ps -a

# Start container
docker start <container_name>

# Stop container
docker stop <container_name>

# Remove container
docker rm <container_name>

# View logs
docker logs <container_name>

# Resource usage
docker stats <container_name>
```

### Image Management

```bash
# List images
docker images

# Remove image (must remove containers first)
docker rmi curobo_ros:ampere-dev

# Check disk usage
docker system df

# Clean up unused resources
docker system prune
```

### GPU Access

```bash
# Check GPU inside container
docker exec -it <container_name> nvidia-smi

# Should show your GPU and CUDA version
```

---

## Comparing DEV vs PROD Workflows

### DEV Mode Example Session

```bash
# Build DEV image (once)
bash build_docker.sh  # Choose GPU, then DEV mode

# Start DEV container
bash start_docker_x86.sh  # Choose GPU, then DEV mode

# Inside container: Edit code on host, build in container
cd /home/ros2_ws
colcon build --symlink-install --packages-select curobo_ros
source install/setup.bash
ros2 launch curobo_ros gen_traj.launch.py

# Edit files on host machine
# vim ~/ros2_ws/src/curobo_ros/curobo_ros/my_file.py

# Rebuild changed package
colcon build --symlink-install --packages-select curobo_ros
```

### PROD Mode Example Session

```bash
# Build PROD image (once)
bash build_docker.sh  # Choose GPU, then PROD mode

# Create your workspace on host
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src
# Add your packages here

# Start PROD container
bash start_docker_x86.sh  # Choose GPU, PROD mode, enter workspace path

# Inside container: curobo_ros is already installed!
cd /home/ros2_ws
colcon build  # Build YOUR packages
source install/setup.bash

# Use curobo_ros
ros2 launch curobo_ros gen_traj.launch.py
```

---

## Troubleshooting

### Container Name Already Exists

```bash
# Option 1: Start existing
docker start <container_name>
docker exec -it <container_name> bash

# Option 2: Remove and recreate
docker stop <container_name>
docker rm <container_name>
bash start_docker_x86.sh
```

### GPU Not Accessible

```bash
# Check NVIDIA Container Toolkit installed
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi

# If fails, install toolkit (see troubleshooting.md)
```

### Out of Disk Space

```bash
# Check Docker space
docker system df

# Clean up
docker system prune -a --volumes  # ⚠️ Removes unused images/containers
```

See full troubleshooting guide: [troubleshooting.md](../troubleshooting.md)

---

## Best Practices

### DO ✅

- **Use VSCode Dev Containers** for best experience
- **DEV mode**: Edit code on host, build in container
- **PROD mode**: Keep your code in your workspace, use curobo_ros as dependency
- **Source workspace** in every new terminal
- **Stop containers** when not using: `docker stop <container_name>`
- **Clean up regularly**: `docker system prune`

### DON'T ❌

- **Don't mix modes** - stick with DEV or PROD for a project
- **Don't run start script multiple times** - use `docker start` + `docker exec`
- **Don't install with apt inside container** (won't persist unless you rebuild image)
- **Don't store important data outside mounted volumes**
- **Don't forget to source**: `source install/setup.bash`

---

## Summary

### Quick Reference

| Task | DEV Mode | PROD Mode |
|------|----------|-----------|
| **Build image** | `bash build_docker.sh` → GPU → DEV | `bash build_docker.sh` → GPU → PROD |
| **Start container** | `bash start_docker_x86.sh` → GPU → DEV | `bash start_docker_x86.sh` → GPU → PROD → path |
| **Edit curobo_ros** | Edit on host, rebuild in container | Not recommended (pre-installed) |
| **Your packages** | Add to ~/ros2_ws/src | Add to your workspace |
| **Rebuild** | `colcon build` in container | `colcon build` in container |

### Key Differences

- **DEV**: curobo_ros source mounted from host → live editing
- **PROD**: curobo_ros pre-installed → smaller image, use as dependency

---

## Next Steps

- **[Getting Started](../getting_started.md)** - Complete setup walkthrough
- **[Your First Trajectory](../tutorials/1_first_trajectory.md)** - Try the system
- **[Adding Your Robot](../tutorials/2_adding_your_robot.md)** - Integrate your robot
- **[Troubleshooting](../troubleshooting.md)** - Common issues

---

## Additional Resources

- **Docker**: [docs.docker.com](https://docs.docker.com/)
- **VSCode Dev Containers**: [code.visualstudio.com/docs/devcontainers/containers](https://code.visualstudio.com/docs/devcontainers/containers)
- **NVIDIA Container Toolkit**: [github.com/NVIDIA/nvidia-docker](https://github.com/NVIDIA/nvidia-docker)

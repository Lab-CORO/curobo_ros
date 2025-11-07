# Docker Workflow Guide

This guide teaches you how to work efficiently with Docker containers for curobo_ros development. If you're new to Docker, don't worry - we'll cover everything step by step!

---

## Why Docker?

The curobo_ros project requires many dependencies:
- ROS 2 Humble
- CUDA 12 + NVIDIA drivers
- PyTorch with GPU support
- cuRobo library
- Python packages

Installing all of this manually would take hours and often fails. Docker provides a **pre-configured environment** where everything works out of the box.

**Important**: The Docker image is **large (~30 GB)**, so make sure you have enough disk space!

---

## Docker Concepts You Need to Know

| Concept | What It Means | Example |
|---------|---------------|---------|
| **Image** | A snapshot of a configured environment | `curobo_docker:rtx30xxx` |
| **Container** | A running instance of an image | `x86docker` |
| **Volume** | A folder shared between host and container | Your code folder mounted in container |
| **Build** | Creating an image from a Dockerfile | `bash build_docker.sh x86` |
| **Start** | Running a container | `bash start_docker_x86.sh` |
| **Exec** | Running a command in an existing container | `docker exec -it x86docker bash` |

---

## The Recommended Workflow

Here's how to work efficiently with Docker:

### 1. Build the Image (Once)

You only need to do this **once** (or when dependencies change):

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh x86
```

This will:
- Download base images
- Install ROS 2, CUDA, PyTorch
- Build cuRobo from source
- **Take 20-30 minutes**

The script will ask you to choose your GPU model:
```
Choose your GPU card model:
1) RTX 30XX
2) RTX 40XX
3) A100
```

Pick the one that matches your GPU (check with `nvidia-smi`).

### 2. Start the Container (First Time)

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash start_docker_x86.sh
```

This:
- Creates and starts a container named `x86docker`
- Mounts your workspace as volumes (your code is shared!)
- Enables GPU access
- Enables X11 forwarding (for RViz/GUIs)
- Drops you into a shell **inside the container**

**Important**: After running this script once, **DO NOT run it again** while the container exists! Use the workflow below instead.

---

## Daily Workflow: Using VSCode

The best way to work with Docker containers is using **Visual Studio Code** with the **Dev Containers** extension.

### Setup (One Time)

1. **Install VSCode**: Download from [code.visualstudio.com](https://code.visualstudio.com/)

2. **Install the Dev Containers extension**:
   - Open VSCode
   - Go to Extensions (Ctrl+Shift+X)
   - Search for "Dev Containers" (by Microsoft)
   - Click Install

3. **Alternative**: Install "Docker" extension by Microsoft for container management UI

### Daily Usage

#### Option A: Attach to Running Container (Recommended)

**Step 1**: Start the container in a terminal (if not already running)
```bash
# Check if container is running
docker ps

# If not running, start it
docker start x86docker

# Optional: attach to it in terminal
docker exec -it x86docker bash
```

**Step 2**: Attach VSCode to the container
1. Open VSCode
2. Click the blue/green icon in the bottom-left corner (><)
3. Select "Attach to Running Container..."
4. Choose `x86docker`

**That's it!** VSCode is now running inside the container. Your integrated terminal is inside the container, and you can edit files directly.

#### Option B: Use Remote-Containers

1. Open the workspace folder in VSCode
2. VSCode may prompt "Reopen in Container" - click it
3. Or manually: Press F1 → "Dev Containers: Reopen in Container"

### Working in VSCode

Once attached:

```bash
# Your terminal is already inside the container!
# No need for docker exec

# Build your workspace
cd /home/ros2_ws
colcon build --symlink-install

# Source the environment
source install/setup.bash

# Launch the system
ros2 launch curobo_ros gen_traj.launch.py
```

**Pro Tip**: Open multiple terminals in VSCode (Terminal → New Terminal) - they all run inside the container!

---

## Opening Additional Terminals

Sometimes you need multiple terminals (one for the node, one for calling services, etc.).

### Method 1: VSCode Integrated Terminal (Easiest)

If you're using VSCode attached to the container:
1. Click "Terminal" → "New Terminal"
2. Done! The new terminal is already inside the container.

### Method 2: Docker Exec (Without VSCode)

Open a new terminal on your **host machine** and run:

```bash
docker exec -it x86docker bash
```

This opens a new shell **inside the running container**.

**Example workflow**:
```bash
# Terminal 1 (start container and launch node)
cd ~/ros2_ws/src/curobo_ros/docker
bash start_docker_x86.sh
# You're now inside the container
cd /home/ros2_ws
source install/setup.bash
ros2 launch curobo_ros gen_traj.launch.py

# Terminal 2 (new terminal window on host)
docker exec -it x86docker bash
# You're now inside the container in a second shell
cd /home/ros2_ws
source install/setup.bash
ros2 service call /curobo_gen_traj/generate_trajectory ...
```

---

## Container Lifecycle

### Starting and Stopping

```bash
# Check if container is running
docker ps

# Check all containers (including stopped)
docker ps -a

# Start a stopped container
docker start x86docker

# Stop a running container
docker stop x86docker

# Restart a container
docker restart x86docker
```

### Important Notes

- **Your code is safe**: Files in mounted volumes (`/home/ros2_ws/src`) persist even if you stop the container
- **Installed packages**: If you install something with `apt` inside the container, it **will be lost** when you remove the container
- **Built packages**: Your ROS workspace builds persist (in mounted volume)

### When to Rebuild the Container

You need to **rebuild the image** if:
- cuRobo dependencies change
- ROS 2 packages need updates
- System-level packages need updates

You do **NOT** need to rebuild if:
- You modify your code (it's in a shared volume)
- You build ROS packages (it's in a shared volume)

---

## Common Docker Commands

### Container Management

```bash
# List running containers
docker ps

# List all containers
docker ps -a

# Start a container
docker start x86docker

# Stop a container
docker stop x86docker

# Remove a container (loses non-volume data!)
docker rm x86docker

# View container logs
docker logs x86docker

# View resource usage
docker stats x86docker
```

### Image Management

```bash
# List all images
docker images

# Remove an image (must remove containers first)
docker rmi curobo_docker:rtx30xxx

# Check disk usage
docker system df

# Clean up unused images/containers/volumes
docker system prune
```

### Debugging

```bash
# Check if container can access GPU
docker exec -it x86docker nvidia-smi

# Check ROS environment
docker exec -it x86docker bash -c "source /home/ros2_ws/install/setup.bash && ros2 node list"

# View container details
docker inspect x86docker
```

---

## File Sharing Between Host and Container

Your workspace is **automatically shared** via Docker volumes:

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `~/ros2_ws/src/curobo_ros` | `/home/ros2_ws/src/curobo_ros` | Main package |
| `~/ros2_ws/src/curobo_rviz` | `/home/ros2_ws/src/curobo_rviz` | RViz plugin |
| `~/ros2_ws/src/curobo_msgs` | `/home/ros2_ws/src/curobo_msgs` | Message definitions |

**This means**:
- Edit files on your host (with your favorite editor)
- Changes are **immediately visible** inside the container
- Build inside the container
- Built files are visible on the host

---

## Troubleshooting Docker Issues

### "Container already exists"

**Error**: `docker: Error response from daemon: Conflict. The container name "/x86docker" is already in use`

**Solution**:
```bash
# Option 1: Start the existing container instead
docker start x86docker
docker exec -it x86docker bash

# Option 2: Remove the old container (careful!)
docker stop x86docker
docker rm x86docker
# Now run start_docker_x86.sh again
```

### "Cannot connect to Docker daemon"

**Error**: `Cannot connect to the Docker daemon at unix:///var/run/docker.sock`

**Solution**:
```bash
# Start Docker service
sudo systemctl start docker

# Check status
sudo systemctl status docker

# Enable Docker to start on boot
sudo systemctl enable docker
```

### "No space left on device"

**Error**: Docker build fails with disk space error

**Solution**:
```bash
# Check Docker disk usage
docker system df

# Clean up (removes stopped containers, unused networks, dangling images)
docker system prune -a

# Remove specific images
docker images  # list images
docker rmi <IMAGE_ID>
```

### RViz/GUI Windows Don't Appear

**On Linux**:
```bash
# Allow Docker to access X server
xhost +local:docker

# If that doesn't work, try
xhost +
```

**On Windows (WSL2)**:
1. Install VcXsrv: [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/)
2. Launch XLaunch with "Disable access control" enabled
3. Set DISPLAY variable:
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

---

## Best Practices

### DO ✅

- **Use VSCode Dev Containers** for the best development experience
- **Source your workspace** in every new terminal: `source /home/ros2_ws/install/setup.bash`
- **Keep your code in the mounted volumes** (everything in `/home/ros2_ws/src`)
- **Stop containers when not using them** to free resources: `docker stop x86docker`
- **Regularly clean up** unused Docker resources: `docker system prune`

### DON'T ❌

- **Don't run `start_docker_x86.sh` multiple times** - use `docker start` + `docker exec` instead
- **Don't install critical packages with `apt` inside container** - they'll be lost when container is removed
- **Don't store important files outside `/home/ros2_ws/src`** - they won't persist
- **Don't forget to source the workspace** - ROS commands won't work otherwise

---

## Typical Development Cycle

Here's a complete example of a typical development session:

```bash
# ========== FIRST TIME SETUP (once) ==========
# On host machine
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh x86  # Takes 20-30 minutes
bash start_docker_x86.sh  # Creates container and enters shell

# ========== DAILY WORKFLOW ==========

# --- Day 1: Start working ---
# On host machine
docker start x86docker

# Open VSCode and attach to container (Dev Containers extension)
# Or use terminal:
docker exec -it x86docker bash

# Inside container
cd /home/ros2_ws
source install/setup.bash
ros2 launch curobo_ros gen_traj.launch.py

# Open another terminal (VSCode or docker exec)
ros2 service call /curobo_gen_traj/generate_trajectory ...

# --- End of day ---
# Stop container to free resources
docker stop x86docker

# --- Day 2: Continue working ---
docker start x86docker
# Attach VSCode or docker exec again
# Your code and builds are still there!
```

---

## Summary

**Key Takeaways**:

1. Build the Docker image **once**: `bash build_docker.sh x86`
2. Start the container **once**: `bash start_docker_x86.sh`
3. After that, use:
   - `docker start x86docker` to start
   - VSCode Dev Containers to work inside
   - `docker exec -it x86docker bash` for additional terminals
   - `docker stop x86docker` when done
4. Your code in `/home/ros2_ws/src` is **always preserved**
5. Need **~30 GB** disk space for the complete image

---

## Next Steps

Now that you understand Docker, let's set up the project:

→ **[Getting Started Guide](../getting_started.md)** - Build, start, and run your first trajectory

---

## Additional Resources

- **Docker Documentation**: [docs.docker.com](https://docs.docker.com/)
- **VSCode Dev Containers**: [code.visualstudio.com/docs/devcontainers/containers](https://code.visualstudio.com/docs/devcontainers/containers)
- **NVIDIA Container Toolkit**: [github.com/NVIDIA/nvidia-docker](https://github.com/NVIDIA/nvidia-docker)

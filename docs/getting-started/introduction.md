# Introduction to curobo_ros

Welcome! This guide will introduce you to **curobo_ros** and the key technologies it uses. If you're new to robotics software, ROS, or Docker, this is the perfect place to start.

---

## What is curobo_ros?

**curobo_ros** is a ROS 2 package that brings GPU-accelerated motion planning to robotic arms. It allows you to:

- **Plan collision-free trajectories** in milliseconds (not seconds!)
- **Solve inverse kinematics** (IK) to find joint angles for desired end-effector positions
- **Avoid obstacles** in real-time using depth cameras or manual object definitions
- **Visualize** everything live in RViz
- **Control real robots** or test in simulation

Think of it as a "smart motion planner" that uses your GPU to compute safe paths very quickly.

---

## Key Technologies

### 1. What is ROS 2?

**ROS (Robot Operating System)** is not actually an operating system - it's a **framework** for building robot software.

#### Why ROS?

- **Modular**: Different parts of your robot (sensors, motors, planners) run as separate programs called **nodes**
- **Communication**: Nodes talk to each other through **topics** (streaming data) and **services** (request/response)
- **Reusable**: Use existing packages instead of writing everything from scratch
- **Visualization**: Tools like RViz let you see what your robot is doing

#### Basic ROS Concepts

| Concept | Description | Example |
|---------|-------------|---------|
| **Node** | An executable program that does one job | `unified_planner` (plans trajectories) |
| **Topic** | A named data stream (publish/subscribe) | `/joint_states` (robot joint positions) |
| **Service** | Request/response communication | `/generate_trajectory` (ask for a path) |
| **Action** | Long-running task with feedback | `/send_trajectrory` (execute a trajectory) |
| **Message** | Data structure sent between nodes | `geometry_msgs/Pose` (position + orientation) |

#### Common ROS Commands

```bash
# List all running nodes
ros2 node list

# List all available topics
ros2 topic list

# See messages on a topic
ros2 topic echo /joint_states

# Call a service
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "{...}"

# Check node parameters
ros2 param list
```

### 2. What is Docker?

**Docker** is a tool that packages software with all its dependencies into a **container** - a lightweight, isolated environment.

#### Why Docker for this project?

Installing curobo_ros requires:
- ROS 2 Humble
- CUDA 12 (NVIDIA GPU libraries)
- PyTorch with CUDA support
- cuRobo library
- Many Python dependencies

Setting this up manually can take hours and often breaks. Docker gives you a **pre-built environment** where everything already works!

#### Docker Key Concepts

| Concept | Description | Analogy |
|---------|-------------|----------|
| **Image** | A template with all software pre-installed | Recipe for a cake |
| **Container** | A running instance of an image | The actual cake you baked |
| **Volume** | Share files between host and container | A shared folder |
| **Build** | Create an image from a Dockerfile | Following the recipe |

For setup details, DEV vs PROD workflows, and daily usage, see the [Installation Guide](installation.md).

### 3. What is cuRobo?

**cuRobo** (CUDA Robot) is NVIDIA's GPU-accelerated motion planning library.

#### Why GPU acceleration?

Traditional motion planning can take seconds. cuRobo uses your NVIDIA GPU to:
- Solve inverse kinematics in **milliseconds**
- Check thousands of collision scenarios **in parallel**
- Optimize trajectories **100x faster** than CPU-only methods

#### What cuRobo Does

- **Inverse Kinematics (IK)**: Given a target position/orientation, find joint angles
  - Example: "Move the gripper to (x=0.5, y=0.2, z=0.3)" → cuRobo calculates joint angles

- **Motion Generation**: Plan a smooth, collision-free trajectory from A to B
  - Considers joint limits, velocity limits, acceleration limits
  - Avoids obstacles
  - Optimizes for smoothness and efficiency

- **Collision Checking**: Uses voxel grids and sphere approximations
  - Robot represented as spheres
  - World represented as voxels
  - Ultra-fast GPU collision detection


---

## Prerequisites

Before starting, make sure you have:

- **Ubuntu 20.04 or 22.04** (or Windows 11 with WSL2)
- **NVIDIA GPU** with drivers installed (check with `nvidia-smi`)
- **~30 GB free disk space** (Docker image is large!)
- **Docker** and **NVIDIA Container Toolkit** installed
- **Basic command line** knowledge (cd, ls, etc.)

---

## Next Steps

Ready to get started?

1. **[Installation Guide](installation.md)** - Set up Docker, build the image, and run your first trajectory
3. **[Parameters Guide](../concepts/parameters.md)** - Understand important configuration parameters

---

## Getting Help

- **Troubleshooting**: See [troubleshooting.md](troubleshooting.md) for common issues
- **ROS 2 Documentation**: [docs.ros.org](https://docs.ros.org/en/humble/)
- **cuRobo Documentation**: [curobo.org](https://curobo.org)
- **Docker Documentation**: [docs.docker.com](https://docs.docker.com/)

---

**Let's begin!** → [Installation Guide](installation.md)

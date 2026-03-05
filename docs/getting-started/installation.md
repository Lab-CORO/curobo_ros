# Install curobo_ros

This guide takes you from a fresh machine to a running GPU-accelerated trajectory in RViz.
Everything runs inside a Docker image so your host stays clean.

---

## Choose Your Workflow

| Workflow | Best For | Image Size | curobo_ros |
|----------|----------|------------|------------|
| **DEV Mode** | Modifying curobo_ros internals | ~25-30 GB | Mounted from host (live editing) |
| **PROD Mode** | Using curobo_ros in your projects | ~15-20 GB | Pre-installed in container |

**Choose DEV if** you want to contribute to or modify curobo_ros source code.
**Choose PROD if** you want to use curobo_ros as a dependency in your robot project.

---

## Prerequisites

| Requirement | Notes |
|-------------|-------|
| **Ubuntu 20.04 / 22.04** (or Windows 11 + WSL 2) | Host system tested in CI. |
| **NVIDIA GPU** with recent drivers | CUDA 12 is used in the container. Check with `nvidia-smi`. |
| **~30 GB free disk space** | Docker image and dependencies are large. |
| **Docker ≥ 24** & **NVIDIA Container Toolkit** | Follow [nvidia.github.io/nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). |

```bash
# Verify prerequisites
nvidia-smi
docker --version
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

---

## PROD Mode Setup

### 1. Clone & Build Image

```bash
cd ~
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules
cd curobo_ros/docker
bash build_docker.sh  # GPU → PROD (mode 2)
```

**Result:** `curobo_ros:ampere-prod` — smaller image with curobo_ros pre-installed.

### 2. Create Your Workspace

> **Create your workspace on the host machine, not inside the container.**
> Docker containers are ephemeral: if you delete or recreate a container, everything stored
> inside that is not in a mounted volume is lost. By keeping your workspace on the host,
> your code and built packages are safe regardless of what happens to the container.

```bash
# On the host machine
mkdir -p ~/my_robot_ws/src
# Place your packages in ~/my_robot_ws/src/
```

This path will be requested when you run `start_docker_x86.sh` (Step 3/3 prompt: `Workspace path:`).
The script mounts it into the container at `/home/ros2_ws`, so any file you edit on the host
is immediately visible inside the container, and vice versa.

### 3. Start the Container

```bash
bash start_docker_x86.sh  # GPU → PROD → enter your workspace path
```

Your workspace is mounted at `/home/ros2_ws`. curobo_ros is pre-installed in `/home/curobo_ws` and auto-sourced on startup.

---

## Daily Workflow

### Opening Additional Terminals

**Option 1 — VSCode (recommended):**
1. Install the "Dev Containers" extension
2. Click `><` in the bottom-left → "Attach to Running Container..."
3. Open terminals with Terminal → New Terminal (all run inside the container)

**Option 2 — docker exec:**
```bash
# On your host
docker exec -it curobo_ampere_dev bash
source /home/ros2_ws/install/setup.bash
```

### Starting a Stopped Container

After the first run, never use the start script again — use:

```bash
docker start curobo_ampere_dev
docker exec -it curobo_ampere_dev bash
```

---



## DEV Mode Setup

### 1. Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules
```

### 2. Import Dependencies

```bash
cd ~/ros2_ws/src
sudo apt install python3-vcstool
vcs import < curobo_ros/my.repos
```

This imports `curobo_msgs` (custom messages) and `curobo_rviz` (RViz plugin).

**Resulting workspace:**
```
~/ros2_ws/
└── src/
    ├── curobo_ros/
    ├── curobo_msgs/
    └── curobo_rviz/
```

### 3. Build the Docker Image

> ⚠️ Takes 20-30 minutes and requires ~30 GB disk space.

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash build_docker.sh
```

**Interactive prompts:**

1. **GPU architecture** — Ampere (RTX 30XX) / Ada Lovelace (RTX 40XX) / Turing (RTX 20XX) / Volta
2. **Mode** — select `1` for DEV

**Result:** image named `curobo_ros:ampere-dev` (or your GPU + mode).

### 4. Start the Container

> ⚠️ Run this script **once** to create the container. Afterward use `docker start` + `docker exec`.

```bash
bash start_docker_x86.sh
```

**Prompts:** same GPU → DEV mode → confirm mount.

**What gets mounted:**
- `~/ros2_ws/src/curobo_ros` → `/home/ros2_ws/src/curobo_ros`
- `~/ros2_ws/src/curobo_rviz` → `/home/ros2_ws/src/curobo_rviz`
- `~/ros2_ws/src/curobo_msgs` → `/home/ros2_ws/src/curobo_msgs`

GPU access and X11 forwarding (RViz) are enabled automatically.

### 5. Build the Workspace

All commands below run **inside the container.**

```bash
cd /home/ros2_ws
colcon build
```

**Build time:** ~2-5 minutes on first build.

### 6. Source the Environment

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
```

### 7. Launch the System

```bash
ros2 launch curobo_ros gen_traj.launch.py
```

**First launch warmup** (~30-60 s): the node compiles GPU kernels. RViz then opens showing the default robot (Doosan M1013).

> ⚠️ RViz doesn't appear? See [Troubleshooting](troubleshooting.md#rviz--gui-windows-do-not-appear).




## Next Steps

- [Your First Trajectory](../tutorials/01-first-trajectory.md)
- [Adding Your Robot](../tutorials/02-adding-your-robot.md)
- [Troubleshooting](troubleshooting.md)
- [Unified Planner Concepts](../concepts/unified-planner.md)

# Install curobo_ros

This guide takes you from a fresh machine to a running GPU-accelerated trajectory in RViz. Everything runs inside a Docker image so your host stays clean.

The supported images are the **ROS 2 Jazzy** dockerfiles — they match the cuRobo **v0.8.0 (v2)** dependency the code requires:

| Platform | Dockerfile | Base |
|---|---|---|
| x86 + NVIDIA GPU | `docker/x86_jazzy.dockerfile` | `nvcr.io/nvidia/pytorch:24.11-py3` (Ubuntu 24.04) |
| Jetson / aarch64 | `docker/aarch64_jazzy.dockerfile` | `nvcr.io/nvidia/cuda:13.2.0-cudnn-devel-ubuntu24.04` |

```{note}
`docker/build_docker.sh` wraps these same two x86 images behind an interactive GPU-architecture prompt. The explicit builds below are the reference — reach for the script only if you want it to pick `TORCH_CUDA_ARCH_LIST` for you.
```

## Choose your workflow

| Workflow | Best for | curobo_ros source |
|---|---|---|
| **DEV mode** | Modifying curobo_ros internals | Mounted from the host (live editing) |
| **PROD mode** | Using curobo_ros in your own project | Pre-installed in the container; your workspace is mounted |

## Prerequisites

| Requirement | Notes |
|---|---|
| **Ubuntu 22.04 / 24.04** (or Windows 11 + WSL 2), or a Jetson with JetPack 6+ | |
| **NVIDIA GPU** with recent drivers | CUDA ≥ 12 inside the container. Check with `nvidia-smi` |
| **~30 GB free disk space** | The image and its dependencies are large |
| **Docker ≥ 24** and the **NVIDIA Container Toolkit** | [Install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |

```bash
# Verify GPU passthrough works before anything else
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

## 1. Clone the repositories

On the host (required for DEV mode; PROD builds can clone inside the image build):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/Lab-CORO/curobo_ros.git
sudo apt install python3-vcstool   # if you don't have vcs
vcs import < curobo_ros/my.repos   # pulls curobo_msgs and curobo_rviz
```

## 2. Build the image

### x86

Pick the CUDA architecture matching your GPU (`75` = RTX 20xx, `86` = RTX 30xx, `89` = RTX 40xx, `80` = A100, `90` = H100) and tag the image `curobo_ros:<gpu_name>-dev` (or `-prod`) so the start script finds it — `<gpu_name>` is one of `ampere`, `ada_lovelace`, `turing`, `volta`:

```bash
cd ~/ros2_ws/src/curobo_ros/docker
docker build -f x86_jazzy.dockerfile \
  --build-arg CUDA_ARCH=86 --build-arg TORCH_CUDA_ARCH_LIST="8.6" \
  -t curobo_ros:ampere-dev .
```

The build compiles cuRobo's CUDA kernels — expect 20–30 minutes. If the build crashes from lack of RAM, lower the parallelism with `--build-arg MAKEFLAGS="-j2"` (~1.5 GB RAM per job).

### Jetson (aarch64)

```bash
cd ~/ros2_ws/src/curobo_ros/docker
docker build -f aarch64_jazzy.dockerfile \
  --build-arg CUDA_ARCH=87 --build-arg TORCH_CUDA_ARCH_LIST="8.7" \
  -t curobo_ros:jetson-dev .
```

`CUDA_ARCH=87` is for Orin (AGX/NX); use `72` for Xavier NX. On Jetson the L4T libraries are mounted from the host at runtime by the NVIDIA container runtime.

## 3. Start the container

### With the start script (x86)

```bash
cd ~/ros2_ws/src/curobo_ros/docker
bash start_docker_x86.sh
```

The script asks three questions: GPU architecture (to pick the image tag), **DEV or PROD**, and — in PROD — the path of *your* workspace to mount.

- **DEV** mounts `~/ros2_ws/src/{curobo_ros, curobo_rviz, curobo_msgs}` from the host into `/home/ros2_ws/src/` — edits on the host are live in the container.
- **PROD** mounts your workspace at `/home/ros2_ws`; curobo_ros itself is pre-installed in the image.

The container runs with `--gpus all --network host --privileged` and X11 forwarding, and is named after the image (e.g. `curobo_ampere_dev`).

### Manually (Jetson, or if you prefer explicit commands)

```bash
docker run --name curobo_jetson_dev -it --gpus all --network host --privileged \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws/src/curobo_ros:/home/ros2_ws/src/curobo_ros \
  -v ~/ros2_ws/src/curobo_rviz:/home/ros2_ws/src/curobo_rviz \
  -v ~/ros2_ws/src/curobo_msgs:/home/ros2_ws/src/curobo_msgs \
  curobo_ros:jetson-dev
```

## 4. Build the workspace (inside the container)

```bash
cd /home/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

First build takes a few minutes; later builds are incremental.

## 5. First launch

```bash
ros2 launch curobo_ros gen_traj.launch.py robot:=emulator
```

RViz opens, then the planner warms up its GPU solvers — **expect 25–35 seconds** before the panels react. When `ros2 param get /unified_planner node_is_available` returns `true`, you are ready for [Tutorial 1: Your First Trajectory](../tutorials/01-first-trajectory.md).

`robot:=emulator` runs without hardware. Omit it (default `robot:=doosan_m1013`) only when the Doosan bridge is up — see [Tutorial 4](../tutorials/04-robot-execution.md).

## Daily workflow

The start script is only for the *first* run (it creates the container). Afterwards:

```bash
docker start -ai curobo_ampere_dev        # restart a stopped container
docker exec -it curobo_ampere_dev bash    # extra shells in a running container
```

If you use VS Code, the *Dev Containers* extension can attach directly to the running container ("Attach to Running Container"), giving you an IDE inside the environment.

## Next steps

- [Tutorial 1: Your First Trajectory](../tutorials/01-first-trajectory.md)
- [Testing](testing.md) — run the integration suites to validate your install
- [Troubleshooting](troubleshooting.md) — GPU passthrough, X11, and common Docker issues

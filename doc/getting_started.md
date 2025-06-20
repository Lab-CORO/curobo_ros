# Getting Started with **curobo_ros**

This guide shows the fastest route from cloning the repository to seeing a
GPU-accelerated trajectory in RViz.  
Everything runs inside a ready-made Docker image, so your host OS stays clean.

---

## 1. Prerequisites

| Requirement | Notes |
|-------------|-------|
| **Ubuntu 20.04 / 22.04** (or Windows 11 + WSL 2) | Host system tested in CI. |
| **NVIDIA GPU** with recent drivers | CUDA 12 is used in the container. |
| **Docker ≥ 24** & **NVIDIA Container Toolkit** | Follow the install steps on [nvidia.github.io/nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). |

---

## 2. Clone the repo

```bash
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules
```
To setup the dev environement, you need this tool:
```
sudo apt install python3-vcstool
vcs import < my.repos
```
This package is mainly for developpement so every packages are include in the docker throught volumes share.


---

## 3. Build the Docker image (x86 *+ GPU*)

```bash
cd curobo_ros/docker
bash build_docker.sh x86
```

The script creates an image called **`curobo_ros:x86`** with all ROS 2 Humble,
CUDA 12 and curobo dependencies pre-installed.

---

## 4. Start the container

```bash
bash start_docker_x86.sh
```

You will be dropped into a shell inside
`/workspaces/curobo_ros`.

### Open extra terminals

```bash
docker exec -it curobo_x86 bash   # terminal #2
```

*(The container name `curobo_x86` is set by the start-script.)*

---

## 8. Next steps

* **Tutorials** – hands-on guides for IK/FK services, custom robots, and more.
* **Concepts** – learn how curobo’s GPU core interfaces with ROS 2.
* **Troubleshooting** – common CUDA/X-server issues and their fixes.


FROM nvcr.io/nvidia/pytorch:25.01-py3

# ── Base: NVIDIA PyTorch 25.01 → CUDA 12.8 / PyTorch 2.6 / Ubuntu 24.04 Noble
# Ubuntu 24.04 Noble is required for ROS 2 Jazzy.

LABEL maintainer="Lab-CORO"
LABEL description="lab_coro forks of nvblox + curobo, compiled from source, with ROS 2 Jazzy"

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy

# ── Branches des forks Lab-CORO ───────────────────────────────────────────────
ARG NVBLOX_BRANCH=ubuntu24
ARG CUROBO_BRANCH=lab-coro

# ── Architectures CUDA cibles (adapter selon le parc GPU) ─────────────────────
ARG TORCH_CUDA_ARCH_LIST="7.0 7.5 8.0 8.6 8.9 9.0+PTX"
ENV TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_LIST}

# ── Variables d'environnement globales ────────────────────────────────────────
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV PATH="${PATH}:/usr/local/cuda/bin"
ENV CUDA_PATH=/usr/local/cuda
# Headless OpenGL via EGL (nécessaire pour les tests de rendu et nvblox)
ENV PYOPENGL_PLATFORM=egl
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 1 — Outils système de base
# ═══════════════════════════════════════════════════════════════════════════════
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    git-lfs \
    gnupg2 \
    iputils-ping \
    libegl1-mesa-dev \
    libeigen3-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglvnd-dev \
    libssl-dev \
    lsb-release \
    ninja-build \
    pkg-config \
    software-properties-common \
    sudo \
    tzdata \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# python3-blinker du système (sans fichier RECORD) bloque la mise à jour pip
# vers blinker>=1.9.0 requise par open3d (dépendance de nvblox_torch)
RUN apt-get remove -y python3-blinker || true

# EGL ICD NVIDIA pour le rendu headless (OpenGL sans display)
RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' \
    >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 2 — numpy<2 (pin avant tout le reste)
# nvblox_torch est compilé contre l'ABI numpy 1.x ;
# torch.Tensor.numpy() lève RuntimeError avec numpy 2.x
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip install --no-cache-dir "numpy<2"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 3 — lab_coro-nvblox : cloner + compiler nvblox_torch
# ═══════════════════════════════════════════════════════════════════════════════
WORKDIR /pkgs
RUN git clone --depth 1 --branch ${NVBLOX_BRANCH} \
    https://github.com/Lab-CORO/nvblox.git lab_coro-nvblox

# Compiler libpy_nvblox.so via cmake AVANT pip install.
# Raison : le dépôt contient un symlink nvblox_torch/lib/.../libpy_nvblox.so →
# ../../../../../build/.../libpy_nvblox.so  — symlink dangling sans cmake.
# Sans cette étape, pip install copie le symlink cassé et le .so est absent.
#
# TORCH_CUDA_ARCH_LIST (ex: "8.9") → CMAKE_CUDA_ARCHITECTURES (ex: "89")
# cmake ne peut pas détecter le GPU pendant docker build (pas de --gpus),
# il faut donc passer l'architecture explicitement.
RUN CMAKE_CUDA_ARCH=$(python3 -c "import os; a=os.environ.get('TORCH_CUDA_ARCH_LIST','8.0'); print(';'.join(x.replace('.','').split('+')[0] for x in a.split()))") && \
    TORCH_CMAKE_PREFIX=$(python3 -c "import torch; print(torch.utils.cmake_prefix_path)") && \
    cd /pkgs/lab_coro-nvblox && mkdir -p build && cd build && \
    cmake .. \
        -DBUILD_TESTING=OFF \
        -DBUILD_PYTORCH_WRAPPER=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCH}" \
        -DCMAKE_PREFIX_PATH="${TORCH_CMAKE_PREFIX}" \
    && make -j$(nproc)

# pip install suit le symlink (maintenant résolu) et copie la vraie .so
RUN pip install --no-cache-dir /pkgs/lab_coro-nvblox/nvblox_torch/ \
    --no-build-isolation

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 4 — lab_coro-curobo : cloner + installer
# ═══════════════════════════════════════════════════════════════════════════════
RUN git clone --depth 1 --branch ${CUROBO_BRANCH} \
    https://github.com/Lab-CORO/curobo.git lab_coro-curobo

RUN pip install --no-cache-dir /pkgs/lab_coro-curobo/ \
    --no-build-isolation

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 5 — Re-pin numpy<2 (certains installs ramènent numpy 2.x)
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip install --no-cache-dir "numpy<2"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 6 — Utilitaires Python complémentaires
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip install --no-cache-dir \
    open3d \
    pyrealsense2 \
    transforms3d \
    trimesh \
    warp-lang

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 7 — ROS 2 Jazzy (Ubuntu 24.04 Noble)
# ═══════════════════════════════════════════════════════════════════════════════

# Ajouter la source apt officielle ROS 2
RUN export ROS_APT_SOURCE_VERSION=$(curl -s \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Installer ROS 2 Jazzy + extensions couramment utilisées
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-desktop \
    ros-jazzy-cyclonedds \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-moveit \
    ros-jazzy-nav2-msgs \
    ros-jazzy-pcl-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# ─── rosdep ───────────────────────────────────────────────────────────────────
RUN rosdep init && rosdep update

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 8 — Workspace ROS 2
# curobo_ros / curobo_msgs / curobo_rviz → montés en volume en mode DEV
# ═══════════════════════════════════════════════════════════════════════════════
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws/src

# Helpers disponibles au build de l'espace de travail
RUN git clone https://github.com/swri-robotics/trajectory_preview.git
RUN git clone https://github.com/Box-Robotics/ros2_numpy.git
RUN git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules && \
    git clone https://github.com/Lab-CORO/curobo_msgs.git && \
    git clone https://github.com/Lab-CORO/curobo_rviz.git
    
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 9 — Environnement final
# ═══════════════════════════════════════════════════════════════════════════════
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros2_ws

# ── Utilisation ───────────────────────────────────────────────────────────────
# Build (depuis la racine du workspace debug_curobo) :
#   docker build -f curobo_ros/docker/jazzy.dockerfile \
#       --build-arg TORCH_CUDA_ARCH_LIST="8.9" \   # adapter à votre GPU
#       -t curobo-ros-jazzy:latest .
#
# Run (mode DEV — monter curobo_ros, curobo_msgs, curobo_rviz) :
#   docker run --gpus all -it --rm \
#       --ipc=host --ulimit memlock=-1 --ulimit stack=67108864 \
#       -v $(pwd)/curobo_ros:/home/ros2_ws/src/curobo_ros \
#       -v $(pwd)/curobo_msgs:/home/ros2_ws/src/curobo_msgs \
#       -v $(pwd)/curobo_rviz:/home/ros2_ws/src/curobo_rviz \
#       curobo-ros-jazzy:latest bash

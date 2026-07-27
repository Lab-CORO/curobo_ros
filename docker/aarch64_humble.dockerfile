FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# ── Base: L4T JetPack 6 r36.4.0 → CUDA 12.x / Ubuntu 22.04 Jammy / Python 3.10
# Ubuntu 22.04 Jammy est requis pour ROS 2 Humble sur Jetson (JetPack 6).
# nvcr.io/nvidia/pytorch:25.01-py3 n'a pas de variante aarch64 — PyTorch est
# installé depuis les wheels NVIDIA hébergés (torch 2.3.0, cp310, aarch64).

LABEL maintainer="Lab-CORO"
LABEL description="lab_coro forks of nvblox + curobo, compiled from source, with ROS 2 Humble — Jetson Orin AGX (JetPack 6)"

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

# ── Branches des forks Lab-CORO ───────────────────────────────────────────────
ARG NVBLOX_BRANCH=ubuntu24
ARG CUROBO_BRANCH=lab-coro

# ── Architecture CUDA cible : Jetson Orin AGX = sm_87 (Ampere, fixe) ─────────
# Pas d'ARG ici : l'Orin est toujours sm_87, pas de variante à configurer.

# ── Variables d'environnement globales ────────────────────────────────────────
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV PATH="${PATH}:/usr/local/cuda/bin"
ENV CUDA_PATH=/usr/local/cuda
# Headless OpenGL via EGL (nécessaire pour les tests de rendu et nvblox)
ENV PYOPENGL_PLATFORM=egl
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Jetson Orin AGX = sm_87 — requis pour que torch.utils.cpp_extension
# détecte l'architecture CUDA lors de la compilation sans GPU disponible.
ENV TORCH_CUDA_ARCH_LIST="8.7"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 1 — Outils système de base
# Identique à jazzy.dockerfile + libopenblas-dev / libopenmpi-dev
# requis par les wheels PyTorch Jetson (BLAS et MPI backend)
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
    libopenblas-dev \
    libopenmpi-dev \
    libssl-dev \
    lsb-release \
    ninja-build \
    pkg-config \
    python3-pip \
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
# ÉTAPE 2 — PyTorch pour Jetson (aarch64, CUDA 12, Python 3.10)
# Pas de wheel PyPI pour aarch64 — wheels NVIDIA hébergés sur nvidia.box.com
# Sources : lab_coro-nvblox/docker/torch_install_helper.py
#   torch-2.3.0-cp310-cp310-linux_aarch64.whl
#   torchvision-0.18.0-cp310-cp310-linux_aarch64.whl
# ═══════════════════════════════════════════════════════════════════════════════
# Les URLs nvidia.box.com sont des redirections — curl -fL (fail + follow) est
# plus fiable que wget pour suivre ces redirections et détecter les erreurs HTTP.
RUN curl -fL -o /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl \
        "https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl" && \
    curl -fL -o /tmp/torchvision-0.18.0-cp310-cp310-linux_aarch64.whl \
        "https://nvidia.box.com/shared/static/xpr06qe6ql3l6rj22cu3c45tz1wzi36p.whl" && \
    pip3 install --no-cache-dir \
        /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl \
        /tmp/torchvision-0.18.0-cp310-cp310-linux_aarch64.whl && \
    rm /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl \
       /tmp/torchvision-0.18.0-cp310-cp310-linux_aarch64.whl

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 3 — numpy<2 (pin avant tout le reste)
# nvblox_torch compilé contre l'ABI numpy 1.x ; numpy 2.x casse .numpy()
# Note : cmake 3.22.1 est déjà installé via apt (ÉTAPE 1) — ne pas upgrader
# via pip car cmake>=4.x casse la compatibilité gflags (dépendance de nvblox).
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip3 install --no-cache-dir "numpy<2"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 4 — lab_coro-nvblox : cloner + compiler nvblox_torch
# ═══════════════════════════════════════════════════════════════════════════════
WORKDIR /pkgs
RUN git clone --depth 1 --branch ${NVBLOX_BRANCH} \
    https://github.com/Lab-CORO/nvblox.git lab_coro-nvblox

# Compiler libpy_nvblox.so via cmake AVANT pip install.
# Raison : le dépôt contient un symlink nvblox_torch/lib/.../libpy_nvblox.so →
# ../../../../../build/.../libpy_nvblox.so  — symlink dangling sans cmake.
#
# CMAKE_CUDA_ARCHITECTURES=87 est fixe : le Jetson Orin AGX est toujours sm_87.
# cmake ne peut pas détecter le GPU pendant docker build (pas de --gpus).
# Note cmake : nvblox_torch/cpp/CMakeLists.txt ajoute automatiquement -lcudnn
# sur aarch64 (if CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64").
RUN TORCH_CMAKE_PREFIX=$(python3 -c "import torch; print(torch.utils.cmake_prefix_path)") && \
    cd /pkgs/lab_coro-nvblox && mkdir -p build && cd build && \
    cmake .. \
        -DBUILD_TESTING=OFF \
        -DBUILD_PYTORCH_WRAPPER=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_ARCHITECTURES="87" \
        -DCMAKE_PREFIX_PATH="${TORCH_CMAKE_PREFIX}" \
    && make -j$(nproc)

# pip install nvblox_torch — ATTENTION : le setup.py appelle setup() uniquement
# dans if __name__ == '__main__', ce qui est sauté quand pip/setuptools exécute
# le script via exec() (build_meta). Sans cet appel, la version dynamique du
# pyproject.toml est non-résolue → package nommé UNKNOWN, vide (1 kB).
#
# Fix : (1) upgrader setuptools>=61 pour que pyproject.toml [project] soit lu,
#        (2) invoquer setup.py directement (python3 setup.py bdist_wheel) pour
#            que __name__=='__main__' → create_version_file() + setup() appelés,
#        (3) installer la wheel produite.
RUN pip3 install --no-cache-dir "setuptools>=61,<70" wheel && \
    cd /pkgs/lab_coro-nvblox/nvblox_torch && \
    python3 setup.py bdist_wheel && \
    pip3 install --no-cache-dir dist/nvblox_torch*.whl || \
    pip3 install --no-cache-dir dist/*.whl

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 5 — lab_coro-curobo : cloner + installer
# ═══════════════════════════════════════════════════════════════════════════════
RUN git clone --depth 1 --branch ${CUROBO_BRANCH} \
    https://github.com/Lab-CORO/curobo.git lab_coro-curobo

RUN pip3 install --no-cache-dir /pkgs/lab_coro-curobo/ \
    --no-build-isolation

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 6 — Re-pin numpy<2 (certains installs ramènent numpy 2.x)
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip3 install --no-cache-dir "numpy<2"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 7 — Utilitaires Python complémentaires
# Note : pyrealsense2 n'a pas de wheel pip pour aarch64.
#        Utiliser apt librealsense2-python si la RealSense est nécessaire.
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip3 install --no-cache-dir \
    open3d \
    transforms3d \
    trimesh \
    warp-lang

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 8 — ROS 2 Humble (Ubuntu 22.04 Jammy)
# ═══════════════════════════════════════════════════════════════════════════════

# Ajouter la source apt officielle ROS 2
RUN export ROS_APT_SOURCE_VERSION=$(curl -s \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Installer ROS 2 Humble + extensions couramment utilisées
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-desktop \
    ros-humble-cyclonedds \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit \
    ros-humble-nav2-msgs \
    ros-humble-pcl-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# ─── rosdep ───────────────────────────────────────────────────────────────────
RUN rosdep init && rosdep update

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 9 — Workspace ROS 2
# curobo_ros / curobo_msgs / curobo_rviz → montés en volume en mode DEV
# ═══════════════════════════════════════════════════════════════════════════════
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws/src

# Helpers disponibles au build de l'espace de travail
RUN git clone https://github.com/swri-robotics/trajectory_preview.git
RUN git clone https://github.com/Box-Robotics/ros2_numpy.git


WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 10 — Environnement final
# ═══════════════════════════════════════════════════════════════════════════════
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros2_ws

# ── Utilisation ───────────────────────────────────────────────────────────────
# Build (depuis la racine du workspace debug_curobo) :
#   docker build -f curobo_ros/docker/humble.dockerfile \
#       -t curobo-ros-humble:latest .
#
# Run (mode DEV — monter curobo_ros, curobo_msgs, curobo_rviz) :
#   docker run --gpus all -it --rm \
#       --ipc=host --ulimit memlock=-1 --ulimit stack=67108864 \
#       -v $(pwd)/curobo_ros:/home/ros2_ws/src/curobo_ros \
#       -v $(pwd)/curobo_msgs:/home/ros2_ws/src/curobo_msgs \
#       -v $(pwd)/curobo_rviz:/home/ros2_ws/src/curobo_rviz \
#       curobo-ros-humble:latest bash

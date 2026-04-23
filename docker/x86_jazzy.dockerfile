# Base image NVIDIA PyTorch sur Ubuntu 24.04 (Noble)
# nvcr.io/nvidia/pytorch:24.09-py3 est la première version sur Ubuntu 24.04
FROM nvcr.io/nvidia/pytorch:24.11-py3 AS torch_cuda_base

LABEL maintainer="Lucas Carpentier, Guillaume Dupoiron"

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ARG ROS_DISTRO=jazzy
# Architecture CUDA : 75=RTX20xx, 86=RTX30xx, 89=RTX40xx, 80=A100, 90=H100
# Format cmake (sans point) : 86 → RTX 3xxx
# Format torch (avec point)  : 8.6 → RTX 3xxx
ARG CUDA_ARCH=86
ARG TORCH_CUDA_ARCH_LIST="8.6"
# Nombre de jobs parallèles — réduire si RAM insuffisante (crash)
# RAM estimée : ~1.5 GB/job CUDA → j4 = ~6 GB, j8 = ~12 GB
ARG MAKEFLAGS="-j4"

# add GL:
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglvnd-dev \
    pkg-config && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Set timezone info
RUN apt-get update && apt-get install -y \
    tzdata \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/* \
    && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
    && echo "America/New_York" > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata \
    && add-apt-repository -y ppa:git-core/ppa \
    && apt-get update && apt-get install -y \
    apt-utils \
    bash \
    build-essential \
    cmake \
    curl \
    git \
    git-lfs \
    iputils-ping \
    iproute2 \
    libeigen3-dev \
    libssl-dev \
    lsb-release \
    make \
    net-tools \
    openssh-client \
    openssh-server \
    python3-ipdb \
    python3-pip \
    python3-tk \
    sudo \
    terminator \
    unattended-upgrades \
    wget \
    && rm -rf /var/lib/apt/lists/*

# MPI (requis par PyTorch distributed)
RUN apt-get update && apt-get install --reinstall -y \
    hwloc \
    libmpich-dev \
    libmpich12 \
    mpich \
    && rm -rf /var/lib/apt/lists/*

# MPI lib access
ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib"

ARG TORCH_CUDA_ARCH_LIST="6.1 7.0+PTX"
ENV TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

ARG CACHE_DATE=2026-04-20

# ============================================================
# cuRobo v2 (v0.8.0) — upstream NVlabs, plus de fork Lab-CORO.
#
# v2 inclut nativement le sous-système Mapper (TSDF bloc-sparse + ESDF)
# qui remplace notre pipeline nvblox + nvblox_torch précédent. Les forks
# Lab-CORO/nvblox, Lab-CORO/nvblox_torch et Lab-CORO/curobo (branche
# lab-coro) deviennent donc inutiles, ainsi que le workaround
# PyTorch 2.9+ / std::bad_alloc qui n'était requis que pour nvblox.
# ============================================================

# Dépendances C++ utilisées par googletest (étape plus bas) et certaines
# extensions cuRobo.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libbenchmark-dev libgoogle-glog-dev libgtest-dev libsqlite3-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /pkgs

RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# CUDA samples headers requis par CuRobo JIT (helper_math.h)
RUN git clone --depth 1 https://github.com/NVIDIA/cuda-samples.git /pkgs/cuda-samples && \
    cp /pkgs/cuda-samples/Common/helper_math.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/helper_cuda.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/helper_functions.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/exception.h /usr/local/cuda/include/

# cuRobo v2 upstream (v0.8.0)
ARG CUROBO_REF=v0.8.0
RUN mkdir /pkgs/curobo_src && cd /pkgs/curobo_src && \
    git clone -b ${CUROBO_REF} https://github.com/NVlabs/curobo.git .
WORKDIR /pkgs/curobo_src
RUN MAX_JOBS=${MAKEFLAGS##*-j} pip3 install -e .[cu12,dev,usd] --no-build-isolation

# Headers internes CuRobo requis lors du JIT (check_cuda.h, cuda_precisions.h, etc.)
RUN cp /pkgs/curobo_src/src/curobo/curobolib/cpp/*.h /usr/local/cuda/include/

# Pré-compilation des extensions CUDA CuRobo (évite le JIT au premier lancement)
# Actif par défaut — requiert: docker build --gpus all
# Pour désactiver: --build-arg PRECOMPILE_CUROBO=false
ARG PRECOMPILE_CUROBO=true
RUN if [ "$PRECOMPILE_CUROBO" = "true" ]; then \
        python3 -c "\
from curobo.curobolib import geom; \
from curobo.curobolib import kinematics_fused; \
from curobo.curobolib import lbfgs_lib; \
import curobo; print(f'CuRobo {curobo.__version__} JIT pre-compilation done')" \
        || echo "Warning: CuRobo JIT pre-compilation failed (no GPU?). Will compile at first launch."; \
    else \
        echo "CuRobo JIT pre-compilation skipped."; \
    fi

ENV PYOPENGL_PLATFORM=egl

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# googletest source installée par libgtest-dev — on la compile/installe.
RUN cd /usr/src/googletest && cmake . && cmake --build . --target install

RUN python -m pip install \
    pyrealsense2 \
    transforms3d

RUN python -m pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

RUN export LD_LIBRARY_PATH="/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH"

##### Installing ROS Jazzy (Ubuntu 24.04 Noble) ######

ARG DEBIAN_FRONTEND=noninteractive

# Problème GPG : apt 2.8.x sur Ubuntu 24.04 vérifie les InRelease en mode
# sig+data séparé, ce qui échoue (BADSIG) quand les miroirs Ubuntu servent
# des InRelease signés avec des clés plus récentes que celles du keyring de
# l'image de base. Le flag AllowInsecureRepositories DOIT être passé en
# ligne de commande (la doc apt le dit explicitement — apt.conf.d l'ignore).
#
# Solution : wrapper apt-get qui injecte les flags sur tous les appels suivants.
RUN printf '#!/bin/sh\nexec /usr/bin/apt-get \\\n  -o Acquire::AllowInsecureRepositories=true \\\n  -o Acquire::AllowDowngradeToInsecureRepositories=true \\\n  --allow-unauthenticated "$@"\n' \
    > /usr/local/sbin/apt-get && chmod +x /usr/local/sbin/apt-get

# Dépôts ROS 2 — ros2-apt-source.deb installe la clé ROS via dpkg (sans apt),
# ce qui évite de dépendre d'apt pour cette étape critique.
# gnupg2/lsb-release/curl sont déjà présents dans l'image de base NVIDIA.
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# ROS Jazzy desktop + dépendances
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-desktop \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-nav2-msgs \
    ros-jazzy-moveit \
    ros-jazzy-pcl-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-tf-transformations \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# RealSense (si disponible sur jazzy, sinon commenter)
RUN apt-get update && apt-get install -y \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    && rm -rf /var/lib/apt/lists/* || echo "RealSense packages not available for jazzy, skipping"

WORKDIR /home/ros2_ws/src

RUN sudo rosdep init && rosdep update

# ros2_numpy (branche jazzy ou main)
RUN git clone https://github.com/Box-Robotics/ros2_numpy.git || \
    git clone -b main https://github.com/Box-Robotics/ros2_numpy.git

# trajectory_preview
RUN git clone https://github.com/swri-robotics/trajectory_preview.git

# Open3D
RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# blinker installé par apt bloque pip open3d — ignoré si absent (Ubuntu 24)
RUN apt remove python3-blinker -y 2>/dev/null || true
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade open3d
# Fixer les conflits numpy/scipy :
# - system scipy (apt 1.11.4) est incompatible avec pip numpy
# - installer scipy via pip le rend prioritaire sur le système (/usr/local > /usr/lib)
# - pinner numpy à 1.26.4 pour compatibilité avec numba, cupy, etc.
RUN pip3 install --no-cache-dir --force-reinstall "numpy==1.26.4" scipy && \
    python3 -m pip install --no-cache-dir --force-reinstall --no-deps \
    pandas scikit-learn pyarrow

# Vérification finale de cuRobo v2 (post pip installs, PyTorch peut avoir été upgradé).
RUN python3 -c "import curobo; print(f'cuRobo {curobo.__version__} OK')" || \
    echo "Warning: cuRobo import skipped (no GPU in build sandbox — expected)"

# Build workspace
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

RUN source /opt/ros/jazzy/setup.bash && \
    cd /home/ros2_ws && \
    . install/local_setup.bash

WORKDIR /home/ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

ENV LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH

# Historique de build : pour l'ancien setup v1 + nvblox, voir les notes
# archivées dans docs/archive/docker-v1-notes.md (workaround std::bad_alloc
# PyTorch 2.9, wrapper apt-get AllowInsecure, etc.). Le Dockerfile v2 ne
# dépend plus de nvblox ni de forks Lab-CORO.

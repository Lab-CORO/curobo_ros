# aarch64 Jazzy — CuRobo v2 (v0.8.0), CUDA 13.2, Ubuntu 24.04 (Noble), ROS 2 Jazzy


# Base image for ARM (Jetson/aarch64) with Ubuntu 24.04 (Noble) + CUDA
#
# NOTE on base image choice:
#   This CUDA multi-arch image supports arm64 and provides Ubuntu 24.04.
#   On Jetson (nvidia-container-runtime), L4T libraries are automatically
#   mounted from the host at runtime (JetPack 6.x or 7.x).
#
#   If NVIDIA releases an official l4t-pytorch image for Ubuntu 24.04 (JetPack 7.x),
#   it can be substituted here, e.g.:
#     FROM nvcr.io/nvidia/l4t-pytorch:r37.x.x-pth2.x-py3 AS l4t_pytorch_base
#   Advantage: Jetson-optimized PyTorch pre-installed; remove the PyTorch build section below.
FROM nvcr.io/nvidia/cuda:13.2.0-cudnn-devel-ubuntu24.04 AS cuda_arm_base

LABEL maintainer="Lucas Carpentier, Guillaume Dupoiron"

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ARG ROS_DISTRO=jazzy
# Jetson CUDA architecture (cmake format, no dot):
#   87 = Orin (AGX/NX), 80 = Xavier AGX, 72 = Xavier NX/TX2-NX, 53 = Nano (older gen)
# Torch format (with dot): 8.7 = Orin
ARG CUDA_ARCH=87
ARG TORCH_CUDA_ARCH_LIST="8.7"
# Parallel build jobs — reduce if RAM is insufficient (CUDA compile crash)
# Estimated RAM: ~1.5 GB/job -> j4 = ~6 GB
ARG MAKEFLAGS="-j4"

# Python 3 (not present in base CUDA images)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# OpenGL / EGL libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglvnd-dev \
    pkg-config && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Timezone + base tools
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
    ninja-build \
    openssh-client \
    openssh-server \
    python3-ipdb \
    python3-tk \
    sudo \
    terminator \
    unattended-upgrades \
    wget \
    && rm -rf /var/lib/apt/lists/*

# MPI — standard OpenMPI (no HPC-X on ARM/Jetson, unlike x86 NVIDIA images)
RUN apt-get update && apt-get install -y \
    libopenmpi-dev \
    openmpi-bin \
    && rm -rf /var/lib/apt/lists/*

ENV TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Ubuntu 24.04 (PEP 668) blocks pip without this flag.
# Safe inside an isolated Docker container.
ENV PIP_BREAK_SYSTEM_PACKAGES=1

ARG CACHE_DATE=2026-04-09

# ============================================================
# PyTorch — built from source for SM_87 (Jetson Orin, aarch64)
#
# WHY: pytorch.org wheels for linux_aarch64 (manylinux SBSA) do NOT include SM_87.
#   SBSA targets server ARM (Grace Hopper SM_90, Blackwell SM_100+) only.
#   JetPack 7.x (Ubuntu 24.04 / Python 3.12, CUDA 13.x) has no NVIDIA-provided
#   pre-built wheels with SM_87 as of 2026-06. Source build is the only correct path.
#
# NOTE: ~2-3 hours. Override with --build-arg PYTORCH_VERSION=vX.Y.Z
# torchvision: NOT built — unused by curobo_ros and capacitynet.
# ============================================================
ARG PYTORCH_VERSION=v2.12.1

# Build PyTorch from source with SM_87
# Uses pip install (modern path) instead of setup.py bdist_wheel (deprecated in 2.x).
# Env vars are exported so cmake/ninja pick them up through pip's subprocess.
RUN git clone --recursive --depth 1 -b ${PYTORCH_VERSION} \
        https://github.com/pytorch/pytorch /pkgs/pytorch_src && \
    cd /pkgs/pytorch_src && \
    pip3 install -r requirements.txt && \
    export TORCH_CUDA_ARCH_LIST="8.7" && \
    export USE_CUDA=1 && \
    export USE_CUDNN=1 && \
    export USE_MKLDNN=0 && \
    export MAX_JOBS=${MAKEFLAGS##*-j} && \
    pip3 install --no-build-isolation . && \
    rm -rf /pkgs/pytorch_src

# Verification (non-fatal — no GPU in build sandbox)
RUN python3 -c "import torch; print('PyTorch', torch.__version__, '| CUDA:', torch.cuda.is_available())" || \
    echo "Warning: PyTorch check skipped (no GPU in build sandbox — expected)"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libbenchmark-dev libgoogle-glog-dev libgtest-dev libsqlite3-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /pkgs

RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# CUDA samples headers required by CuRobo JIT (helper_math.h, etc.)
RUN git clone --depth 1 https://github.com/NVIDIA/cuda-samples.git /pkgs/cuda-samples && \
    cp /pkgs/cuda-samples/Common/helper_math.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/helper_cuda.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/helper_functions.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/exception.h /usr/local/cuda/include/

# CuRobo v2
ARG CUROBO_REF=v0.8.0
RUN mkdir /pkgs/curobo_src && cd /pkgs/curobo_src && \
    git clone -b ${CUROBO_REF} https://github.com/NVlabs/curobo.git .
WORKDIR /pkgs/curobo_src
# usd-core has no ARM64 wheel on PyPI — usd extra dropped for aarch64
RUN MAX_JOBS=${MAKEFLAGS##*-j} pip3 install .[cu13,dev] --no-build-isolation

# CuRobo v2 internal headers — moved to curobo/_src/curobolib/kernels/{common,...}.
# v2 compiles kernels via NVRTC (cuda_core backend) reading headers from the
# installed package, so this copy is best-effort (belt-and-suspenders for any
# NVRTC include path pointing at /usr/local/cuda/include). Non-fatal.
RUN find /pkgs/curobo_src/curobo/_src/curobolib/kernels \
        \( -name '*.h' -o -name '*.cuh' \) -exec cp {} /usr/local/cuda/include/ \; \
    2>/dev/null || echo "Note: CuRobo v2 headers stay in-package (NVRTC reads them there)."

# Pre-compile CuRobo CUDA extensions (avoids NVRTC compile at first launch).
# v2 compiles kernels via NVRTC at runtime, which needs a CUDA context (GPU).
# A GPU is NOT available during `docker build`, so this is OFF by default and the
# kernels are compiled at first container launch on the Jetson. Kept guarded.
# To enable on a GPU build host: --build-arg PRECOMPILE_CUROBO=true
ARG PRECOMPILE_CUROBO=false
RUN if [ "$PRECOMPILE_CUROBO" = "true" ]; then \
        python3 -c "\
import curobo; \
from curobo._src.curobolib import cuda_ops; \
print(f'CuRobo {curobo.__version__} kernel pre-compilation done')" \
        || echo "Warning: CuRobo pre-compilation failed (no GPU?). Will compile at first launch."; \
    else \
        echo "CuRobo kernel pre-compilation skipped (compiled at first launch on GPU)."; \
    fi

ENV PYOPENGL_PLATFORM=egl

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# googletest — libraries already installed above, only cmake install step needed
RUN cd /usr/src/googletest && cmake . && cmake --build . --target install

RUN python3 -m pip install \
    pyrealsense2 \
    transforms3d

##### Installing ROS Jazzy (Ubuntu 24.04 Noble) ######

ARG DEBIAN_FRONTEND=noninteractive

# GPG issue on Ubuntu 24.04 — apt 2.8.x verifies InRelease files in split sig+data
# mode: BADSIG when mirrors serve a recently rotated signing key.
# AllowInsecureRepositories MUST be passed on the command line
# (apt.conf.d silently ignores it per the official apt documentation).
# Fix: wrapper at /usr/local/sbin/apt-get injected before /usr/bin in PATH.
RUN printf '#!/bin/sh\nexec /usr/bin/apt-get \\\n  -o Acquire::AllowInsecureRepositories=true \\\n  -o Acquire::AllowDowngradeToInsecureRepositories=true \\\n  --allow-unauthenticated "$@"\n' \
    > /usr/local/sbin/apt-get && chmod +x /usr/local/sbin/apt-get

# ROS 2 repository — ros2-apt-source.deb installed via dpkg (handles ROS key without apt)
# gnupg2/lsb-release/curl are already present in the CUDA base image.
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# ROS Jazzy desktop + dependencies
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

# RealSense (if arm64 package is available for jazzy)
RUN apt-get update && apt-get install -y \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    && rm -rf /var/lib/apt/lists/* || \
    echo "Warning: RealSense packages not available for jazzy/arm64, skipping"

WORKDIR /home/ros2_ws/src

RUN sudo rosdep init && rosdep update

# ros2_numpy (jazzy or main branch)
RUN git clone https://github.com/Box-Robotics/ros2_numpy.git || \
    git clone -b main https://github.com/Box-Robotics/ros2_numpy.git

# trajectory_preview
RUN git clone https://github.com/swri-robotics/trajectory_preview.git

# open3d is not used in this workspace and has no pre-built ARM64 wheel — removed.
# libgomp1 kept as it may be needed by other packages.
RUN apt-get update && apt-get install --no-install-recommends -y \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Resolve numpy/scipy conflicts:
# - numpy pinned to 1.26.4 for numba/cupy compatibility
# - apt-installed packages have no RECORD file so --force-reinstall fails;
#   --ignore-installed installs into /usr/local/lib which takes precedence over /usr/lib
RUN pip3 install --no-cache-dir --ignore-installed "numpy==1.26.4" scipy && \
    python3 -m pip install --no-cache-dir --ignore-installed --no-deps \
    pandas scikit-learn pyarrow

# Vérification finale CuRobo v2
RUN python3 -c "import curobo; print(f'cuRobo {curobo.__version__} OK')" || \
    echo "Warning: cuRobo import check skipped (no GPU in build sandbox — expected)"

# Retrait des bibliothèques CUDA forward-compat de l'image de base.
# Inutiles sur Jetson (le driver hôte R39 supporte CUDA 13.2 nativement) et
# elles font planter le hook 'cudacompat' du nvidia-container-toolkit 1.19.1
# (panic: slice bounds out of range) au démarrage avec le runtime nvidia.
# Sans ce nettoyage, le conteneur ne démarre pas en mode GPU.
RUN rm -rf /usr/local/cuda*/compat /usr/local/cuda*/compat_orin 2>/dev/null || true

# Build ROS workspace
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

WORKDIR /home/ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

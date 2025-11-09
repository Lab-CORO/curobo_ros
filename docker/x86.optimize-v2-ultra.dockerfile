# Production Dockerfile - Optimized for using curobo_ros
# This image is smaller and meant for users who want to use curobo_ros without modifying it

FROM nvcr.io/nvidia/pytorch:23.08-py3 AS torch_cuda_base

LABEL maintainer="Lucas Carpentier, Guillaume Dupoiron"
LABEL description="Optimized curobo_ros image for production use"

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ARG ROS_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive

# Add GL libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglvnd-dev \
    pkg-config && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Install essential packages only (minimal set)
RUN apt-get update && apt-get install -y \
    tzdata \
    software-properties-common \
    bash \
    build-essential \
    cmake \
    curl \
    git \
    iputils-ping \
    libssl-dev \
    lsb-core \
    python3-pip \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata

# Install MPI dependencies
RUN apt-get update && apt-get install --reinstall -y \
    hwloc-nox \
    libmpich-dev \
    libmpich12 \
    mpich \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib"

ARG TORCH_CUDA_ARCH_LIST="8.0 8.6"
ENV TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Install cuRobo (production version)
ARG CACHE_DATE=2024-07-19
RUN mkdir /pkgs && cd /pkgs && git clone https://github.com/NVlabs/curobo.git
WORKDIR /pkgs/curobo
RUN pip3 install . --no-build-isolation

# Install nvblox for collision checking
ENV PYOPENGL_PLATFORM=egl
RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN apt-get update && \
    apt-get install -y libbenchmark-dev libgoogle-glog-dev libgtest-dev libsqlite3-dev && \
    cd /usr/src/googletest && cmake . && cmake --build . --target install && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /pkgs
RUN git clone https://github.com/valtsblukis/nvblox.git && \
    cd nvblox && cd nvblox && mkdir build && cd build && \
    cmake .. -DPRE_CXX11_ABI_LINKABLE=ON && \
    make -j32 && make install

RUN git clone https://github.com/Lab-CORO/nvblox_torch.git && \
    cd nvblox_torch && \
    sh install.sh $(python -c 'import torch.utils; print(torch.utils.cmake_prefix_path)') && \
    python3 -m pip install -e .

# Install essential Python packages
RUN python -m pip install \
    pyrealsense2 \
    transforms3d \
    open3d

# Install ROS 2 Humble (minimal)
RUN apt-get update && apt-get install -y \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt update && sudo apt install curl -y && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    sudo apt install /tmp/ros2-apt-source.deb

# Install minimal ROS packages
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf-transformations \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN sudo rosdep init && rosdep update

# Install curobo_ros, curobo_msgs, and curobo_rviz from Git
WORKDIR /opt/curobo_workspace/src
RUN git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules && \
    git clone https://github.com/Lab-CORO/curobo_msgs.git && \
    git clone https://github.com/Lab-CORO/curobo_rviz.git

# Build the packages
WORKDIR /opt/curobo_workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/curobo_workspace/install/setup.bash" >> ~/.bashrc

# Set Cyclone DDS as RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Update UCX path
ENV LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH

# Set default working directory
WORKDIR /home/ros2_ws

# Production image is ready - user will mount their workspace here

FROM nvcr.io/nvidia/pytorch:23.08-py3 AS torch_cuda_base

LABEL maintainer="Lucas Carpentier, Guillaume Dupoiron"

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ARG ROS_DISTRO=humble

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
    glmark2 \
    iputils-ping \
    libeigen3-dev \
    libssl-dev \
    lsb-core \
    make \
    openssh-client \
    openssh-server \
    python3-ipdb \
    python3-pip \
    python3-tk \
    python3-wstool \
    snapd \
    sudo \
    terminator \
    unattended-upgrades \
    wget \
    && rm -rf /var/lib/apt/lists/*


# Téléchargez le fichier tar.gz depuis le site de JetBrains
RUN wget https://download.jetbrains.com/python/pycharm-community-2023.1.2.tar.gz

# Décompressez le fichier
RUN tar -xzf pycharm-community-2023.1.2.tar.gz

# Déplacez le dossier décompressé
RUN mv pycharm-community-2023.1.2 /opt/pycharm-community

# Ajoutez un lien symbolique pour faciliter l'exécution
RUN ln -s /opt/pycharm-community/bin/pycharm.sh /usr/local/bin/pycharm

# push defaults to bashrc:
RUN apt-get update && apt-get install --reinstall -y \
    hwloc-nox \
    libmpich-dev \
    libmpich12 \
    mpich \
    && rm -rf /var/lib/apt/lists/*

# This is required to enable mpi lib access:
ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib"

ENV TORCH_CUDA_ARCH_LIST="6.1 7.0+PTX"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Add cache date to avoid using cached layers older than this
ARG CACHE_DATE=2024-07-19

RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"


# if you want to use a different version of curobo, create folder as docker/pkgs and put your
# version of curobo there. Then uncomment below line and comment the next line that clones from
# github

# COPY pkgs /pkgs
RUN mkdir /pkgs && cd /pkgs && git clone  https://github.com/NVlabs/curobo.git

WORKDIR /pkgs/curobo
RUN pip3 install .[dev,usd] --no-build-isolation

# Optionally install nvblox:

# we require this environment variable to  render images in unit test curobo/tests/nvblox_test.py

ENV PYOPENGL_PLATFORM=egl

# add this file to enable EGL for rendering

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN apt-get update && \
    apt-get install -y libbenchmark-dev libgoogle-glog-dev libgtest-dev libsqlite3-dev && \
    cd /usr/src/googletest && cmake . && cmake --build . --target install && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /pkgs
RUN git clone https://github.com/valtsblukis/nvblox.git && \
    cd nvblox && cd nvblox && mkdir build && cd build && \
    cmake .. -DPRE_CXX11_ABI_LINKABLE=ON && \
    make -j32 && \
    make install
RUN git clone https://github.com/Lab-CORO/nvblox_torch.git && \
    cd nvblox_torch && \
    sh install.sh $(python -c 'import torch.utils; print(torch.utils.cmake_prefix_path)') && \
    python3 -m pip install -e .

#################################################
# Cloner le dépôt OpenCV et les modules supplémentaires

RUN git clone https://github.com/opencv/opencv.git /pkgs/opencv

WORKDIR /pkgs/opencv
RUN mkdir -p build

RUN python -m pip install opencv-python-headless \
    pyrealsense2 \
    transforms3d

# install benchmarks:
RUN python -m pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

RUN export LD_LIBRARY_PATH="/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH"

ADD http://archive.ubuntu.com/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb /tmp/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb
RUN dpkg -i /tmp/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb


##### Installing ROS Humble ######

# Définir des arguments pour désactiver les invites interactives pendant l'installation
ARG DEBIAN_FRONTEND=noninteractive

# Ajouter les dépôts ROS 2
RUN apt-get update && apt-get install -y \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/* \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Installer des dépendances générales
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-nav2-msgs \
    ros-humble-moveit \
    ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# Ajouter les sources de ROS 2 Humble
RUN apt-get update && apt-get install -y software-properties-common && rm -rf /var/lib/apt/lists/*
RUN add-apt-repository universe

RUN sudo apt update && sudo apt install curl -y && \ 
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    sudo apt install /tmp/ros2-apt-source.deb

# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Mettre à jour et installer ROS 2 Humble
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros2_ws/src

RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# Construire les packages un par un pour résoudre les dépendances
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/ros2_ws && colcon build --packages-select curobo_msgs"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/ros2_ws && colcon build"
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN sudo rosdep init # "sudo rosdep init --include-eol-distros" && \
    rosdep update # "sudo rosdep update --include-eol-distros" 

# Setup for trajectory_preview
RUN git clone https://github.com/swri-robotics/trajectory_preview.git

# Setup for curobo_rviz
# RUN git clone https://github.com/Lab-CORO/curobo_rviz.git

# Add tools for pcd_fuse
RUN apt remove python3-blinker -y

# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    python3-pip \
    ros-humble-tf-transformations\
    && rm -rf /var/lib/apt/lists/*

# Install Open3D from the PyPI repositories
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade open3d

# # # Set the workspace directory
WORKDIR /home/ros2_ws/src

# # # Clone the repository directly into the src directory
RUN git clone -b humble https://github.com/Box-Robotics/ros2_numpy.git


# Build workspace
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"

RUN source /opt/ros/"$ROS_DISTRO"/setup.bash && \
    cd /home/ros2_ws && \
    . install/local_setup.bash

WORKDIR /home/ros2_ws

# Fix error: "AttributeError: module 'cv2.dnn' has no attribute 'DictValue'"
RUN sed -i '171d' /usr/local/lib/python3.10/dist-packages/cv2/typing/__init__.py

# update ucx path: https://github.com/openucx/ucc/issues/476
ENV LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH
COPY branch_switch_entrypoint.sh /home/


# Not needed anymore ENTRYPOINT [ "/home/branch_switch_entrypoint.sh" ]


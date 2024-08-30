FROM nvcr.io/nvidia/pytorch:23.08-py3 AS torch_cuda_base

LABEL maintainer "Lucas Carpentier, Guillaume Dupoiron"

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ARG ROS_DISTRO=humble

# add GL:
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libglvnd-dev \
        libgl1-mesa-dev \
        libegl1-mesa-dev \
        libgles2-mesa-dev && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute


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
  curl \
  lsb-core \
  wget \
  build-essential \
  cmake \
  snapd \
  git \
  git-lfs \
  iputils-ping \
  make \
  openssh-server \
  openssh-client \
  libeigen3-dev \
  libssl-dev \
  python3-pip \
  python3-ipdb \
  python3-tk \
  python3-wstool \
  sudo git bash unattended-upgrades \
  apt-utils \
  terminator \
  glmark2 \
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
  libmpich-dev \
  hwloc-nox libmpich12 mpich \
  && rm -rf /var/lib/apt/lists/*

# This is required to enable mpi lib access:
ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib"



ENV TORCH_CUDA_ARCH_LIST "7.0+PTX"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Add cache date to avoid using cached layers older than this
ARG CACHE_DATE=2024-07-19


RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# if you want to use a different version of curobo, create folder as docker/pkgs and put your
# version of curobo there. Then uncomment below line and comment the next line that clones from
# github

# COPY pkgs /pkgs
RUN mkdir /pkgs && cd /pkgs && git clone  https://github.com/NVlabs/curobo.git

RUN cd /pkgs/curobo && pip3 install .[dev,usd] --no-build-isolation

WORKDIR /pkgs/curobo

# Optionally install nvblox:

# we require this environment variable to  render images in unit test curobo/tests/nvblox_test.py

ENV PYOPENGL_PLATFORM=egl

# add this file to enable EGL for rendering

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN apt-get update && \
    apt-get install -y libgoogle-glog-dev libgtest-dev curl libsqlite3-dev libbenchmark-dev && \
    cd /usr/src/googletest && cmake . && cmake --build . --target install && \
    rm -rf /var/lib/apt/lists/*

RUN cd /pkgs &&  git clone https://github.com/valtsblukis/nvblox.git && \
    cd nvblox && cd nvblox && mkdir build && cd build && \
    cmake .. -DPRE_CXX11_ABI_LINKABLE=ON && \
    make -j32 && \
    make install

RUN cd /pkgs && git clone https://github.com/Lab-CORO/nvblox_torch.git && \
    cd nvblox_torch && \
    sh install.sh $(python -c 'import torch.utils; print(torch.utils.cmake_prefix_path)') && \
    python3 -m pip install -e .

#################################################
# Cloner le dépôt OpenCV et les modules supplémentaires
WORKDIR /pkgs

RUN git clone https://github.com/Lab-CORO/curobo_doosan.git 

RUN git clone https://github.com/opencv/opencv.git /pkgs/opencv

WORKDIR /home/ros2_ws/src

RUN git clone https://github.com/Lab-CORO/CapacitiNet_msg.git src/CapacitiNet_msg && git clone https://github.com/Lab-CORO/curobo_ros.git && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

WORKDIR /pkgs/opencv
RUN mkdir -p build

RUN python -m pip install pyrealsense2 \
    opencv-python-headless \
    transforms3d

# install benchmarks:
RUN python -m pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

RUN export LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH

RUN wget 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64' -O /tmp/code_latest_amd64.deb && \
    sudo dpkg -i /tmp/code_latest_amd64.deb

#alias for code:

RUN echo "alias code='code --user-data-dir=./vscode --no-sandbox'" >> /root/.bashrc
RUN wget 'http://archive.ubuntu.com/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb' -O /tmp/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb && \
    dpkg -i /tmp/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb
    
COPY script_python/. /pkgs/curobo/examples/isaac_sim

RUN sudo mkdir -p /etc/apt/keyrings && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update && apt-get install -y librealsense2-dkms \
    librealsense2-utils\
    librealsense2-dev\
    librealsense2-dbg 



##### Installing ROS Humble ######

# Définir des arguments pour désactiver les invites interactives pendant l'installation
ARG DEBIAN_FRONTEND=noninteractive

# Ajouter les dépôts ROS 2
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update

# Installer des dépendances générales
RUN apt-get update && apt-get install -y \
    lsb-release \
    python3-rosdep \
    gnupg2 \
    curl \
    python3-pip \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Ajouter les sources de ROS 2 Humble
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository universe
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Mettre à jour et installer ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-argcomplete \
    ros-humble-rviz2 \
    ros-humble-pcl-ros \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY ros_packages/. /home/ros2_ws/src

# Construire les packages un par un pour résoudre les dépendances
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/ros2_ws && colcon build --packages-select capacinet_msg"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/ros2_ws && colcon build"
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN sudo rosdep init # "sudo rosdep init --include-eol-distros" && \
         rosdep update # "sudo rosdep update --include-eol-distros" && \
         rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /home/ros2_ws && \
    . install/local_setup.bash

WORKDIR /home/ros2_ws
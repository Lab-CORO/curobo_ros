FROM nvcr.io/nvidia/pytorch:23.08-py3

LABEL maintainer="Lucas Carpentier, Guillaume Dupoiron"

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ARG ROS_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive

# Combine GL libraries installation
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglvnd-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Install system dependencies in one layer with cleanup
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    software-properties-common \
    apt-utils \
    bash \
    build-essential \
    cmake \
    curl \
    git \
    git-lfs \
    iputils-ping \
    libeigen3-dev \
    libssl-dev \
    lsb-core \
    make \
    python3-ipdb \
    python3-pip \
    python3-tk \
    python3-wstool \
    wget \
    hwloc-nox \
    libmpich-dev \
    libmpich12 \
    mpich \
    libbenchmark-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libsqlite3-dev \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
    && echo "America/New_York" > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata \
    && add-apt-repository -y ppa:git-core/ppa \
    && apt-get update && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

# Environment variables
ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib:/usr/local/lib:/opt/hpcx/ucx/lib"
ARG TORCH_CUDA_ARCH_LIST="6.1 7.0+PTX"
ENV TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST
ARG CACHE_DATE=2024-07-19
ENV PYOPENGL_PLATFORM=egl

# Install Python packages with no-cache
RUN pip install --no-cache-dir \
    "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git" \
    pyrealsense2 \
    transforms3d \
    numpy \
    scipy

# Build and install curobo, then cleanup
RUN mkdir /pkgs && cd /pkgs \
    && git clone --depth 1 https://github.com/NVlabs/curobo.git \
    && cd curobo \
    && pip3 install --no-cache-dir .[dev,usd] --no-build-isolation \
    && rm -rf /pkgs/curobo/.git \
    && find /pkgs/curobo -type f -name "*.pyc" -delete \
    && find /pkgs/curobo -type d -name "__pycache__" -delete

# Add EGL config
RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Build googletest and cleanup source
RUN cd /usr/src/googletest \
    && cmake . \
    && cmake --build . --target install \
    && cd / \
    && rm -rf /usr/src/googletest

# Build nvblox with aggressive cleanup (using specific compatible commit)
WORKDIR /pkgs
RUN git clone https://github.com/valtsblukis/nvblox.git \
    && cd nvblox \
    && git checkout 7bda93e \
    && rm -rf .git \
    && cd nvblox \
    && mkdir build && cd build \
    && cmake .. -DPRE_CXX11_ABI_LINKABLE=ON \
    && make -j$(nproc) \
    && make install \
    && cd /pkgs \
    && rm -rf nvblox

# Build nvblox_torch and cleanup (using specific compatible commit)
RUN git clone https://github.com/Lab-CORO/nvblox_torch.git \
    && cd nvblox_torch \
    && git checkout 4ddee1e \
    && sh install.sh $(python -c 'import torch.utils; print(torch.utils.cmake_prefix_path)') \
    && python3 -m pip install --no-cache-dir -e . \
    && rm -rf .git \
    && find /pkgs/nvblox_torch -name "*.o" -delete \
    && find /pkgs/nvblox_torch -name "*.a" -delete \
    && rm -rf /pkgs/nvblox_torch/build

# Install libusb and cleanup
ADD http://archive.ubuntu.com/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb /tmp/libusb.deb
RUN dpkg -i /tmp/libusb.deb && rm /tmp/libusb.deb

##### Installing ROS Humble ######

# Add ROS 2 repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && add-apt-repository universe

# Install ROS apt source
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# Install ROS 2 packages in one go
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-desktop \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-nav2-msgs \
    ros-humble-moveit \
    ros-humble-realsense2-* \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    libegl1 \
    libgl1 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Remove conflicting package
RUN apt-get remove -y python3-blinker || true

# Install Open3D and other Python packages
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
    open3d \
    pandas \
    scikit-learn \
    pyarrow

# Clone ROS workspaces with shallow clones
WORKDIR /home/ros2_ws/src
RUN git clone --depth 1 https://github.com/IntelRealSense/realsense-ros.git -b ros2-master \
    && git clone --depth 1 https://github.com/Lab-CORO/curobo_msgs.git \
    && git clone --depth 1 https://github.com/swri-robotics/trajectory_preview.git \
    && git clone --depth 1 https://github.com/Lab-CORO/curobo_rviz.git \
    && git clone --depth 1 --recurse-submodules https://github.com/Lab-CORO/curobo_ros.git \
    && git clone --depth 1 -b humble https://github.com/Box-Robotics/ros2_numpy.git \
    && find . -name ".git" -exec rm -rf {} + 2>/dev/null || true

# Initialize rosdep
RUN rosdep init && rosdep update

# Build ROS workspace and cleanup build artifacts
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log"

# Setup bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

# Fix cv2.dnn error
RUN sed -i '171d' /usr/local/lib/python3.10/dist-packages/cv2/typing/__init__.py 2>/dev/null || true

WORKDIR /home/ros2_ws

ENV LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ULTRA AGGRESSIVE cleanup to reduce image size
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* ~/.cache/pip ~/.cache/huggingface \
    # Remove all .git directories
    && find / -name ".git" -type d -exec rm -rf {} + 2>/dev/null || true \
    # Remove Python cache everywhere
    && find / -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true \
    && find / -type f -name "*.pyc" -delete 2>/dev/null || true \
    && find / -type f -name "*.pyo" -delete 2>/dev/null || true \
    # Remove ALL locales except English (more aggressive)
    && find /usr/share/locale -mindepth 1 -maxdepth 1 ! -name 'en*' -exec rm -rf {} + \
    && find /usr/share/i18n/locales -mindepth 1 -maxdepth 1 ! -name 'en_*' -exec rm -rf {} + 2>/dev/null || true \
    # Remove docs, man pages, info files
    && rm -rf /usr/share/doc/* /usr/share/man/* /usr/share/groff/* /usr/share/info/* \
    # Remove all build artifacts (.o, .a files)
    && find /pkgs -type f -name "*.o" -delete 2>/dev/null || true \
    && find /pkgs -type f -name "*.a" -delete 2>/dev/null || true \
    && find /home/ros2_ws -type f -name "*.o" -delete 2>/dev/null || true \
    && find /usr -type f -name "*.a" ! -path "*/gcc/*" -delete 2>/dev/null || true \
    # Remove development headers not needed at runtime
    && rm -rf /usr/include/eigen3/unsupported/test \
    # Remove CUDA samples and docs (HUGE space saver ~1-2GB)
    && rm -rf /usr/local/cuda/samples /usr/local/cuda/doc \
    # Remove CUDA static libraries (save ~500MB-1GB, keep only .so)
    && find /usr/local/cuda -name "*.a" -delete 2>/dev/null || true \
    # Remove PyTorch test files and examples
    && find /usr/local/lib/python3.10/dist-packages/torch -name "test" -type d -exec rm -rf {} + 2>/dev/null || true \
    && find /usr/local/lib/python3.10/dist-packages/torch -name "*test*.py" -delete 2>/dev/null || true \
    # Remove debug symbols
    && find / -name "*.debug" -delete 2>/dev/null || true \
    && find / -name ".debug" -type d -exec rm -rf {} + 2>/dev/null || true \
    # Remove benchmark and example files
    && find / -name "benchmark" -type d -path "*/share/*" -exec rm -rf {} + 2>/dev/null || true \
    && find / -name "examples" -type d -path "*/share/*" -exec rm -rf {} + 2>/dev/null || true \
    # Clean pip cache again
    && pip cache purge 2>/dev/null || true \
    # Remove opencv clone directory if it exists
    && rm -rf /pkgs/opencv \
    # Remove apt cache and autoremove
    && apt-get autoremove -y \
    && apt-get autoclean \
    # Remove CMake cache files
    && find / -name "CMakeCache.txt" -delete 2>/dev/null || true \
    && find / -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true \
    # Remove git lfs cache
    && git lfs prune 2>/dev/null || true \
    # Remove log files
    && find /var/log -type f -delete 2>/dev/null || true \
    # Remove systemd journal if present
    && rm -rf /var/log/journal/* 2>/dev/null || true

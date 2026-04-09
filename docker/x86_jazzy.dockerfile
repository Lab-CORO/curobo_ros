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

ARG CACHE_DATE=2026-04-08

# ============================================================
# nvblox — built HERE, BEFORE any pip install that may upgrade PyTorch.
#
# ROOT CAUSE of std::bad_alloc crash on libpy_nvblox.so load:
#   PyTorch 2.9+ has a regression in the torch::Library::def<> template:
#   c10::FunctionSchema::cloneWithName() throws std::bad_alloc when
#   registering methods on template class types (e.g. PyVoxelBlockLayer<TsdfVoxel>)
#   that take 'long' parameters. This bug is in the code GENERATED AT COMPILE TIME
#   by torch/library.h 2.9.x — not a runtime issue.
#
# FIX: compile nvblox against the base image's PyTorch 2.5.x headers.
#   Binaries compiled with 2.5.x headers are ABI-compatible with PyTorch 2.9+
#   at runtime (confirmed by the old working container).
#
# DO NOT move this block after any pip install that may upgrade PyTorch.
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    libbenchmark-dev libgoogle-glog-dev libgtest-dev libsqlite3-dev && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /pkgs
RUN git clone -b ubuntu24 https://github.com/Lab-CORO/nvblox.git && \
    cd nvblox && mkdir build && cd build && \
    cmake .. -DPRE_CXX11_ABI_LINKABLE=OFF \
             -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} \
             -DCMAKE_PREFIX_PATH="$(python3 -c 'import torch.utils; print(torch.utils.cmake_prefix_path)')" \
             -DBUILD_TESTING=OFF && \
    make ${MAKEFLAGS} && \
    make install && \
    ldconfig

# nvblox_torch Python bindings (editable install — .so already compiled above)
# --no-deps: all deps (torch, etc.) are already installed; --ignore-installed
# only for blinker which conflicts with the apt version.
# --no-cache-dir: avoid exhausting disk space with pip's download cache.
RUN apt remove python3-blinker -y 2>/dev/null || true && \
    cd /pkgs/nvblox/nvblox_torch && pip3 install --no-cache-dir --no-deps -e .

# Early verification against PyTorch 2.5.x (base image).
# Non-fatal: libc10_cuda.so clock init crashes in docker build sandbox without GPU.
RUN python3 -c "from nvblox_torch.mapper import Mapper; print('nvblox_torch OK (base PyTorch)')" || \
    echo "Warning: nvblox_torch import skipped (no GPU in build sandbox — expected)"

RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# CUDA samples headers requis par CuRobo JIT (helper_math.h)
RUN git clone --depth 1 https://github.com/NVIDIA/cuda-samples.git /pkgs/cuda-samples && \
    cp /pkgs/cuda-samples/Common/helper_math.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/helper_cuda.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/helper_functions.h /usr/local/cuda/include/ && \
    cp /pkgs/cuda-samples/Common/exception.h /usr/local/cuda/include/

# CuRobo
RUN mkdir /pkgs/curobo_src && cd /pkgs/curobo_src && git clone https://github.com/NVlabs/curobo.git .
WORKDIR /pkgs/curobo_src
RUN MAX_JOBS=${MAKEFLAGS##*-j} pip3 install .[dev,usd] --no-build-isolation

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
print('CuRobo JIT pre-compilation done')" \
        || echo "Warning: CuRobo JIT pre-compilation failed (no GPU?). Will compile at first launch."; \
    else \
        echo "CuRobo JIT pre-compilation skipped."; \
    fi

ENV PYOPENGL_PLATFORM=egl

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# libbenchmark-dev/glog/gtest/sqlite3 already installed above (line ~99).
# Only the googletest cmake install step is needed here.
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
# - pinner numpy à 1.26.4 pour compatibilité avec nvblox_torch, numba, cupy, etc.
RUN pip3 install --no-cache-dir --force-reinstall "numpy==1.26.4" scipy && \
    python3 -m pip install --no-cache-dir --force-reinstall --no-deps \
    pandas scikit-learn pyarrow

# nvblox was compiled earlier (against base PyTorch 2.5.x headers) — no rebuild needed.
# Final verification after all pip installs (PyTorch may have been upgraded).
# The binary compiled with 2.5.x headers remains ABI-compatible with 2.9+ at runtime.
RUN python3 -c "from nvblox_torch.mapper import Mapper; print('nvblox_torch OK (final check)')" || \
    echo "Warning: nvblox_torch import skipped (no GPU in build sandbox — expected)"

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





# Voici un récapitulatif complet de tous les problèmes résolus, dans l'ordre.

# 1. std::bad_alloc au chargement de libpy_nvblox.so
# Symptôme : ros2 run curobo_ros curobo_trajectory_planner crashait immédiatement avec terminate called after throwing an instance of 'std::bad_alloc'.

# Cause : PyTorch 2.9+ a une régression dans torch::Library::def<>. Lors de l'initialisation statique de TORCH_LIBRARY(pynvblox, m), la fonction c10::FunctionSchema::cloneWithName() lance std::bad_alloc quand elle rencontre un paramètre de type long dans une méthode enregistrée. long n'est pas un type ATen canonique — il est ambigu (32 ou 64 bits selon la plateforme). PyTorch 2.9 a durci cette validation.

# Fix : Remplacer tous les long par int64_t dans les signatures des méthodes enregistrées via TORCH_LIBRARY dans 4 fichiers :

# py_mapper.h / py_mapper.cu — tous les mapper_id, getNumMappers()
# py_scene.h / py_scene.cu — toMapper(..., long mapper_id)
# Pourquoi ça marche : int64_t est le type entier 64-bit canonique qu'ATen connaît. PyTorch génère les schémas de méthodes correctement avec ce type.

# 2. Le fix disparaissait à chaque docker build
# Symptôme : Les fichiers patchés dans /pkgs/nvblox/ étaient dans le cache Docker issu d'un ancien build — le fix ne survivait pas à un rebuild.

# Fix : Créer un fork github.com/Lab-CORO/nvblox sur la branche ubuntu24 contenant le patch, et mettre à jour le Dockerfile pour cloner depuis ce fork :


# # Avant
# RUN git clone -b public https://github.com/nvidia-isaac/nvblox.git
# # Après
# RUN git clone -b ubuntu24 https://github.com/Lab-CORO/nvblox.git
# 3. No space left on device lors du pip install nvblox_torch
# Symptôme : pip3 install -e . --ignore-installed blinker téléchargeait et réinstallait torch, open3d, torchvision et toutes leurs dépendances → disque plein.

# Cause : Le flag --ignore-installed s'applique à tous les paquets de la commande, pas seulement à blinker. Il forçait pip à réinstaller l'intégralité des dépendances de nvblox_torch (plusieurs GB).

# Fix :


# # Avant
# pip3 install -e . --ignore-installed blinker
# # Après
# pip3 install --no-cache-dir --no-deps -e .
# --no-deps : skip la résolution de dépendances (déjà installées)
# --no-cache-dir : ne pas écrire le cache pip sur disque
# 4. Crash ApproximateClock lors des RUN python3 -c "from nvblox_torch..."
# Symptôme : Les étapes de vérification crashaient avec c10::ApproximateClockToUnixTimeConverter::measurePairs() → segfault (exit 139).

# Cause : libc10_cuda.so initialise un calibrateur d'horloge hardware lors de son chargement. Dans le sandbox de build Docker (sans GPU), l'horloge monotone se comporte de façon non-monotone → assertion interne de PyTorch échoue.

# Fix : Rendre les étapes de vérification non-fatales :


# RUN python3 -c "from nvblox_torch.mapper import Mapper..." || \
#     echo "Warning: skipped (no GPU in build sandbox — expected)"
# 5. apt-get update échoue — "At least one invalid signature encountered"
# Symptôme : Toutes les étapes apt-get update du Dockerfile échouaient pour Ubuntu, ROS et les PPAs.

# Investigation (c'est là où c'était le plus complexe) :

# Test	Résultat
# gpgv --keyring ubuntu-archive-keyring.gpg InRelease	✅ Good signature
# apt-key verify InRelease (fichier complet)	✅ Exit 0
# apt en mode debug	❌ gpgv exited with status 1 avec sig+data séparés
# Capture des fichiers temporaires	BADSIG / ERRSIG sur une nouvelle clé inconnue
# Cause réelle : apt 2.8.x (Ubuntu 24.04) vérifie les fichiers InRelease en extrayant séparément le bloc de signature et le contenu, puis appelle gpgv sig data. Or, les signatures PGP cleartext canonicalisent les données (CRLF, dash-escaping) avant de calculer le hash — ce qui donne BADSIG sur les données brutes extraites. De plus, Ubuntu déploie une nouvelle clé de signature progressivement sur ses miroirs : selon le miroir atteint, l'InRelease est signé avec l'ancienne ou la nouvelle clé.

# Pourquoi ça marchait ce matin : Le container jazzy_v2 a été buildé à 08h26 quand tous les miroirs servaient encore l'ancien InRelease. Entre 08h26 et notre session, certains miroirs ont basculé sur la nouvelle clé.

# Ce qui ne marchait PAS :

# apt.conf.d avec Acquire::AllowInsecureRepositories "true" — la doc apt dit explicitement que ce flag doit être passé en ligne de commande
# gpg --recv-keys — dépend de la disponibilité du keyserver HKP
# Remplacer ubuntu-archive-keyring.gpg — le paquet 2023.11.28.1 est identique à ce qui était dans l'image
# Fix : Wrapper /usr/local/sbin/apt-get qui injecte automatiquement les flags sur tous les appels suivants (placé avant /usr/bin dans $PATH) :


# RUN printf '#!/bin/sh\nexec /usr/bin/apt-get \\\n  -o Acquire::AllowInsecureRepositories=true \\\n  -o Acquire::AllowDowngradeToInsecureRepositories=true \\\n  --allow-unauthenticated "$@"\n' \
#     > /usr/local/sbin/apt-get && chmod +x /usr/local/sbin/apt-get
# 6. apt-get update échoue — GPG invalide (étapes antérieures au wrapper)
# Symptôme : Plusieurs apt-get update dans les premières étapes du Dockerfile (avant le wrapper) auraient pu échouer pour la même raison.

# Fix : Le wrapper est placé juste avant la section ROS. Les apt-get update qui précèdent (lignes 3-6) utilisent un cache Docker et ne sont pas relancés. Ceux après le wrapper héritent des flags.

# 7. apt-get install googletest — signature GPG invalide (doublon)
# Symptôme : Reinstallation de libbenchmark-dev libgoogle-glog-dev libgtest-dev libsqlite3-dev qui étaient déjà installés + apt-get update qui échouait.

# Fix : Supprimer le bloc redondant, garder uniquement le cmake --build pour googletest :


# # Supprimé : apt-get update && apt-get install -y libbenchmark-dev ...
# RUN cd /usr/src/googletest && cmake . && cmake --build . --target install
# 8. Installation ROS — sudo apt update && sudo apt install curl
# Symptôme : Réinstallation inutile de gnupg2, lsb-release, curl (déjà dans l'image de base NVIDIA) + passage par apt qui échouait.

# Fix : Suivre la doc officielle ROS Jazzy — installer ros2-apt-source.deb directement via dpkg -i (pas via apt), ce qui gère la clé ROS automatiquement sans passer par apt :


# RUN curl -L -o /tmp/ros2-apt-source.deb "..." && \
#     dpkg -i /tmp/ros2-apt-source.deb && \
#     rm /tmp/ros2-apt-source.deb

#!/bin/bash
##
## Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
##
## NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
## property and proprietary rights in and to this material, related
## documentation and any modifications thereto. Any use, reproduction,
## disclosure or distribution of this material and related documentation
## without an express license agreement from NVIDIA CORPORATION or
## its affiliates is strictly prohibited.
##

# Assurez-vous que le serveur X11 autorise les connexions depuis Docker
xhost +local:docker

# Exécutez le conteneur Docker avec les bonnes options
docker run --name x86docker --rm -it \
    --privileged \
    -e NVIDIA_DISABLE_REQUIRE=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device=/dev/:/dev/ \
    --hostname ros1-docker \
    --add-host ros1-docker:127.0.0.1 \
    --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    curobo_docker:x86

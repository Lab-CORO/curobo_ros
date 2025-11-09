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

# Check if the branch argument is empty
echo "Choose your GPU card model :"
echo "1) RTX 30XX"
echo "2) RTX 40XX"
echo "3) A100"
read -p "Enter the number corresponding to your model: " choice

# Définir TORCH_CUDA_ARCH_LIST en fonction du choix de l'utilisateur
case $choice in
    1)
        TORCH_CUDA_ARCH_LIST="8.0 8.6"  # Pour RTX 30XX
        image_tag="rtx30xxx"
        ;;
    2)
        TORCH_CUDA_ARCH_LIST="8.9 9.0"  # Pour RTX 40XX
        image_tag="rtx40xxx"
        ;;
    3)
        TORCH_CUDA_ARCH_LIST="8.0"      # Pour A100
        image_tag="A100"
        ;;
    *)
        echo "Choix invalide, utilisant la valeur par défaut pour RTX 30XX"
        TORCH_CUDA_ARCH_LIST="8.0 8.6"
        image_tag="rtx30xxx"
        ;;
esac



if [ -z "$1" ]; then
    echo "Branch argument empty, sending default branch: main"
    branch_arg="main"
else
    branch_arg="$1"
fi

if ! [[ "$OSTYPE" == "msys" ]]; then
    # Assurez-vous que le serveur X11 autorise les connexions depuis Docker
    xhost +local:docker

    # Exécutez le conteneur Docker avec les bonnes options
    docker run --name x86docker -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        --device=/dev/:/dev/ \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        curobo-ros:optimized-v2-ultra
else
    echo "Detected OS is msys, make sure to have an X server running on your host machine"
    # Exécutez seulement le conteneur Docker avec les options appropriées
    docker run --name x86docker --rm -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --hostname ros1-docker \
        --add-host ros1-docker:127.0.0.1 \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        curobo_docker:${image_tag}
fi

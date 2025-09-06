#!/bin/bash

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

docker build --build-arg TORCH_CUDA_ARCH_LIST="$TORCH_CUDA_ARCH_LIST" -t curobo_docker:${image_tag} -f x86.dockerfile . 

echo "Construction de l'image Docker terminée avec TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST"

# build docker file:
# Make sure you enable nvidia runtime by:
# Edit/create the /etc/docker/daemon.json with content:
# {
#    "runtimes": {
#        "nvidia": {
#            "path": "/usr/bin/nvidia-container-runtime",
#            "runtimeArgs": []
#         } 
#    },
#    "default-runtime": "nvidia" # ADD this line (the above lines will already exist in your json file)
# }

#!/bin/bash

##
## Build script for curobo_ros Docker images
## Supports DEV (development) and PROD (production) modes
##
## Usage:
##   Interactive mode:  ./build_docker.sh
##   Automated mode:    ./build_docker.sh <gpu_choice> <mode_choice>
##
## Arguments:
##   gpu_choice:   1=Ampere, 2=Ada Lovelace, 3=Turing, 4=Volta, 5=Jetson Orin (aarch64)
##   mode_choice:  1=DEV, 2=PROD
##
## Examples:
##   ./build_docker.sh 1 1    # Ampere GPU, DEV mode
##   ./build_docker.sh 2 2    # Ada Lovelace GPU, PROD mode
##   ./build_docker.sh 5 1    # Jetson Orin (aarch64)
##
## Jetson note: both aarch64_jazzy*.dockerfile (DEV/PROD) install PyTorch from
## a pre-built SM_87 wheel (no pytorch.org wheel covers aarch64+SM_87+cp312+
## CUDA13 — a from-source build takes ~3h). This script looks for that wheel
## in /data/wheels/ and refuses to build if it's missing — see
## /data/wheels/README.md for the one-time build recipe.

set -e  # Exit on error

echo "======================================"
echo "  curobo_ros Docker Build Script"
echo "======================================"
echo ""

# Check if arguments are provided
if [ $# -eq 2 ]; then
    gpu_choice=$1
    mode_choice=$2
    echo "Using command-line arguments:"
    echo "  GPU choice: $gpu_choice"
    echo "  Mode choice: $mode_choice"
    echo ""
else
    # Step 1: Choose GPU architecture
    echo "Step 1/2: Choose your GPU architecture"
    echo "---------------------------------------"
    echo "1) Ampere (RTX 30XX series: 3060, 3070, 3080, 3090, A100)"
    echo "2) Ada Lovelace (RTX 40XX series: 4060, 4070, 4080, 4090)"
    echo "3) Turing (RTX 20XX series: 2060, 2070, 2080)"
    echo "4) Volta (Titan V, V100)"
    echo "5) Jetson Orin (AGX/NX, aarch64, SM_87)"
    echo ""
    read -p "Enter the number corresponding to your GPU: " gpu_choice
fi

# Set CUDA architecture based on choice
case $gpu_choice in
    1)
        TORCH_CUDA_ARCH_LIST="8.0 8.6"
        GPU_NAME="ampere"
        echo "Selected: Ampere (Compute Capability 8.0, 8.6)"
        ;;
    2)
        TORCH_CUDA_ARCH_LIST="8.9 9.0"
        GPU_NAME="ada_lovelace"
        echo "Selected: Ada Lovelace (Compute Capability 8.9, 9.0)"
        ;;
    3)
        TORCH_CUDA_ARCH_LIST="7.5"
        GPU_NAME="turing"
        echo "Selected: Turing (Compute Capability 7.5)"
        ;;
    4)
        TORCH_CUDA_ARCH_LIST="7.0"
        GPU_NAME="volta"
        echo "Selected: Volta (Compute Capability 7.0)"
        ;;
    5)
        TORCH_CUDA_ARCH_LIST="8.7"
        GPU_NAME="jetson"
        IS_JETSON=1
        echo "Selected: Jetson Orin (Compute Capability 8.7)"
        ;;
    *)
        echo "Invalid choice, defaulting to Ampere (RTX 30XX)"
        TORCH_CUDA_ARCH_LIST="8.0 8.6"
        GPU_NAME="ampere"
        ;;
esac

echo ""

# Step 2: Choose build mode (only if not provided as argument)
if [ $# -ne 2 ]; then
    echo "Step 2/2: Choose build mode"
    echo "-----------------------------"
    echo "1) DEV  - Development mode (for modifying curobo_ros internals)"
    echo "            → Full image with development tools"
    echo "            → Workspace mounted from host for live editing"
    echo "            → Size: ~25-30 GB"
    echo ""
    echo "2) PROD - Production mode (for using curobo_ros)"
    echo "            → Optimized image with curobo_ros pre-installed"
    echo "            → Mount your own workspace to use the package"
    echo "            → Size: ~15-20 GB (smaller)"
    echo ""
    read -p "Enter your choice (1 for DEV, 2 for PROD): " mode_choice
fi

# Jetson has its own pair of Dockerfiles (aarch64_jazzy*.dockerfile) — the
# DEV tag stays "curobo_ros:aarch64-jazzy" unversioned by GPU_NAME because
# leeloo_docker and capacitynet build FROM that exact tag.
case $mode_choice in
    1)
        BUILD_MODE="dev"
        if [ -n "$IS_JETSON" ]; then
            DOCKERFILE="aarch64_jazzy.dockerfile"
            IMAGE_TAG="curobo_ros:aarch64-jazzy"
        else
            DOCKERFILE="x86_jazzy.dockerfile"
            IMAGE_TAG="curobo_ros:${GPU_NAME}-dev"
        fi
        echo "Selected: DEV mode"
        echo "Building development image with full tools..."
        ;;
    2)
        BUILD_MODE="prod"
        if [ -n "$IS_JETSON" ]; then
            DOCKERFILE="aarch64_jazzy_prod.dockerfile"
            IMAGE_TAG="curobo_ros:aarch64-jazzy-prod"
        else
            DOCKERFILE="x86_jazzy_prod.dockerfile"
            IMAGE_TAG="curobo_ros:${GPU_NAME}-prod"
        fi
        echo "Selected: PROD mode"
        echo "Building optimized production image..."
        ;;
    *)
        echo "Invalid choice, defaulting to DEV mode"
        BUILD_MODE="dev"
        if [ -n "$IS_JETSON" ]; then
            DOCKERFILE="aarch64_jazzy.dockerfile"
            IMAGE_TAG="curobo_ros:aarch64-jazzy"
        else
            DOCKERFILE="x86_jazzy.dockerfile"
            IMAGE_TAG="curobo_ros:${GPU_NAME}-dev"
        fi
        ;;
esac

echo ""

if [ -n "$IS_JETSON" ]; then
    WHEEL_DIR="/data/wheels"
    WHEEL_PATH=$(ls "$WHEEL_DIR"/torch-*-cp312-cp312-linux_aarch64.whl 2>/dev/null | head -1)
    if [ -z "$WHEEL_PATH" ]; then
        echo "ERROR: no pre-built PyTorch wheel found in $WHEEL_DIR" >&2
        echo "  $DOCKERFILE installs PyTorch from a wheel bind-mounted from" >&2
        echo "  $WHEEL_DIR — no pytorch.org wheel covers aarch64+SM_87+cp312+CUDA13," >&2
        echo "  so building one from source (~3h) is a one-time manual step." >&2
        echo "  See $WHEEL_DIR/README.md for the build recipe, then re-run this script." >&2
        exit 1
    fi
    WHEEL_FILENAME=$(basename "$WHEEL_PATH")
    echo "Found PyTorch wheel: $WHEEL_FILENAME"

    # torchgen is NOT inside the wheel but `import torch` needs it — see the
    # long comment in the Dockerfile. Checked here so a missing tarball fails
    # now, not 20 minutes into the build.
    TORCHGEN_PATH=$(ls "$WHEEL_DIR"/torchgen-*.tar.gz 2>/dev/null | head -1)
    if [ -z "$TORCHGEN_PATH" ]; then
        echo "ERROR: no torchgen tarball found in $WHEEL_DIR" >&2
        echo "  The PyTorch wheel does not bundle torchgen, and torch 2.12 fails" >&2
        echo "  at 'import torch' without it. Recover it from an image that has a" >&2
        echo "  working torch of the same commit, e.g.:" >&2
        echo "    docker run --rm -v $WHEEL_DIR:/out --entrypoint bash <image> -c \\" >&2
        echo "      'cd /usr/local/lib/python3.12/dist-packages && \\" >&2
        echo "       tar czf /out/torchgen-<version>.tar.gz --exclude=__pycache__ torchgen'" >&2
        exit 1
    fi
    TORCHGEN_FILENAME=$(basename "$TORCHGEN_PATH")
    echo "Found torchgen tarball: $TORCHGEN_FILENAME"
fi

echo ""
echo "======================================"
echo "  Build Configuration"
echo "======================================"
echo "GPU Architecture:  $GPU_NAME"
echo "CUDA Arch List:    $TORCH_CUDA_ARCH_LIST"
echo "Build Mode:        $BUILD_MODE"
echo "Dockerfile:        $DOCKERFILE"
echo "Image Tag:         $IMAGE_TAG"
echo ""
if [ -n "$IS_JETSON" ]; then
    echo "⚠️  Note: This build compiles curobo's CUDA kernels (~20-30 min) and requires"
    echo "   ~30 GB disk space. PyTorch itself comes from the pre-built wheel above"
    echo "   ($WHEEL_FILENAME), skipping the ~3h from-source build."
else
    echo "⚠️  Note: This build will take 20-30 minutes and require ~30 GB disk space during build."
fi
echo ""

# Build the Docker image
echo ""
echo "Building Docker image..."
if [ -n "$IS_JETSON" ]; then
    docker build \
        --build-context wheels="$WHEEL_DIR" \
        --build-arg TORCH_WHEEL_FILENAME="$WHEEL_FILENAME" \
        --build-arg TORCHGEN_TARBALL_FILENAME="$TORCHGEN_FILENAME" \
        --build-arg CUDA_ARCH=87 \
        --build-arg TORCH_CUDA_ARCH_LIST="$TORCH_CUDA_ARCH_LIST" \
        -t "$IMAGE_TAG" \
        -f "$DOCKERFILE" \
        .
else
    docker build \
        --build-arg TORCH_CUDA_ARCH_LIST="$TORCH_CUDA_ARCH_LIST" \
        -t "$IMAGE_TAG" \
        -f "$DOCKERFILE" \
        .
fi

echo ""
echo "======================================"
echo "  ✅ Build Complete!"
echo "======================================"
echo "Image tag: $IMAGE_TAG"
echo "Mode:      $BUILD_MODE"
echo ""
echo "Next steps:"
if [ -n "$IS_JETSON" ]; then
    echo "  Start the container with --runtime nvidia (NOT --gpus all, which fails on"
    echo "  Jetson's cudacompat hook), e.g.:"
    echo "     docker run -it --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all \\"
    echo "         -e NVIDIA_DRIVER_CAPABILITIES=all --network host $IMAGE_TAG"
    echo ""
    if [ "$BUILD_MODE" = "dev" ]; then
        echo "  Projects building on top of this image (leeloo_docker, capacitynet) expect"
        echo "  the tag curobo_ros:aarch64-jazzy — rebuild them after this if it changed."
    else
        echo "  PROD image has curobo_ros pre-installed at /home/curobo_ws. Mount your own"
        echo "  ROS 2 workspace (packages depending on curobo_ros) at /home/ros2_ws."
    fi
    echo ""
elif [ "$BUILD_MODE" = "dev" ]; then
    echo "  1. Ensure you have imported dependencies:"
    echo "     cd ~/ros2_ws/src"
    echo "     vcs import < curobo_ros/my.repos"
    echo ""
    echo "  2. Start the container:"
    echo "     bash start_docker_x86.sh"
    echo ""
else
    echo "  1. Create your ROS 2 workspace on the host:"
    echo "     mkdir -p ~/my_ros2_ws/src"
    echo ""
    echo "  2. Start the container:"
    echo "     bash start_docker_x86.sh"
    echo "     (You'll be asked for your workspace path)"
    echo ""
fi
echo "See docs/getting-started/installation.md for more details."
echo "======================================"

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
##   gpu_choice:   1=Ampere, 2=Ada Lovelace, 3=Turing, 4=Volta
##   mode_choice:  1=DEV, 2=PROD
##
## Examples:
##   ./build_docker.sh 1 1    # Ampere GPU, DEV mode
##   ./build_docker.sh 2 2    # Ada Lovelace GPU, PROD mode
##

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

case $mode_choice in
    1)
        BUILD_MODE="dev"
        DOCKERFILE="x86.dockerfile"
        IMAGE_TAG="curobo_ros:${GPU_NAME}-dev"
        echo "Selected: DEV mode"
        echo "Building development image with full tools..."
        ;;
    2)
        BUILD_MODE="prod"
        DOCKERFILE="x86.optimize-v2-ultra.dockerfile"
        IMAGE_TAG="curobo_ros:${GPU_NAME}-prod"
        echo "Selected: PROD mode"
        echo "Building optimized production image..."
        ;;
    *)
        echo "Invalid choice, defaulting to DEV mode"
        BUILD_MODE="dev"
        DOCKERFILE="x86.dockerfile"
        IMAGE_TAG="curobo_ros:${GPU_NAME}-dev"
        ;;
esac

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
echo "⚠️  Note: This build will take 20-30 minutes and require ~30 GB disk space during build."
echo ""

# Build the Docker image
echo ""
echo "Building Docker image..."
docker build \
    --build-arg TORCH_CUDA_ARCH_LIST="$TORCH_CUDA_ARCH_LIST" \
    -t "$IMAGE_TAG" \
    -f "$DOCKERFILE" \
    .

echo ""
echo "======================================"
echo "  ✅ Build Complete!"
echo "======================================"
echo "Image tag: $IMAGE_TAG"
echo "Mode:      $BUILD_MODE"
echo ""
echo "Next steps:"
if [ "$BUILD_MODE" = "dev" ]; then
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
echo "See doc/concepts/docker_workflow.md for more details."
echo "======================================"

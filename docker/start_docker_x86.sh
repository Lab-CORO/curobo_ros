#!/bin/bash

##
## Start script for curobo_ros Docker containers
## Supports DEV (development) and PROD (production) modes
##

set -e  # Exit on error

echo "======================================"
echo "  curobo_ros Docker Start Script"
echo "======================================"
echo ""

# Step 1: Choose GPU architecture (to select correct image)
echo "Step 1/3: Choose your GPU architecture"
echo "---------------------------------------"
echo "1) Ampere (RTX 30XX series: 3060, 3070, 3080, 3090, A100)"
echo "2) Ada Lovelace (RTX 40XX series: 4060, 4070, 4080, 4090)"
echo "3) Turing (RTX 20XX series: 2060, 2070, 2080)"
echo "4) Volta (Titan V, V100)"
echo ""
read -p "Enter the number corresponding to your GPU: " gpu_choice

case $gpu_choice in
    1)
        GPU_NAME="ampere"
        echo "Selected: Ampere"
        ;;
    2)
        GPU_NAME="ada_lovelace"
        echo "Selected: Ada Lovelace"
        ;;
    3)
        GPU_NAME="turing"
        echo "Selected: Turing"
        ;;
    4)
        GPU_NAME="volta"
        echo "Selected: Volta"
        ;;
    *)
        echo "Invalid choice, defaulting to Ampere"
        GPU_NAME="ampere"
        ;;
esac

echo ""

# Step 2: Choose mode
echo "Step 2/3: Choose container mode"
echo "---------------------------------"
echo "1) DEV  - Development mode"
echo "            → Mounts curobo_ros, curobo_rviz, curobo_msgs from host"
echo "            → For modifying package internals"
echo ""
echo "2) PROD - Production mode"
echo "            → curobo_ros pre-installed in container"
echo "            → Mount your own workspace to use the package"
echo ""
read -p "Enter your choice (1 for DEV, 2 for PROD): " mode_choice

case $mode_choice in
    1)
        RUN_MODE="dev"
        IMAGE_TAG="curobo_ros:${GPU_NAME}-dev"
        CONTAINER_NAME="curobo_${GPU_NAME}_dev"
        echo "Selected: DEV mode"
        ;;
    2)
        RUN_MODE="prod"
        IMAGE_TAG="curobo_ros:${GPU_NAME}-prod"
        CONTAINER_NAME="curobo_${GPU_NAME}_prod"
        echo "Selected: PROD mode"
        ;;
    *)
        echo "Invalid choice, defaulting to DEV mode"
        RUN_MODE="dev"
        IMAGE_TAG="curobo_ros:${GPU_NAME}-dev"
        CONTAINER_NAME="curobo_${GPU_NAME}_dev"
        ;;
esac

echo ""

# Check if image exists
if ! docker image inspect "$IMAGE_TAG" > /dev/null 2>&1; then
    echo "❌ Error: Image '$IMAGE_TAG' not found!"
    echo ""
    echo "Please build the image first:"
    echo "  bash build_docker.sh"
    echo ""
    exit 1
fi

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo ""
    echo "⚠️  Container '$CONTAINER_NAME' already exists!"
    echo ""
    echo "Options:"
    echo "  1) Start existing container"
    echo "  2) Remove and create new container (⚠️  will lose container-specific changes)"
    echo "  3) Cancel"
    echo ""
    read -p "Enter your choice: " container_choice
    
    case $container_choice in
        1)
            echo "Starting existing container..."
            docker start "$CONTAINER_NAME"
            docker exec -it "$CONTAINER_NAME" bash
            exit 0
            ;;
        2)
            echo "Removing existing container..."
            docker stop "$CONTAINER_NAME" 2>/dev/null || true
            docker rm "$CONTAINER_NAME"
            echo "Creating new container..."
            ;;
        3)
            echo "Cancelled."
            exit 0
            ;;
        *)
            echo "Invalid choice, starting existing container..."
            docker start "$CONTAINER_NAME"
            docker exec -it "$CONTAINER_NAME" bash
            exit 0
            ;;
    esac
fi

# Step 3: Configure volumes based on mode
if [ "$RUN_MODE" = "dev" ]; then
    echo "Step 3/3: Development Mode Configuration"
    echo "-----------------------------------------"
    echo "DEV mode mounts these packages from your host:"
    echo "  • curobo_ros"
    echo "  • curobo_rviz"  
    echo "  • curobo_msgs"
    echo ""
    echo "Make sure you have run 'vcs import' to set up dependencies:"
    echo "  cd ~/ros2_ws/src"
    echo "  vcs import < curobo_ros/my.repos"
    echo ""
    read -p "Press Enter to continue..."
    
    # Get the parent directory of curobo_ros (should be ros2_ws/src)
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    CUROBO_ROS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    SRC_DIR="$(dirname "$CUROBO_ROS_DIR")"
    
    # Volume mounts for DEV mode
    VOLUME_MOUNTS="-v ${SRC_DIR}/curobo_ros:/home/ros2_ws/src/curobo_ros \
                   -v ${SRC_DIR}/curobo_rviz:/home/ros2_ws/src/curobo_rviz \
                   -v ${SRC_DIR}/curobo_msgs:/home/ros2_ws/src/curobo_msgs"
    
    WORKSPACE_PATH="/home/ros2_ws"
    
else  # PROD mode
    echo "Step 3/3: Production Mode Configuration"
    echo "----------------------------------------"
    echo "PROD mode requires a ROS 2 workspace path on your host."
    echo ""
    echo "Enter the absolute path to your ROS 2 workspace:"
    echo "Example: /home/username/my_ros2_ws"
    echo ""
    read -p "Workspace path: " USER_WS_PATH
    
    # Validate path
    if [ -z "$USER_WS_PATH" ]; then
        echo "❌ Error: No path provided!"
        exit 1
    fi
    
    # Expand ~ to home directory
    USER_WS_PATH="${USER_WS_PATH/#\~/$HOME}"
    
    # Create workspace if it doesn't exist
    if [ ! -d "$USER_WS_PATH" ]; then
        echo ""
        echo "Workspace directory doesn't exist. Create it?"
        echo "  Path: $USER_WS_PATH"
        read -p "(y/n): " create_choice
        if [ "$create_choice" = "y" ] || [ "$create_choice" = "Y" ]; then
            mkdir -p "$USER_WS_PATH/src"
            echo "✅ Created workspace at $USER_WS_PATH"
        else
            echo "❌ Cancelled."
            exit 1
        fi
    fi
    
    # Volume mount for PROD mode
    VOLUME_MOUNTS="-v ${USER_WS_PATH}:/home/ros2_ws"
    WORKSPACE_PATH="/home/ros2_ws"
fi

echo ""
echo "======================================"
echo "  Container Configuration"
echo "======================================"
echo "Image:      $IMAGE_TAG"
echo "Container:  $CONTAINER_NAME"
echo "Mode:       $RUN_MODE"
if [ "$RUN_MODE" = "prod" ]; then
    echo "Workspace:  $USER_WS_PATH → $WORKSPACE_PATH"
fi
echo ""
read -p "Press Enter to start container, or Ctrl+C to cancel..."

# Enable X11 forwarding (for RViz)
if ! [[ "$OSTYPE" == "msys" ]]; then
    xhost +local:docker 2>/dev/null || echo "Warning: Could not enable X11 forwarding"
fi

# Start the container
echo ""
echo "Starting container..."

if ! [[ "$OSTYPE" == "msys" ]]; then
    # Linux
    docker run --name "$CONTAINER_NAME" -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        --device=/dev/:/dev/ \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        $VOLUME_MOUNTS \
        "$IMAGE_TAG"
else
    # Windows (WSL)
    echo "Detected Windows/WSL. Make sure X server is running!"
    docker run --name "$CONTAINER_NAME" -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        $VOLUME_MOUNTS \
        "$IMAGE_TAG"
fi

echo ""
echo "======================================"
echo "  Container Started!"
echo "======================================"
echo ""
echo "To open additional terminals:"
echo "  docker exec -it $CONTAINER_NAME bash"
echo ""
echo "To stop the container:"
echo "  docker stop $CONTAINER_NAME"
echo ""
echo "To restart the container later:"
echo "  docker start $CONTAINER_NAME"
echo "  docker exec -it $CONTAINER_NAME bash"
echo ""

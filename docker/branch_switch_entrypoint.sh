#!/bin/bash
# Entrypoint script for switching and building branches in the docker container on boot

# Check if the branch argument is empty
if [ -z "$1" ]; then
    # No branch requested: stay where the image left us and just update.
    # This used to run `git checkout ${1}` with $1 empty -- i.e. `git checkout ""`,
    # which fails -- despite the message promising to keep the current branch.
    echo "Branch argument empty, keeping current branch and updating"
    cd /home/ros2_ws/src/curobo_ros
    git fetch
    git pull
    cd /home/ros2_ws/
    colcon build

elif [ "$1" == "main" ]; then
    echo "Keeping default branch: main"
else
    echo "Switching to branch: ${1}"

    # Switch to the branch and rebuild
    cd /home/ros2_ws/src/curobo_ros
    git fetch
    git checkout ${1}
    git pull
    cd /home/ros2_ws/
    colcon build
fi

echo "source /opt/ros/jazzy/setup.bash" >>/root/.bashrc
echo "source /opt/ros/jazzy/share/ros2cli/environment/ros2-argcomplete.bash" >>/root/.bashrc

# Fix missing "ucm_set_global_opts"
apt-get update && apt-get install --reinstall -y \
    libmpich-dev \
    hwloc-nox libmpich12 mpich

# Start an interactive bash shell with ROS and workspace sourced, including bash completion
exec bash --rcfile /root/.bashrc

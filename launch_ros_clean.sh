#!/bin/bash

# Complete environment cleanup for snap conflicts
unset LD_PRELOAD
export LD_LIBRARY_PATH=""

# Set clean library path with system libs first
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# Preload correct pthread to override snap version
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpthread.so.0

# Source ROS
source /opt/ros/jazzy/setup.bash

# Now LD_LIBRARY_PATH has ROS paths, but we need to prepend system paths again
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Remove any snap paths that might have been added
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/snap/' | paste -sd ':')

# Source workspace
source ~/turtlebot3_ws/install/setup.bash

# Set TurtleBot model
export TURTLEBOT3_MODEL=burger

echo "Environment configured:"
echo "LD_LIBRARY_PATH (first 200 chars): ${LD_LIBRARY_PATH:0:200}..."

# Launch
ros2 launch turtlebot3_maze slam.launch.py "$@"

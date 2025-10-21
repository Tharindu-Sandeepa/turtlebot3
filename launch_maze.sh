#!/bin/bash

# Fix for snap library conflicts - remove snap paths from library search
unset LD_PRELOAD

# Source ROS 2 first to get all ROS libraries
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Now prepend system libraries to override any snap libraries
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Remove snap paths if they exist
if [[ ":$LD_LIBRARY_PATH:" == *":/snap/"* ]]; then
    LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/snap/' | tr '\n' ':' | sed 's/:$//')
    export LD_LIBRARY_PATH
fi

# Launch the simulation
ros2 launch turtlebot3_maze slam.launch.py "$@"

#!/bin/bash

echo "========================================="
echo "TurtleBot3 SLAM Launcher"
echo "========================================="
echo ""
echo "IMPORTANT: This script must be run from a REGULAR terminal,"
echo "NOT from VS Code's integrated terminal!"
echo ""
echo "To run this properly:"
echo "1. Open a NEW terminal window (Ctrl+Alt+T)"
echo "2. Run: cd ~/turtlebot3_ws"
echo "3. Run: ./run_outside_vscode.sh"
echo ""
echo "========================================="
echo ""

# Check if running inside snap environment
if [[ -n "${SNAP}" ]] || [[ -n "${SNAP_NAME}" ]] || [[ "$0" == *"/snap/"* ]]; then
    echo "❌ ERROR: You are running from inside a snap environment!"
    echo "   This will cause library conflicts."
    echo "   Please run from a regular terminal instead."
    exit 1
fi

# Fix for snap library conflicts
unset LD_PRELOAD

# Source ROS 2 first
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Now prepend system libraries to take priority over any snap libraries
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Remove snap paths if they exist
if [[ ":$LD_LIBRARY_PATH:" == *":/snap/"* ]]; then
    LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/snap/' | tr '\n' ':' | sed 's/:$//')
    export LD_LIBRARY_PATH
fi

# Set TurtleBot model
export TURTLEBOT3_MODEL=burger

echo "✓ Environment configured"
echo "✓ Starting ROS2 SLAM simulation..."
echo ""

# Launch the simulation
ros2 launch turtlebot3_maze slam.launch.py "$@"

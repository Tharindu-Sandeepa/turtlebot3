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

# Completely clear snap-related environment variables
unset SNAP
unset SNAP_NAME
unset SNAP_INSTANCE_NAME
unset SNAP_REVISION
unset SNAP_VERSION
unset SNAP_REAL_HOME
unset SNAP_USER_DATA
unset SNAP_USER_COMMON
unset SNAP_COMMON
unset SNAP_DATA
unset SNAP_ARCH
unset SNAP_LIBRARY_PATH

# Force the system pthread library to override snap's
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# Source ROS 2 first
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Completely rebuild LD_LIBRARY_PATH without any snap references
# Start with system libraries
NEW_LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/local/lib"

# Add ROS libraries (filter out any snap paths)
if [[ -n "$LD_LIBRARY_PATH" ]]; then
    for path in $(echo "$LD_LIBRARY_PATH" | tr ':' ' '); do
        if [[ "$path" != *"/snap/"* ]] && [[ -d "$path" ]]; then
            NEW_LD_LIBRARY_PATH="$NEW_LD_LIBRARY_PATH:$path"
        fi
    done
fi

export LD_LIBRARY_PATH="$NEW_LD_LIBRARY_PATH"

# Also clean PATH to remove snap
NEW_PATH=""
for path in $(echo "$PATH" | tr ':' ' '); do
    if [[ "$path" != *"/snap/"* ]]; then
        if [[ -z "$NEW_PATH" ]]; then
            NEW_PATH="$path"
        else
            NEW_PATH="$NEW_PATH:$path"
        fi
    fi
done
export PATH="$NEW_PATH"

# Set TurtleBot model
export TURTLEBOT3_MODEL=burger

echo "✓ Environment configured"
echo "✓ LD_LIBRARY_PATH cleaned (no snap libraries)"
echo "✓ LD_PRELOAD set to force system pthread"
echo "✓ Starting ROS2 SLAM simulation..."
echo ""

# Launch the simulation
ros2 launch turtlebot3_maze slam.launch.py "$@"
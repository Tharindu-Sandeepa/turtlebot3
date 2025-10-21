#!/bin/bash

# TurtleBot3 Maze Environment - System Test Script
# Tests all components for IE4060 Assignment 02

echo "🤖 TurtleBot3 Maze Environment - System Test"
echo "============================================"

# Check if ROS2 environment is set up
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 environment not sourced. Please run: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Check TurtleBot3 model
if [ "$TURTLEBOT3_MODEL" != "burger" ]; then
    echo "⚠️  TURTLEBOT3_MODEL not set to 'burger'. Setting it now..."
    export TURTLEBOT3_MODEL=burger
fi

echo "✅ ROS2 Environment: $ROS_DISTRO"
echo "✅ TurtleBot3 Model: $TURTLEBOT3_MODEL"

# Test 1: Maze Generation
echo ""
echo "🧪 Test 1: Maze Generation"
python3 src/turtlebot3_maze/generate_assignment_maze.py

if [ $? -eq 0 ]; then
    echo "✅ Maze generation test passed"
else
    echo "❌ Maze generation test failed"
    exit 1
fi

# Test 2: Package Build
echo ""
echo "🧪 Test 2: Package Build"
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze

if [ $? -eq 0 ]; then
    echo "✅ Package build test passed"
else
    echo "❌ Package build test failed"
    exit 1
fi

# Test 3: Launch File Syntax
echo ""
echo "🧪 Test 3: Launch File Syntax"
source install/setup.bash
ros2 launch turtlebot3_maze slam.launch.py --dry-run

if [ $? -eq 0 ]; then
    echo "✅ Launch file syntax test passed"
else
    echo "❌ Launch file syntax test failed"
    exit 1
fi

# Test 4: DQN Model
echo ""
echo "🧪 Test 4: DQN Model Check"
python3 -c "
import torch
from turtlebot3_maze.dqn_agent import DQNAgent
agent = DQNAgent(28, 5)
print('✅ DQN model initialization test passed')
"

# Test 5: Performance Tester
echo ""
echo "🧪 Test 5: Performance Testing Script"
python3 src/turtlebot3_maze/performance_tester.py --help

if [ $? -eq 0 ]; then
    echo "✅ Performance tester script test passed"
else
    echo "❌ Performance tester script test failed"
    exit 1
fi

echo ""
echo "============================================"
echo "🎉 All system tests completed successfully!"
echo ""
echo "Next steps for assignment:"
echo "1. Generate your maze: python3 src/turtlebot3_maze/generate_assignment_maze.py"
echo "2. Build the package: colcon build --packages-select turtlebot3_maze"
echo "3. Source the workspace: source install/setup.bash"
echo "4. Launch SLAM: ros2 launch turtlebot3_maze slam.launch.py world:=maze_assignment.world"
echo "5. Build your map and save it"
echo "6. Test navigation: ros2 launch turtlebot3_maze navigation.launch.py"
echo ""
echo "Good luck with your assignment! 🚀"

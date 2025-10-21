# TurtleBot3 Autonomous Navigation - IE4060 Assignment 02

Autonomous navigation system for TurtleBot3 Burger using Nav2 (A*/Dijkstra) and Deep Q-Learning (DQN) in custom maze environments.

## 🎯 Assignment Features

- ✅ Custom Gazebo maze creation (0.5m passages as specified)
- ✅ LiDAR-based SLAM mapping with slam_toolbox
- ✅ Nav2 autonomous navigation (Dijkstra/A* path planning)
- ✅ Deep Q-Learning implementation for maze navigation
- ✅ Interactive goal selection via RViz2 map clicks
- ✅ Performance comparison between traditional and learning methods
- ✅ Complete ROS2 Jazzy compatibility

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04/24.04
- ROS2 Jazzy
- Gazebo 11+
- Python 3.8+

### Installation
```bash
# Clone the repository
cd ~/turtlebot3_ws/src
git clone <your-repo-url> turtlebot3_maze

# Build the package
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze

# Source the workspace
source install/setup.bash
export TURTLEBOT3_MODEL=burger
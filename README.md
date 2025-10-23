# TurtleBot3 Autonomous Navigation

A project implementing **autonomous navigation** for the TurtleBot3 Burger using **ROS2 Nav2** (A*/Dijkstra) and **Deep Q-Learning (DQN)** in a custom maze environment.

---

## ✨ Features

- **Custom Gazebo Maze**: Designed with 0.5m passages for navigation testing
- **LiDAR-based SLAM**: Mapping using `slam_toolbox` for accurate localization
- **Nav2 Path Planning**: Efficient navigation with Dijkstra/A* algorithms
- **Deep Q-Learning**: DQN-based navigation for learning optimal paths
- **Interactive Goals**: Set navigation targets via RViz2 map clicks
- **Performance Comparison**: Analyze Nav2 vs. DQN navigation metrics
- **ROS2 Jazzy Compatibility**: Fully compatible with ROS2 Jazzy

---

## 🚀 Quick Start

### 🛠️ Prerequisites

- **OS**: Ubuntu 22.04 or 24.04
- **ROS2**: Jazzy distribution
- **Gazebo**: Version 11 or higher
- **Python**: 3.8 or later

### 📦 Installation

1. **Clone the repository**:
   ```bash
   cd ~/turtlebot3_ws/src
   git clone https://github.com/Tharindu-Sandeepa/turtlebot3.git turtlebot3_maze


Build the package:
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze


Source the workspace:
source install/setup.bash
export TURTLEBOT3_MODEL=burger



▶️ Usage
Run the following commands to start the simulation and navigation:

Launch Gazebo simulation:
ros2 launch turtlebot3_maze gazebo.launch.py


Start SLAM mapping:
ros2 launch turtlebot3_maze slam.launch.py


Run Nav2 navigation:
ros2 launch turtlebot3_maze navigation.launch.py


Train DQN model:
ros2 run turtlebot3_maze train_dqn.py




📜 License
This project is licensed under the MIT License.


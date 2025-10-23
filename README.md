TurtleBot3 Autonomous Navigation
This project implements autonomous navigation for TurtleBot3 Burger using ROS2 Nav2 (A*/Dijkstra) and Deep Q-Learning (DQN) in a custom maze.
Features

Custom Gazebo maze with 0.5m passages
LiDAR-based SLAM with slam_toolbox
Nav2 path planning (Dijkstra/A*)
Deep Q-Learning for maze navigation
Interactive goal selection in RViz2
Comparison of Nav2 and DQN performance
Compatible with ROS2 Jazzy

Quick Start
Prerequisites

Ubuntu 22.04 or 24.04
ROS2 Jazzy
Gazebo 11+
Python 3.8+

Installation

Clone the repository:cd ~/turtlebot3_ws/src
git clone https://github.com/Tharindu-Sandeepa/turtlebot3.git turtlebot3_maze


Build the package:cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze


Source the workspace:source install/setup.bash
export TURTLEBOT3_MODEL=burger



Usage

Launch Gazebo simulation:ros2 launch turtlebot3_maze gazebo.launch.py


Start SLAM mapping:ros2 launch turtlebot3_maze slam.launch.py


Run Nav2 navigation:ros2 launch turtlebot3_maze navigation.launch.py


Train DQN model:ros2 run turtlebot3_maze train_dqn.py


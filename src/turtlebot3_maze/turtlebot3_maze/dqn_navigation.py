#!/usr/bin/env python3
"""
Deep Q-Learning Navigation for TurtleBot3
ROS2 node that implements DQN-based autonomous navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import torch
import math
import time
from collections import deque

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool, String

from .dqn_agent import DQNAgent

class DQNNavigation(Node):
    def __init__(self):
        super().__init__('dqn_navigation')
        
        # Parameters
        self.declare_parameter('model_path', 'models/dqn_trained.pth')
        self.declare_parameter('state_size', 28)  # 24 lidar + 2 goal + 2 position
        self.declare_parameter('action_size', 5)  # Forward, Backward, Left, Right, Stop
        self.declare_parameter('learning_rate', 0.001)
        self.declare_parameter('gamma', 0.99)
        
        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.lidar_data = None
        self.episode_reward = 0
        self.step_count = 0
        self.episode_count = 0
        self.last_action = 0
        self.goal_reached = False
        self.collision = False
        
        # DQN Agent
        state_size = self.get_parameter('state_size').value
        action_size = self.get_parameter('action_size').value
        learning_rate = self.get_parameter('learning_rate').value
        gamma = self.get_parameter('gamma').value
        
        self.agent = DQNAgent(
            state_size=state_size,
            action_size=action_size,
            learning_rate=learning_rate,
            gamma=gamma
        )
        
        # Load pre-trained model if available
        model_path = self.get_parameter('model_path').value
        try:
            self.agent.load(model_path)
            self.get_logger().info(f"Loaded pre-trained model from {model_path}")
        except:
            self.get_logger().info("No pre-trained model found, starting from scratch")
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigation_status_pub = self.create_publisher(String, '/dqn_status', 10)
        self.performance_pub = self.create_publisher(Float32, '/dqn_performance', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/dqn_goal',
            self.goal_callback,
            10
        )
        
        # Timer for DQN control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # Action space definitions
        self.actions = [
            self.forward_action,    # 0: Forward
            self.backward_action,   # 1: Backward  
            self.left_action,       # 2: Turn left
            self.right_action,      # 3: Turn right
            self.stop_action        # 4: Stop
        ]
        
        self.get_logger().info("DQN Navigation Node Started")
        
    def odom_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose.pose
        
    def lidar_callback(self, msg):
        """Process LiDAR data for state representation"""
        # Convert LiDAR data to numpy array and preprocess
        ranges = np.array(msg.ranges)
        
        # Replace inf with max range
        ranges[ranges == np.inf] = msg.range_max
        
        # Downsample to 24 readings (every 15 degrees)
        lidar_downsampled = []
        for i in range(0, len(ranges), 15):
            if i < len(ranges):
                lidar_downsampled.append(ranges[i])
        
        # Ensure we have exactly 24 readings
        while len(lidar_downsampled) < 24:
            lidar_downsampled.append(msg.range_max)
            
        self.lidar_data = np.array(lidar_downsampled[:24])
        
    def goal_callback(self, msg):
        """Set new navigation goal"""
        self.goal_pose = msg.pose
        self.goal_reached = False
        self.episode_reward = 0
        self.step_count = 0
        self.episode_count += 1
        
        self.get_logger().info(f"New goal received: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")
        
    def get_state(self):
        """Get current state representation for DQN"""
        if self.lidar_data is None or self.current_pose is None or self.goal_pose is None:
            return None
            
        # Normalize LiDAR data (0-1)
        lidar_state = self.lidar_data / 3.5  # Normalize by max range
        
        # Calculate relative goal position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # Relative position to goal
        rel_x = goal_x - current_x
        rel_y = goal_y - current_y
        
        # Distance to goal
        distance_to_goal = math.sqrt(rel_x**2 + rel_y**2)
        
        # Normalize relative positions
        rel_x_norm = rel_x / 10.0  # Assuming max maze size 10m
        rel_y_norm = rel_y / 10.0
        
        # Combine all state components
        state = np.concatenate([
            lidar_state,           # 24 values
            [rel_x_norm, rel_y_norm],  # 2 values
            [distance_to_goal / 10.0]  # 1 value
        ])
        
        return state
        
    def calculate_reward(self, state, action):
        """Calculate reward for current state and action"""
        if state is None:
            return 0
            
        reward = 0
        
        # Extract state components
        lidar_readings = state[:24]
        distance_to_goal = state[-1] * 10.0  # Denormalize
        
        # Goal reached reward
        if distance_to_goal < 0.3:  # 0.3m threshold
            reward += 100
            self.goal_reached = True
            self.get_logger().info("Goal reached!")
            
        # Progress reward (negative reward based on distance)
        reward -= distance_to_goal * 0.1
        
        # Collision penalty
        min_distance = np.min(lidar_readings)
        if min_distance < 0.15:  # Very close to obstacle
            reward -= 10
            self.collision = True
            
        # Action penalty (encourage efficiency)
        reward -= 0.01
        
        # Large penalty for getting too close to walls
        if min_distance < 0.25:
            reward -= 5
            
        return reward
        
    def control_loop(self):
        """Main DQN control loop"""
        if self.goal_pose is None or self.goal_reached:
            return
            
        # Get current state
        state = self.get_state()
        if state is None:
            return
            
        # Get action from DQN agent
        action = self.agent.act(state)
        self.last_action = action
        
        # Execute action
        self.actions[action]()
        
        # Get next state
        next_state = self.get_state()
        
        # Calculate reward
        reward = self.calculate_reward(state, action)
        self.episode_reward += reward
        
        # Check if episode is done
        done = self.goal_reached or self.collision or self.step_count > 500
        
        # Store experience and train
        self.agent.remember(state, action, reward, next_state, done)
        self.agent.replay()
        
        # Update step count
        self.step_count += 1
        
        # Publish performance metrics
        performance_msg = Float32()
        performance_msg.data = self.episode_reward
        self.performance_pub.publish(performance_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Episode: {self.episode_count}, Steps: {self.step_count}, Reward: {self.episode_reward:.2f}"
        self.navigation_status_pub.publish(status_msg)
        
        # Reset if episode done
        if done:
            self.get_logger().info(f"Episode {self.episode_count} completed. Total reward: {self.episode_reward:.2f}")
            self.stop_action()
            self.episode_reward = 0
            self.step_count = 0
            self.collision = False
            
            # Update target network periodically
            if self.episode_count % 10 == 0:
                self.agent.update_target_network()
                
            # Save model periodically
            if self.episode_count % 50 == 0:
                self.agent.save('models/dqn_latest.pth')
                
    # Action implementations
    def forward_action(self):
        msg = Twist()
        msg.linear.x = 0.15
        self.cmd_vel_pub.publish(msg)
        
    def backward_action(self):
        msg = Twist()
        msg.linear.x = -0.1
        self.cmd_vel_pub.publish(msg)
        
    def left_action(self):
        msg = Twist()
        msg.angular.z = 0.5
        self.cmd_vel_pub.publish(msg)
        
    def right_action(self):
        msg = Twist()
        msg.angular.z = -0.5
        self.cmd_vel_pub.publish(msg)
        
    def stop_action(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        dqn_navigation = DQNNavigation()
        rclpy.spin(dqn_navigation)
    except KeyboardInterrupt:
        pass
    finally:
        # Save model on shutdown
        dqn_navigation.agent.save('models/dqn_final.pth')
        dqn_navigation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Performance Testing and Comparison Tool
Tests Nav2 vs DQN navigation and generates comparison metrics
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import json
import matplotlib.pyplot as plt
from datetime import datetime

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String

class PerformanceTester(Node):
    def __init__(self):
        super().__init__('performance_tester')
        
        # Test configuration
        self.test_goals = [
            (2.0, 2.0),    # Easy - close to start
            (5.0, 3.0),    # Medium
            (8.0, 8.0),    # Hard - far corner
            (3.0, 7.0),    # Medium-hard
            (7.0, 2.0)     # Medium
        ]
        
        # Performance metrics storage
        self.nav2_metrics = []
        self.dqn_metrics = []
        self.current_test = 0
        self.test_start_time = 0
        self.test_method = "nav2"  # or "dqn"
        
        # Publishers for setting goals
        self.nav2_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.dqn_goal_pub = self.create_publisher(PoseStamped, '/dqn_goal', 10)
        
        # Subscribers for performance data
        self.nav2_perf_sub = self.create_subscription(
            Float32,
            '/navigation_performance',
            self.nav2_perf_callback,
            10
        )
        
        self.dqn_perf_sub = self.create_subscription(
            Float32,
            '/dqn_performance',
            self.dqn_perf_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/navigation_status',
            self.status_callback,
            10
        )
        
        # Test control
        self.test_timer = self.create_timer(2.0, self.run_test_sequence)
        self.current_goal_reached = False
        
        self.get_logger().info("Performance Tester Started")
        
    def create_goal_message(self, x, y):
        """Create a PoseStamped goal message"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        return goal
        
    def run_test_sequence(self):
        """Run automated test sequence"""
        if self.current_test >= len(self.test_goals):
            self.analyze_results()
            return
            
        if not self.current_goal_reached:
            # Start new test
            goal_x, goal_y = self.test_goals[self.current_test]
            
            if self.test_method == "nav2":
                goal_msg = self.create_goal_message(goal_x, goal_y)
                self.nav2_goal_pub.publish(goal_msg)
                self.get_logger().info(f"Testing Nav2 - Goal {self.current_test + 1}: ({goal_x}, {goal_y})")
            else:
                goal_msg = self.create_goal_message(goal_x, goal_y)
                self.dqn_goal_pub.publish(goal_msg)
                self.get_logger().info(f"Testing DQN - Goal {self.current_test + 1}: ({goal_x}, {goal_y})")
                
            self.test_start_time = time.time()
            self.current_goal_reached = False
            
        else:
            # Goal reached, move to next test
            self.current_test += 1
            self.current_goal_reached = False
            
            # Switch methods if all goals tested with current method
            if self.current_test >= len(self.test_goals):
                if self.test_method == "nav2":
                    # Switch to DQN testing
                    self.test_method = "dqn"
                    self.current_test = 0
                    self.get_logger().info("Switching to DQN testing")
                else:
                    # All tests completed
                    self.get_logger().info("All tests completed")
                    
    def nav2_perf_callback(self, msg):
        """Receive Nav2 performance data"""
        if self.test_method == "nav2" and self.test_start_time > 0:
            travel_time = time.time() - self.test_start_time
            efficiency = msg.data
            
            metric = {
                'goal_index': self.current_test,
                'goal_position': self.test_goals[self.current_test],
                'travel_time': travel_time,
                'path_efficiency': efficiency,
                'method': 'nav2',
                'timestamp': datetime.now().isoformat()
            }
            
            self.nav2_metrics.append(metric)
            self.current_goal_reached = True
            self.test_start_time = 0
            
            self.get_logger().info(f"Nav2 Test {self.current_test + 1} completed: Time={travel_time:.2f}s, Efficiency={efficiency:.2f}")
            
    def dqn_perf_callback(self, msg):
        """Receive DQN performance data"""
        if self.test_method == "dqn" and self.test_start_time > 0:
            travel_time = time.time() - self.test_start_time
            reward = msg.data
            
            metric = {
                'goal_index': self.current_test,
                'goal_position': self.test_goals[self.current_test],
                'travel_time': travel_time,
                'total_reward': reward,
                'method': 'dqn',
                'timestamp': datetime.now().isoformat()
            }
            
            self.dqn_metrics.append(metric)
            self.current_goal_reached = True
            self.test_start_time = 0
            
            self.get_logger().info(f"DQN Test {self.current_test + 1} completed: Time={travel_time:.2f}s, Reward={reward:.2f}")
            
    def status_callback(self, msg):
        """Monitor navigation status"""
        if "goal reached" in msg.data.lower() or "success" in msg.data.lower():
            self.current_goal_reached = True
            
    def analyze_results(self):
        """Analyze and display performance results"""
        self.get_logger().info("Analyzing performance results...")
        
        # Calculate statistics
        nav2_times = [m['travel_time'] for m in self.nav2_metrics]
        dqn_times = [m['travel_time'] for m in self.dqn_metrics]
        
        nav2_avg_time = np.mean(nav2_times) if nav2_times else 0
        dqn_avg_time = np.mean(dqn_times) if dqn_times else 0
        
        nav2_std_time = np.std(nav2_times) if nav2_times else 0
        dqn_std_time = np.std(dqn_times) if dqn_times else 0
        
        # Print results
        print("\n" + "="*60)
        print("PERFORMANCE COMPARISON RESULTS")
        print("="*60)
        
        print(f"\nNav2 Performance:")
        print(f"  Average Travel Time: {nav2_avg_time:.2f} ± {nav2_std_time:.2f} seconds")
        print(f"  Total Tests: {len(self.nav2_metrics)}")
        
        print(f"\nDQN Performance:")
        print(f"  Average Travel Time: {dqn_avg_time:.2f} ± {dqn_std_time:.2f} seconds")
        print(f"  Total Tests: {len(self.dqn_metrics)}")
        
        # Calculate improvement
        if nav2_avg_time > 0 and dqn_avg_time > 0:
            time_ratio = dqn_avg_time / nav2_avg_time
            if time_ratio > 1:
                print(f"\nNav2 is {time_ratio:.2f}x faster than DQN")
            else:
                print(f"\nDQN is {1/time_ratio:.2f}x faster than Nav2")
        
        # Generate plots
        self.generate_plots()
        
        # Save results to file
        self.save_results()
        
        print(f"\nDetailed results saved to: performance_results.json")
        print("Plots saved to: performance_comparison.png")
        
        # Shutdown after analysis
        self.get_logger().info("Performance testing completed. Shutting down...")
        rclpy.shutdown()
        
    def generate_plots(self):
        """Generate performance comparison plots"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Travel time comparison
        methods = ['Nav2', 'DQN']
        avg_times = [
            np.mean([m['travel_time'] for m in self.nav2_metrics]) if self.nav2_metrics else 0,
            np.mean([m['travel_time'] for m in self.dqn_metrics]) if self.dqn_metrics else 0
        ]
        
        std_times = [
            np.std([m['travel_time'] for m in self.nav2_metrics]) if self.nav2_metrics else 0,
            np.std([m['travel_time'] for m in self.dqn_metrics]) if self.dqn_metrics else 0
        ]
        
        bars = ax1.bar(methods, avg_times, yerr=std_times, capsize=5, color=['blue', 'orange'])
        ax1.set_ylabel('Travel Time (seconds)')
        ax1.set_title('Average Travel Time Comparison')
        
        # Add value labels on bars
        for bar, value in zip(bars, avg_times):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                    f'{value:.1f}s', ha='center', va='bottom')
        
        # Individual test results
        test_numbers = list(range(1, len(self.test_goals) + 1))
        nav2_times = [m['travel_time'] for m in self.nav2_metrics[:len(test_numbers)]]
        dqn_times = [m['travel_time'] for m in self.dqn_metrics[:len(test_numbers)]]
        
        ax2.plot(test_numbers, nav2_times, 'o-', label='Nav2', linewidth=2, markersize=8)
        ax2.plot(test_numbers, dqn_times, 's-', label='DQN', linewidth=2, markersize=8)
        ax2.set_xlabel('Test Number')
        ax2.set_ylabel('Travel Time (seconds)')
        ax2.set_title('Individual Test Performance')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('performance_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()
        
    def save_results(self):
        """Save detailed results to JSON file"""
        results = {
            'test_configuration': {
                'test_goals': self.test_goals,
                'total_tests': len(self.test_goals),
                'timestamp': datetime.now().isoformat()
            },
            'nav2_results': self.nav2_metrics,
            'dqn_results': self.dqn_metrics,
            'summary': {
                'nav2_avg_time': np.mean([m['travel_time'] for m in self.nav2_metrics]) if self.nav2_metrics else 0,
                'dqn_avg_time': np.mean([m['travel_time'] for m in self.dqn_metrics]) if self.dqn_metrics else 0,
                'nav2_std_time': np.std([m['travel_time'] for m in self.nav2_metrics]) if self.nav2_metrics else 0,
                'dqn_std_time': np.std([m['travel_time'] for m in self.dqn_metrics]) if self.dqn_metrics else 0
            }
        }
        
        with open('performance_results.json', 'w') as f:
            json.dump(results, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    
    print("Starting Performance Testing...")
    print("This will test both Nav2 and DQN navigation with multiple goals.")
    print("Results will be saved automatically.")
    
    performance_tester = PerformanceTester()
    
    try:
        rclpy.spin(performance_tester)
    except KeyboardInterrupt:
        pass
    finally:
        performance_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

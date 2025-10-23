import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import time
import csv
import os

class NavigationLogger(Node):
    # Initialize the logger node
    def __init__(self):
        super().__init__('navigation_logger')
        # Subscribe to the Nav2 global path topic
        self.path_sub = self.create_subscription(
            Path, '/nav2_plan', self.path_callback, 10)
        self.start_time = time.time()
        self.log_file = 'navigation_metrics.csv'
        # Initialize CSV file with headers
        self.init_csv()

    def init_csv(self):
        # Create or overwrite CSV file with headers
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Path_Length', 'Time_Elapsed'])

    def path_callback(self, msg):
        # Calculate path length from the received path message
        path_length = self.calculate_path_length(msg)
        elapsed_time = time.time() - self.start_time
        # Log metrics to CSV
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), 
                           f'{path_length:.2f}', 
                           f'{elapsed_time:.2f}'])
        self.get_logger().info(f'Logged path length: {path_length:.2f}m, time: {elapsed_time:.2f}s')

    def calculate_path_length(self, path):
        # Compute the total length of the path by summing distances between consecutive points
        length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            length += ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
        return length

def main():
    # Initialize ROS2 and run the logger node
    rclpy.init()
    node = NavigationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigation logger')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

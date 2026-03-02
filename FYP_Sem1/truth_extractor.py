import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import os
import csv

class TruthExtractor(Node):
    def __init__(self):
        super().__init__('truth_extractor')
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth',
            self.listener_callback,
            10)
        
        #file setup
        self.file_path = 'ground_truth_log.csv'
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['timestamp', 'x', 'y', 'yaw'])

        self.get_logger().info(f'Logging Ground Truth to {os.path.abspath(self.file_path)}')
    
    def quaternion_to_euler(self, x, y, z, w):
        """convert quaternion to yaw (z-axis rotation)"""
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1-2*(y*y+z*z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def listener_callback(self, msg):
        #extract timestamp
        t=msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        #extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        #extract and convert orientation
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

        #save to csv
        self.writer.writerow([t, x, y, yaw])

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(rgs=args)
    node = TruthExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        rclpy.shutdown()


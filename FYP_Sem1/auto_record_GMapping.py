import os
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime

class AutoRecorder(Node):
    def __init__(self):
        super().__init__('auto_recorder')
        #listen to /cmd_vel to know when the robot starts moving
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.recording_started = False
        self.save_path = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/"
        self.get_logger().info('Waiting for robot movement to start recording...')
    
    def listener_callback(self, msg):
        #trigger if linear or angular velocity is non-zero
        if not self.recording_started and (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01):
            self.recording_started = True

            #create a unique folder name with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = os.path.join(self.save_path, f"baseline_bag_{timestamp}")

            #define topics to record
            topics = "/pose /ground_truth /tf /tf_static /cmd_vel /map"

            self.get_logger().info(f'Recording started: {bag_name}')

            #start recording
            cmd = f"ros2 bag record -o {bag_name} {topics}"
            self.process = subprocess.Popen(cmd, shell=True)
            

def main(args=None):
    rclpy.init(args=args)
    node = AutoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.recording_started:
            subprocess.run(["pkill", "-f", "ros2 bag record"])
            print("\nRecording saved and closed")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

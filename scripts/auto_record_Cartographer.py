#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from datetime import datetime
from action_msgs.msg import GoalStatusArray, GoalStatus

class AutoRecorder(Node):
    def __init__(self):
        super().__init__('auto_recorder_cartographer')

        # --- CONFIGURATION ---
        self.cmd_vel_topic = '/cmd_vel'
        self.nav2_status_topic = '/follow_waypoints/_action/status' 
        self.publish_topic = '/cartographer_pose' 
        
        # --- TF LISTENER ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.publish_topic, 10)
        
        # Timer: Run at 20Hz
        self.timer = self.create_timer(0.05, self.publish_tf_pose)

        # Subscribers
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(GoalStatusArray, self.nav2_status_topic, self.nav_status_callback, 10)

        # State
        self.recording_started = False
        self.process = None
        self.save_path = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/"
        self.bag_folder_name = ""
        self.nav_goal_id = None          
        self.nav_start_time_ns = None    
        self.nav_done = False
        self.motion_start_time_ns = None

        self.get_logger().info('Cartographer Recorder Ready. Waiting for TF (map -> base_footprint)...')

    def publish_tf_pose(self):
        try:
            # Look up the latest available transform
            # We use Time() which creates a "zero" time, requesting the most recent data
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = trans.transform.translation.x
            msg.pose.pose.position.y = trans.transform.translation.y
            msg.pose.pose.orientation = trans.transform.rotation
            
            self.pose_pub.publish(msg)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # ONLY print this once every few seconds to avoid spam, or if debugging
            # self.get_logger().warn(f"TF Failure: {e}", throttle_duration_sec=2.0)
            pass

    def now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def cmd_vel_callback(self, msg: Twist):
        moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)
        
        if self.motion_start_time_ns is None and moving:
            self.motion_start_time_ns = self.now_ns()

        if not self.recording_started and moving:
            self.recording_started = True
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.bag_folder_name = os.path.join(self.save_path, f"Cartographer_bag_{timestamp}")
            os.makedirs(self.bag_folder_name, exist_ok=True)
            bag_out = os.path.join(self.bag_folder_name, "bag")

            # RECORD /cartographer_pose
            topics_list = [self.publish_topic, "/ground_truth", "/tf", "/tf_static", "/cmd_vel", "/map", "/clock"]

            self.get_logger().info(f'Recording started: {bag_out}')
            self.get_logger().info(f'ENSURE YOU SEE MESSAGES ON {self.publish_topic}!')
            
            cmd = ["ros2", "bag", "record", "-o", bag_out] + topics_list
            self.process = subprocess.Popen(cmd)
            
            with open(os.path.join(self.bag_folder_name, "run_info.txt"), "w") as f:
                f.write(f"bag_out: {bag_out}\n")

    def nav_status_callback(self, msg: GoalStatusArray):
        if self.nav_done: return

        if self.nav_goal_id is None:
            for st in msg.status_list:
                if st.status == GoalStatus.STATUS_EXECUTING:
                    self.nav_goal_id = bytes(st.goal_info.goal_id.uuid)
                    self.nav_start_time_ns = self.now_ns()
                    self.get_logger().info("Task Active -> Timer Started.")
                    break

        if self.nav_goal_id is not None:
            for st in msg.status_list:
                if bytes(st.goal_info.goal_id.uuid) != self.nav_goal_id: continue
                if st.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                    duration_s = (self.now_ns() - self.nav_start_time_ns) / 1e9
                    outcome = "SUCCEEDED" if st.status == GoalStatus.STATUS_SUCCEEDED else "FAILED"
                    
                    self.get_logger().info(f"Finished: {outcome}. Duration: {duration_s:.3f}s")
                    self.save_time_log(duration_s, outcome)
                    self.stop_recording()
                    self.nav_done = True
                    break

    def save_time_log(self, duration_s, outcome):
        if not self.bag_folder_name: return
        with open(os.path.join(self.bag_folder_name, "nav_time.txt"), "w") as f:
            f.write(f"outcome: {outcome}\nnav_time_s: {duration_s:.6f}\n")

    def stop_recording(self):
        if self.recording_started and self.process:
            self.process.terminate()
            subprocess.run(["pkill", "-f", "ros2 bag record"], check=False)

    def destroy(self):
        self.stop_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    

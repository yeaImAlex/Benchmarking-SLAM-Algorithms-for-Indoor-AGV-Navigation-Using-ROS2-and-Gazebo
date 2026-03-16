#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime

# Import Action Status messages for timing
from action_msgs.msg import GoalStatusArray
from action_msgs.msg import GoalStatus

class AutoRecorder(Node):
    def __init__(self):
        super().__init__('auto_recorder')

        # --- TOPICS CONFIGURATION ---
        self.cmd_vel_topic = '/cmd_vel'
        # Important: Listen to Waypoints status, not just navigate_to_pose
        self.nav2_status_topic = '/follow_waypoints/_action/status' 
        
        # --- SUBSCRIBERS ---
        
        # 1. Listen for Movement (Start Trigger)
        self.subscription_cmd = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # 2. Listen for Nav Completion (Stop Trigger & Timing)
        self.subscription_status = self.create_subscription(
            GoalStatusArray,
            self.nav2_status_topic,
            self.nav_status_callback,
            10
        )

        # --- STATE VARIABLES ---
        self.recording_started = False
        self.process = None
        self.save_path = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/"
        self.bag_folder_name = ""

        # --- TIMING VARIABLES ---
        self.nav_goal_id = None          # To track the specific action ID
        self.nav_start_time_ns = None    
        self.nav_end_time_ns = None
        self.nav_done = False
        self.motion_start_time_ns = None

        self.get_logger().info('Waiting for robot movement to start recording...')
        self.get_logger().info(f'Monitoring status on: {self.nav2_status_topic}')

    # ---------------------------------------------------
    # Helper: Get current ROS time in nanoseconds
    # ---------------------------------------------------
    def now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    # ---------------------------------------------------
    # 1. START RECORDING ON MOVEMENT
    # ---------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        # Trigger if linear or angular velocity is non-zero
        moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)
        
        # Capture physical start time
        if self.motion_start_time_ns is None and moving:
            self.motion_start_time_ns = self.now_ns()

        if not self.recording_started and moving:
            self.recording_started = True

            # Create folder name with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.bag_folder_name = os.path.join(self.save_path, f"SLAMToolbox_bag_{timestamp}")
            os.makedirs(self.bag_folder_name, exist_ok=True)
            
            # Use a subfolder 'bag' for the actual data to keep it clean
            bag_out = os.path.join(self.bag_folder_name, "bag")

            # Define topics to record (Added /pose as requested)
            # Note: '/pose' is usually published by SLAM toolbox for the robot pose.
            # If using AMCL, you might also want '/amcl_pose'
            topics_str = "/pose /ground_truth /tf /tf_static /cmd_vel /map /clock"
            topics_list = topics_str.split()

            self.get_logger().info(f'Recording started: {bag_out}')

            # Start recording process
            cmd = ["ros2", "bag", "record", "-o", bag_out] + topics_list
            self.process = subprocess.Popen(cmd)

            # Save basic info
            with open(os.path.join(self.bag_folder_name, "run_info.txt"), "w") as f:
                f.write(f"bag_out: {bag_out}\n")
                f.write(f"topics: {topics_str}\n")

    # ---------------------------------------------------
    # 2. STOP RECORDING & LOG TIME ON COMPLETION
    # ---------------------------------------------------
    def nav_status_callback(self, msg: GoalStatusArray):
        if self.nav_done:
            return

        # A. Detect when a Goal becomes Active (Start Timer)
        if self.nav_goal_id is None:
            for st in msg.status_list:
                if st.status == GoalStatus.STATUS_EXECUTING:
                    self.nav_goal_id = bytes(st.goal_info.goal_id.uuid)
                    self.nav_start_time_ns = self.now_ns()
                    self.get_logger().info("Waypoint following ACTIVE -> Timer Started.")
                    break

        # B. Detect when that specific Goal finishes
        if self.nav_goal_id is not None:
            for st in msg.status_list:
                # Check if this status update is for our current goal
                if bytes(st.goal_info.goal_id.uuid) != self.nav_goal_id:
                    continue

                if st.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                    self.nav_end_time_ns = self.now_ns()
                    self.nav_done = True
                    
                    # Calculate duration
                    duration_s = (self.nav_end_time_ns - self.nav_start_time_ns) / 1e9
                    
                    outcome_map = {
                        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                        GoalStatus.STATUS_CANCELED: "CANCELED"
                    }
                    outcome = outcome_map.get(st.status, "UNKNOWN")

                    self.get_logger().info(f"Task Finished: {outcome}. Duration: {duration_s:.3f}s")

                    # Log the time to file
                    self.save_time_log(duration_s, outcome)
                    
                    # Stop recording
                    self.stop_recording()
                    break

    def save_time_log(self, duration_s, outcome):
        if not self.bag_folder_name:
            return # Can't save if recording never started

        log_path = os.path.join(self.bag_folder_name, "nav_time.txt")
        
        delay_info = ""
        if self.motion_start_time_ns and self.nav_start_time_ns:
            delay = (self.nav_start_time_ns - self.motion_start_time_ns) / 1e9
            delay_info = f"\nstartup_delay_s: {delay:.3f}"

        with open(log_path, "w") as f:
            f.write(f"outcome: {outcome}\n")
            f.write(f"nav_time_s: {duration_s:.6f}{delay_info}\n")
        
        self.get_logger().info(f"Time log saved to: {log_path}")

    def stop_recording(self):
        if self.recording_started and self.process:
            self.get_logger().info("Stopping rosbag...")
            self.process.terminate()
            # Ensure all bag record processes are killed
            subprocess.run(["pkill", "-f", "ros2 bag record"], check=False)
            self.recording_started = False

    def destroy(self):
        self.stop_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> Shutting down")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

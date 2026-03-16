#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime

from action_msgs.msg import GoalStatusArray
from action_msgs.msg import GoalStatus


class AutoRecorder(Node):
    def __init__(self):
        super().__init__('auto_recorder')

        # --- Topics ---
        self.cmd_vel_topic = '/cmd_vel'
        self.nav2_status_topic = '/navigate_to_pose/_action/status'

        # Listen to /cmd_vel to know robot starts moving (trigger rosbag)
        self.subscription_cmd = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Listen to Nav2 action status to know when goal completes
        self.subscription_status = self.create_subscription(
            GoalStatusArray,
            self.nav2_status_topic,
            self.nav_status_callback,
            10
        )

        # --- Recording state ---
        self.recording_started = False
        self.process = None
        self.save_path = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/"

        # --- Timing state (ROS time, respects sim time if use_sim_time=True) ---
        self.nav_goal_id = None          # active goal UUID bytes
        self.nav_start_time_ns = None    # when goal became ACTIVE
        self.nav_end_time_ns = None
        self.nav_done = False

        # Optional: keep a movement-start time too (not the official nav time)
        self.motion_start_time_ns = None

        self.get_logger().info("Waiting for robot movement to start recording...")
        self.get_logger().info(f"Listening: {self.cmd_vel_topic} (start recording) + {self.nav2_status_topic} (nav completion)")

    # -------------------------
    # Helper: ROS time now (ns)
    # -------------------------
    def now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    # -----------------------------------
    # 1) Trigger rosbag when robot moves
    # -----------------------------------
    def cmd_vel_callback(self, msg: Twist):
        moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)

        # Track first physical motion time (optional)
        if self.motion_start_time_ns is None and moving:
            self.motion_start_time_ns = self.now_ns()

        # Start rosbag recording only once
        if (not self.recording_started) and moving:
            self.recording_started = True

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = os.path.join(self.save_path, f"baseline_bag_{timestamp}")
            os.makedirs(bag_name, exist_ok=True)

            # Put bag output in a subfolder (cleaner)
            bag_out = os.path.join(bag_name, "bag")

            # Topics to record (add/remove as needed)
            topics = [
                "/tf", "/tf_static",
                "/scan", "/odom",
                "/cmd_vel",
                "/amcl_pose",
                "/map", "/map_metadata",     # useful for SLAM toolbox/cartographer cases
                "/clock"
            ]

            # NOTE: if your ground truth topic name differs, add it here
            # e.g. "/ground_truth" or "/gazebo/model_states"
            topics.append("/ground_truth")

            self.get_logger().info(f"Recording started: {bag_out}")
            cmd = ["ros2", "bag", "record", "-o", bag_out] + topics

            # Start recording
            self.process = subprocess.Popen(cmd)

            # Save a small metadata file
            with open(os.path.join(bag_name, "run_info.txt"), "w") as f:
                f.write(f"bag_out: {bag_out}\n")
                f.write(f"topics: {' '.join(topics)}\n")

    # ---------------------------------------------------
    # 2) Detect Nav2 goal ACTIVE -> SUCCEEDED/ABORTED etc.
    # ---------------------------------------------------
    def nav_status_callback(self, msg: GoalStatusArray):
        if self.nav_done:
            return

        # If we haven't latched onto an active goal yet:
        if self.nav_goal_id is None:
            for st in msg.status_list:
                if st.status == GoalStatus.STATUS_EXECUTING:
                    self.nav_goal_id = bytes(st.goal_info.goal_id.uuid)
                    self.nav_start_time_ns = self.now_ns()
                    self.get_logger().info("Nav2 goal became ACTIVE -> start timing.")
                    break

        # If we already have a goal id, watch for completion:
        if self.nav_goal_id is not None:
            for st in msg.status_list:
                if bytes(st.goal_info.goal_id.uuid) != self.nav_goal_id:
                    continue

                if st.status in (
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_CANCELED,
                ):
                    self.nav_end_time_ns = self.now_ns()
                    self.nav_done = True

                    duration_s = (self.nav_end_time_ns - self.nav_start_time_ns) / 1e9
                    outcome = {
                        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                        GoalStatus.STATUS_CANCELED: "CANCELED",
                    }[st.status]

                    self.get_logger().info(f"Navigation finished: {outcome}, time = {duration_s:.3f} s")

                    # Write timing to file (in same parent folder as bag)
                    # We infer the last created baseline_bag_* folder by looking at process args
                    self.write_time_log(duration_s, outcome)

                    # Stop recording automatically (optional but recommended)
                    self.stop_recording()
                    break

    # -------------------------
    # Write timing log to file
    # -------------------------
    def write_time_log(self, duration_s: float, outcome: str):
        # Try to locate the bag folder from the recording command
        # We created structure: <save_path>/baseline_bag_<timestamp>/bag
        # So we log to parent of 'bag' folder.
        bag_parent = None

        if self.process is not None and hasattr(self.process, "args"):
            # args list: ["ros2","bag","record","-o", bag_out, ...]
            args = self.process.args
            if isinstance(args, list) and "-o" in args:
                out_index = args.index("-o") + 1
                bag_out = args[out_index]
                bag_parent = os.path.dirname(bag_out)

        if bag_parent is None:
            bag_parent = self.save_path

        log_path = os.path.join(bag_parent, "nav_time.txt")

        motion_info = ""
        if self.motion_start_time_ns is not None and self.nav_start_time_ns is not None:
            # How long between "robot started moving" and "nav goal executing"
            dt = (self.nav_start_time_ns - self.motion_start_time_ns) / 1e9
            motion_info = f"\nstart_delay_cmdvel_to_active_s: {dt:.3f}"

        with open(log_path, "w") as f:
            f.write(f"outcome: {outcome}\n")
            f.write(f"nav_time_s: {duration_s:.6f}{motion_info}\n")

        self.get_logger().info(f"Saved navigation time log: {log_path}")

    # -------------------------
    # Stop rosbag cleanly
    # -------------------------
    def stop_recording(self):
        if not self.recording_started:
            return

        self.get_logger().info("Stopping rosbag recording...")

        if self.process is not None:
            # Try graceful stop
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except Exception:
                pass

        # Fallback kill (if rosbag still alive)
        subprocess.run(["pkill", "-f", "ros2 bag record"], check=False)

        self.get_logger().info("Recording stopped and saved.")

    # -------------------------
    # Shutdown handling
    # -------------------------
    def destroy(self):
        if self.recording_started and not self.nav_done:
            self.stop_recording()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> shutting down")
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

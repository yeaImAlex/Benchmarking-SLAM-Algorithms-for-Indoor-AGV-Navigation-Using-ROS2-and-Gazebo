#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import math
import os

def get_quaternion_from_yaw(yaw):
    """
    Convert yaw (radians) to quaternion (z, w) for 2D navigation.
    """
    z = math.sin(yaw * 0.5)
    w = math.cos(yaw * 0.5)
    return z, w

def create_pose(navigator, x, y, yaw):
    """
    Helper to create a PoseStamped message from x, y, yaw.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Position
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    # Orientation (Convert Yaw to Quaternion)
    z, w = get_quaternion_from_yaw(float(yaw))
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # --- 1. SETUP INITIAL POSE ---
    # Even with SLAM, sending this helps ensure the map starts where we expect
    initial_pose = create_pose(navigator, 0.0, 0.0, 0.0)
    
    print("Setting initial pose...")
    navigator.setInitialPose(initial_pose)


    # --- 2. LOAD WAYPOINTS FROM YAML ---
    yaml_file_path = os.path.join(os.getcwd(), 'L_corridor_waypoints.yaml')
    
    if not os.path.exists(yaml_file_path):
        print(f"Error: YAML file not found at {yaml_file_path}")
        return

    print(f"Loading waypoints from {yaml_file_path}...")
    
    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'waypoints' not in data:
        print("Error: YAML file does not contain 'waypoints' key.")
        return

    # --- 3. CONVERT YAML DATA TO NAV2 GOALS ---
    nav_goals = []
    for wp in data['waypoints']:
        pose = create_pose(navigator, wp['x'], wp['y'], wp['yaw'])
        nav_goals.append(pose)

    print(f"Sending path with {len(nav_goals)} waypoints...")
    
    # --- 4. EXECUTE NAVIGATION ---
    navigator.followWaypoints(nav_goals)

    # Keep the script running until the task is complete
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
             print(f"Executing waypoint index: {feedback.current_waypoint}", end='\r')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\nPath completed successfully!")
    elif result == TaskResult.CANCELED:
        print("\nPath was canceled.")
    elif result == TaskResult.FAILED:
        print("\nPath failed!")
    
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()

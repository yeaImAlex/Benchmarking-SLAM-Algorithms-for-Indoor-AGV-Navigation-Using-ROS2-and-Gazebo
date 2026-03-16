#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # Fixed typo: .mg -> .msg
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def create_pose(navigator, x, y, z_orient, w_orient):
    """
    Helper to create a PoseStamped message easily.
    Accepts x, y position and z, w orientation (quaternion).
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # Fixed: added parentheses to now()
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Position
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    # Orientation (Quaternion)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = float(z_orient)
    pose.pose.orientation.w = float(w_orient)
    
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # 1. create the initial pose (start point)
    initial_pose = create_pose(navigator, 0.0, 0.0, 0.0, 1.0)
    
    # 2. send the initial pose to AMCL
    print("setting initial pose...")
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to be fully active
    navigator.waitUntilNav2Active()

    # --- CONFIGURATION: YOUR SPECIFIC COORDINATES ---
    
    # 1) coord1: x = 3.8, y=-3.8; orient1: z=-0.707, w=0.707
    p1 = create_pose(navigator, 3.8, -3.8, -0.707, 0.707)
    
    # 2) coord2: x = 0.3, y=-7.4; orient2: z=1.0, w=0
    p2 = create_pose(navigator, 0.3, -7.4, 1.0, 0.0)
    
    # 3) coord3: x=-3.8, y=-3.8; orient3: z=0.707, w=0.707
    p3 = create_pose(navigator, -3.8, -3.8, 0.707, 0.707)
    
    # 4) coord4: x=0, y=0; orient4: z=0, w=1
    p_finish = create_pose(navigator, 0.0, 0.0, 0.0, 1.0)

    # List of waypoints
    waypoints = [p1, p2, p3, p_finish]

    print("Sending loop command with 4 waypoints...")
    # Fixed typo: followWaypoinys -> followWaypoints
    navigator.followWaypoints(waypoints)

    # Keep the script running until the task is complete
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # Optional: Print current waypoint index
        # if feedback:
        #     print(f"Executing waypoint: {feedback.current_waypoint}")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Loop completed successfully!")
    elif result == TaskResult.CANCELED:
        print("Loop was canceled.")
    elif result == TaskResult.FAILED:
        print("Loop failed!")
    
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()

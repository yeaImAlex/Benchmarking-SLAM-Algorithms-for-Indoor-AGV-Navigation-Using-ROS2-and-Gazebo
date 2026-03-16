import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from scipy.interpolate import interp1d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import sys

def get_bag_data(bag_dir):
    """
    Robustly extracts data from a bag directory by searching for relevant topics.
    Returns: (df_slam, df_gt, slam_topic_name_found)
    """
    print(f"\n--- Reading Bag: {bag_dir} ---")
    
    # 1. Setup Reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"CRITICAL ERROR: Could not open bag: {e}")
        return None, None, None

    # 2. Inspect Topics (Auto-Detection)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    slam_topic = None
    gt_topic = None

    print("Topics found in bag:")
    for t_name, t_type in type_map.items():
        print(f"  - {t_name} ({t_type})")
        
        # Fuzzy search for the topics we want
        if "cartographer_pose" in t_name:
            slam_topic = t_name
        if "ground_truth" in t_name:
            gt_topic = t_name

    if not slam_topic:
        print("\nERROR: Could not find any topic matching 'cartographer_pose'")
        return None, None, None
    if not gt_topic:
        print("\nERROR: Could not find any topic matching 'ground_truth'")
        return None, None, None

    print(f"\n-> Selected SLAM Topic: '{slam_topic}'")
    print(f"-> Selected GT Topic:   '{gt_topic}'")

    # 3. Extract Data
    slam_data = []
    gt_data = []
    
    slam_msg_type = get_message(type_map[slam_topic])
    gt_msg_type = get_message(type_map[gt_topic])

    while reader.has_next():
        (topic, raw_msg, timestamp) = reader.read_next()
        
        if topic == slam_topic:
            msg = deserialize_message(raw_msg, slam_msg_type)
            slam_data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            
        elif topic == gt_topic:
            msg = deserialize_message(raw_msg, gt_msg_type)
            # Handle different GT message types (Odometry vs Pose)
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                gt_data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            elif hasattr(msg, 'pose'): # Sometimes it's just PoseStamped
                gt_data.append([timestamp, msg.pose.position.x, msg.pose.position.y])

    df_slam = pd.DataFrame(slam_data, columns=['time', 'x', 'y'])
    df_gt = pd.DataFrame(gt_data, columns=['time', 'x', 'y'])
    
    print(f"-> Extracted {len(df_slam)} SLAM points and {len(df_gt)} GT points.")
    return df_slam, df_gt, slam_topic

def plot_and_calculate(parent_dir):
    # Handle auto-recorder subfolders vs manual folders
    bag_sub = os.path.join(parent_dir, "bag")
    if os.path.exists(bag_sub):
        target_dir = bag_sub
    else:
        target_dir = parent_dir

    df_slam, df_gt, topic_name = get_bag_data(target_dir)

    if df_slam is None or df_slam.empty or df_gt is None or df_gt.empty:
        print("Analysis Aborted: Insufficient data.")
        return

    # Synchronize timestamps (interpolation)
    df_gt = df_gt.sort_values('time')
    f_x = interp1d(df_gt['time'], df_gt['x'], fill_value="extrapolate")
    f_y = interp1d(df_gt['time'], df_gt['y'], fill_value="extrapolate")
    
    df_slam['gt_x'] = f_x(df_slam['time'])
    df_slam['gt_y'] = f_y(df_slam['time'])

    # Calculate RMSE
    error = np.sqrt((df_slam['x'] - df_slam['gt_x'])**2 + (df_slam['y'] - df_slam['gt_y'])**2)
    rmse = np.sqrt(np.mean(error**2))

    print(f"\n=== RESULTS ===")
    print(f"RMSE: {rmse:.4f} meters")

    # Plot
    plt.figure(figsize=(10, 8))
    plt.plot(df_gt['x'], df_gt['y'], 'k--', label='Ground Truth', alpha=0.5)
    plt.plot(df_slam['x'], df_slam['y'], 'c-', label='Cartographer', linewidth=2)

    # Error lines
    step = max(1, len(df_slam) // 50) 
    for i in range(0, len(df_slam), step):
        plt.plot([df_slam['x'].iloc[i], df_slam['gt_x'].iloc[i]],
                 [df_slam['y'].iloc[i], df_slam['gt_y'].iloc[i]], 'r-', alpha=0.3)
    
    plt.title(f'Cartographer vs Ground Truth\nRMSE: {rmse:.4f} m')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    save_path = os.path.join(parent_dir, 'cartographer_final_graph.png')
    plt.savefig(save_path)
    print(f"Graph saved to: {save_path}")
    plt.show()

if __name__ == "__main__":
    # Point this to your manual_cart_bag folder
    PARENT_DIR = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/manual_cart_bag"
    
    if os.path.exists(PARENT_DIR):
        plot_and_calculate(PARENT_DIR)
    else:
        print(f"Folder not found: {PARENT_DIR}")

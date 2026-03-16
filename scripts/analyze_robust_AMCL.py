import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from scipy.interpolate import interp1d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os

def get_bag_data(bag_dir):
    print(f"\n--- Reading Bag: {bag_dir} ---")
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"CRITICAL ERROR: Could not open bag: {e}")
        return None, None, None

    # 1. Auto-Detect Topics
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    slam_topic = None
    gt_topic = None

    # Find AMCL
    for t in type_map.keys():
        if "amcl_pose" in t:
            slam_topic = t
            break
            
    # Find Baseline (Prefer /ground_truth, fallback to /odom)
    if "/ground_truth" in type_map:
        gt_topic = "/ground_truth"
    elif "/odom" in type_map:
        gt_topic = "/odom"
        print("[WARN] Using '/odom' as ground truth (Simulated GT not found).")

    if not slam_topic or not gt_topic:
        print(f"ERROR: Missing topics. Found: {list(type_map.keys())}")
        return None, None, None

    print(f"-> AMCL Topic: '{slam_topic}'")
    print(f"-> Baseline:   '{gt_topic}'")

    # 2. Extract Data
    slam_data = []
    gt_data = []
    
    slam_msg_type = get_message(type_map[slam_topic])
    gt_msg_type = get_message(type_map[gt_topic])

    while reader.has_next():
        (topic, raw_msg, timestamp) = reader.read_next()
        
        if topic == slam_topic:
            msg = deserialize_message(raw_msg, slam_msg_type)
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                slam_data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            
        elif topic == gt_topic:
            msg = deserialize_message(raw_msg, gt_msg_type)
            # /odom uses msg.pose.pose.position
            # /ground_truth often uses msg.pose.position
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                gt_data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            elif hasattr(msg, 'pose'):
                gt_data.append([timestamp, msg.pose.position.x, msg.pose.position.y])

    return pd.DataFrame(slam_data, columns=['time', 'x', 'y']), pd.DataFrame(gt_data, columns=['time', 'x', 'y']), gt_topic

def plot_and_calculate(parent_dir):
    # Check for bag subfolder
    bag_sub = os.path.join(parent_dir, "bag")
    target_dir = bag_sub if os.path.exists(bag_sub) else parent_dir

    df_slam, df_gt, gt_name = get_bag_data(target_dir)

    if df_slam is None or df_slam.empty:
        print("Analysis Aborted: No AMCL data.")
        return

    # Synchronize
    df_gt = df_gt.sort_values('time')
    f_x = interp1d(df_gt['time'], df_gt['x'], fill_value="extrapolate")
    f_y = interp1d(df_gt['time'], df_gt['y'], fill_value="extrapolate")
    
    df_slam['gt_x'] = f_x(df_slam['time'])
    df_slam['gt_y'] = f_y(df_slam['time'])

    # RMSE
    error = np.sqrt((df_slam['x'] - df_slam['gt_x'])**2 + (df_slam['y'] - df_slam['gt_y'])**2)
    rmse = np.sqrt(np.mean(error**2))

    print(f"\n=== RESULTS ===")
    print(f"RMSE: {rmse:.4f} m")

    # Plot
    plt.figure(figsize=(10, 8))
    plt.plot(df_gt['x'], df_gt['y'], 'k--', label=f'Baseline ({gt_name})', alpha=0.5)
    plt.plot(df_slam['x'], df_slam['y'], 'b-', label='AMCL Estimate', linewidth=2)

    plt.title(f'AMCL vs {gt_name}\nRMSE: {rmse:.4f} m')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    save_path = os.path.join(parent_dir, 'amcl_robust_analysis.png')
    plt.savefig(save_path)
    print(f"Graph saved to: {save_path}")
    plt.show()

if __name__ == "__main__":
    # Point this to your NEW, longer bag folder
    PARENT_DIR = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/baseline_bag_20260115_144158"
    
    if os.path.exists(PARENT_DIR):
        plot_and_calculate(PARENT_DIR)
    else:
        print(f"Path not found: {PARENT_DIR}")

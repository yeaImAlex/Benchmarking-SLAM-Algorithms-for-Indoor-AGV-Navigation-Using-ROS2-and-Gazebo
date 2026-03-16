import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from scipy.interpolate import interp1d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os

def get_bag_data(bag_dir):
    """
    Robustly extracts data by searching for 'cartographer' and 'ground_truth' topics.
    """
    print(f"\n--- Reading Bag for Loop Analysis: {bag_dir} ---")
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"CRITICAL ERROR: Could not open bag: {e}")
        return None, None

    # 1. Auto-detect topics
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    slam_topic = None
    gt_topic = None

    # Fuzzy search to avoid typo errors
    for t_name in type_map.keys():
        if "cartographer" in t_name:
            slam_topic = t_name
        if "ground_truth" in t_name:
            gt_topic = t_name

    if not slam_topic or not gt_topic:
        print(f"ERROR: Could not find topics. Found: {list(type_map.keys())}")
        return None, None

    print(f"-> SLAM Topic detected: '{slam_topic}'")
    print(f"-> GT Topic detected:   '{gt_topic}'")

    # 2. Extract Data
    slam_data = []
    gt_data = []
    
    slam_msg_type = get_message(type_map[slam_topic])
    gt_msg_type = get_message(type_map[gt_topic])

    while reader.has_next():
        (topic, raw_msg, timestamp) = reader.read_next()
        
        if topic == slam_topic:
            msg = deserialize_message(raw_msg, slam_msg_type)
            # Standard PoseWithCovarianceStamped
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                slam_data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            
        elif topic == gt_topic:
            msg = deserialize_message(raw_msg, gt_msg_type)
            # Handle Odometry vs Pose messages
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                gt_data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            elif hasattr(msg, 'pose'):
                gt_data.append([timestamp, msg.pose.position.x, msg.pose.position.y])

    df_slam = pd.DataFrame(slam_data, columns=['time', 'x', 'y'])
    df_gt = pd.DataFrame(gt_data, columns=['time', 'x', 'y'])
    
    return df_slam, df_gt

def get_completion_time(parent_dir):
    """Read the nav_time.txt file if it exists"""
    time_file = os.path.join(parent_dir, "nav_time.txt")
    nav_time = "N/A"
    if os.path.exists(time_file):
        with open(time_file, 'r') as f:
            for line in f:
                if "nav_time_s" in line:
                    try:
                        val = float(line.split(":")[1].strip())
                        nav_time = f"{val:.2f} s"
                    except: pass
    return nav_time

def plot_and_calculate(parent_dir):
    # Handle manual vs auto folder structures
    bag_sub = os.path.join(parent_dir, "bag")
    if os.path.exists(bag_sub):
        target_dir = bag_sub
    else:
        target_dir = parent_dir

    df_slam, df_gt = get_bag_data(target_dir)

    if df_slam is None or df_slam.empty:
        print("Analysis Aborted: No SLAM data found.")
        return

    # Synchronize timestamps
    df_gt = df_gt.sort_values('time')
    f_x = interp1d(df_gt['time'], df_gt['x'], fill_value="extrapolate")
    f_y = interp1d(df_gt['time'], df_gt['y'], fill_value="extrapolate")
    
    df_slam['gt_x'] = f_x(df_slam['time'])
    df_slam['gt_y'] = f_y(df_slam['time'])

    # 1. Calculate RMSE (Trajectory Accuracy)
    error = np.sqrt((df_slam['x'] - df_slam['gt_x'])**2 + (df_slam['y'] - df_slam['gt_y'])**2)
    rmse = np.sqrt(np.mean(error**2))

    # 2. Calculate Loop Closure Error
    # (Distance between Final Estimated Pose and Initial Ground Truth Pose)
    start_gt_x = df_gt['x'].iloc[0]
    start_gt_y = df_gt['y'].iloc[0]
    
    end_est_x = df_slam['x'].iloc[-1]
    end_est_y = df_slam['y'].iloc[-1]
    
    loop_error = np.sqrt((end_est_x - start_gt_x)**2 + (end_est_y - start_gt_y)**2)
    
    # Get Time
    completion_time = get_completion_time(parent_dir)

    print(f"\n=== RESULTS ===")
    print(f"RMSE:       {rmse:.4f} m")
    print(f"Loop Error: {loop_error:.4f} m")

    # Plot
    plt.figure(figsize=(10, 8))
    plt.plot(df_gt['x'], df_gt['y'], 'k--', label='Ground Truth', alpha=0.5)
    
    # Use Cyan ('c-') for Cartographer
    plt.plot(df_slam['x'], df_slam['y'], 'c-', label='Cartographer', linewidth=2)

    # Visualization: Loop Error Line (Orange Dotted)
    plt.plot([end_est_x, start_gt_x], [end_est_y, start_gt_y], 
             color='orange', linewidth=2, linestyle=':', label='Loop Closure Error')

    # Visualization: Trajectory Error Lines
    step = max(1, len(df_slam) // 50) 
    for i in range(0, len(df_slam), step):
        plt.plot([df_slam['x'].iloc[i], df_slam['gt_x'].iloc[i]],
                 [df_slam['y'].iloc[i], df_slam['gt_y'].iloc[i]], 'r-', alpha=0.3)
    
    plt.title(f'Cartographer Loop Analysis\nRMSE: {rmse:.4f} m | Loop Err: {loop_error:.4f} m | Time: {completion_time}')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    save_path = os.path.join(parent_dir, 'cartographer_loop_graph.png')
    plt.savefig(save_path)
    print(f"Graph saved to: {save_path}")
    plt.show()

if __name__ == "__main__":
    # Point this to your MANUAL bag folder
    PARENT_DIR = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/Cartographer_bag_20260113_214740"
    
    if os.path.exists(PARENT_DIR):
        plot_and_calculate(PARENT_DIR)
    else:
        print(f"Folder not found: {PARENT_DIR}")

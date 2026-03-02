import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from scipy.interpolate import interp1d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os

def get_bag_data(bag_path, topic_name):
    """extract (timestamp, x, y) from a ROS2 bag file"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    if topic_name not in type_map:
        print(f"Topic {topic_name} not found.")
        return None
    
    msg_type = get_message(type_map[topic_name])
    data = []

    while reader.has_next():
        (topic, raw_msg, timestamp) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(raw_msg, msg_type)
            
            # Logic for SLAM /pose or Ground Truth /ground_truth
            # Both use msg.pose.pose.position in your specific setup
            try:
                data.append([timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y])
            except AttributeError:
                # Fallback in case message structure differs
                data.append([timestamp, msg.pose.position.x, msg.pose.position.y])
    
    return pd.DataFrame(data, columns=['time', 'x', 'y'])

def plot_and_calculate(bag_path):
    print(f"Analyzing SLAM Toolbox bag: {bag_path}")
    gt_topic = "/ground_truth"
    slam_topic = "/pose" # As verified in your ros2 topic list
    
    # 1. Get data
    df_slam = get_bag_data(bag_path, slam_topic)
    df_gt = get_bag_data(bag_path, gt_topic)

    if df_slam is None or df_gt is None: 
        print("Error: Missing topics in bag. Ensure /pose and /ground_truth were recorded.")
        return

    # 2. Synchronize timestamps (interpolation)
    # Using relative time in seconds for numerical stability
    t_slam = (df_slam['time'] - df_slam['time'].min()) / 1e9
    t_gt = (df_gt['time'] - df_gt['time'].min()) / 1e9

    f_x = interp1d(t_gt, df_gt['x'], fill_value="extrapolate")
    f_y = interp1d(t_gt, df_gt['y'], fill_value="extrapolate")
    
    df_slam['gt_x'] = f_x(t_slam)
    df_slam['gt_y'] = f_y(t_slam)

    # 3. Calculate errors and RMSE
    # Fixed the df_amcl typo to df_slam here:
    error = np.sqrt((df_slam['x'] - df_slam['gt_x'])**2 + (df_slam['y'] - df_slam['gt_y'])**2)
    rmse = np.sqrt(np.mean(error**2))

    # 4. Create visual graph
    plt.figure(figsize=(10, 8))
    plt.plot(df_gt['x'], df_gt['y'], 'k--', label='Ground Truth (Gazebo)', alpha=0.5)
    plt.plot(df_slam['x'], df_slam['y'], 'b-', label=f'SLAM Toolbox Path (RMSE: {rmse:.4f}m)', linewidth=2)

    # Add red error lines (residual lines)
    for i in range(0, len(df_slam), 20):
        plt.plot([df_slam['x'].iloc[i], df_slam['gt_x'].iloc[i]],
                 [df_slam['y'].iloc[i], df_slam['gt_y'].iloc[i]], 'r-', alpha=0.3)
    
    plt.title(f'Step 2.2 Evaluation: SLAM Toolbox vs Ground Truth\nEnvironment: L-Shaped Corridor')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)

    # Save for thesis report
    save_file = os.path.join(bag_path, 'slam_toolbox_analysis.png')
    plt.savefig(save_file)
    print(f"Analysis saved to: {save_file}")
    plt.show()

if __name__ == "__main__":
    # UPDATE THIS to your actual bag folder name from Step 2.2
    BAG_DIR = "/home/ser/FYP/ros2_ws/src/fyp_bot/fyp_bot/SLAMToolbox_bag_20251225_154057" 
    plot_and_calculate(BAG_DIR)
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('fyp_bot')
    config_dir = os.path.join(pkg_share, 'config', 'cartographer')
    lua_file = os.path.join(config_dir, 'cartographer_2d.lua')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name = 'cartographer_node',
        output = 'screen',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_2d.lua'
        ],
        remappings = [
            ('scan', '/scan'),
            ('odom', '/odom'),
        ]
    )

    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        name = 'cartographer_occupancy_grid_node',
        output='screen',
        parameters = [{'use_sim_time': True}],
        arguments = ['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])

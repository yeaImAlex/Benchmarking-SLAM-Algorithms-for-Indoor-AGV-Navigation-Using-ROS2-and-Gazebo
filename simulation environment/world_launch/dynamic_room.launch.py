import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    pkg_fyp_bot = get_package_share_directory('fyp_bot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Path to your custom fyp_burger files
    model_folder = 'fyp_burger'
    sdf_path = os.path.join(pkg_fyp_bot, 'models', model_folder, 'model.sdf')
    urdf_path = os.path.join(pkg_fyp_bot, 'models', model_folder, 'urdf', 'turtlebot3_burger.urdf')

    # Ensure the URDF exists for the state publisher
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # 2. Environment Setup
    # Tell Gazebo to look in your package for the 'fyp_burger' and 'L-shaped_corridor'
    models_path = os.path.join(pkg_fyp_bot, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    # Path to your world file
    world_path = os.path.join(pkg_fyp_bot, 'worlds', 'dynamic_room.world')

    # 3. Launch Gazebo Server (gzserver)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 4. Launch Gazebo Client (gzclient)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 5. Robot State Publisher (Broadcasts TFs like base_link to base_scan)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    # 6. Entity Spawner (Loads the fyp_burger with Ground Truth plugin into Gazebo)
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'fyp_burger',
            '-file', sdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # Create Launch Description and add actions in correct order
    ld = LaunchDescription()

    # Set environment variables first
    ld.add_action(set_gazebo_model_path)

    # Start Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Start Robot logic
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_turtlebot_cmd)

    return ld

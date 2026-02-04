import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Get the path to YOUR package
    pkg_my_bot = get_package_share_directory('my_bot')

    # 2. Setup Gazebo resource path (optional, but good practice)
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_my_bot)
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path

    # 3. Arguments
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='view_bot.rviz', # Changed to a file we know exists
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # 4. Path to Standard SLAM Launch
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # 5. Path to YOUR Config (CRITICAL FIX HERE)
    # Changed 'diff_drive_robot' -> 'my_bot'
    slam_toolbox_params_path = os.path.join(
        pkg_my_bot,
        'config',
        'slam_toolbox_mapping.yaml'
    )

    # 6. Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # Changed folder 'rviz' -> 'config' to match your tree
        arguments=['-d', PathJoinSubstitution([pkg_my_bot, 'config', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    return LaunchDescription([
        rviz_launch_arg,
        rviz_config_arg,
        sim_time_arg,
        rviz_node,
        slam_toolbox_launch
    ])
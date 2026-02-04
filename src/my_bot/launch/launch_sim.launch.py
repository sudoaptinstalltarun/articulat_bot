import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Name
    package_name = 'my_bot'

    # 1. Include the Robot State Publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Path to our world file
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacles.sdf')

    # 3. Launch Gazebo Harmony
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': f'-r {world_path}'}.items()
             )

    # 4. Spawn the Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.1'], # Spawn slightly above ground
        output='screen'
    )

    # 5. Bridge ROS 2 <-> Gazebo Harmony (USING YAML CONFIG)
    
    # Path to the YAML config file we just created
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ros_gz_bridge.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge
    ])
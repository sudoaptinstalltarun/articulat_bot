import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Name
    package_name = 'my_bot'

    # Include the robot_state_publisher launch file
    # This processes the URDF and publishes the /robot_description topic
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Joint State Publisher GUI
    # This node pops up a small window with sliders to move your wheels manually
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

# RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # THIS LINE LOADS YOUR SAVED CONFIG:
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'view_bot.rviz')],
    )

    return LaunchDescription([
        rsp,
        joint_state_publisher_gui,
        rviz2_node
    ])
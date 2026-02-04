import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    # Path to RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'view_bot.rviz')

    return LaunchDescription([
        # WE REMOVED RSP HERE because launch_sim already runs it!
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}], # Syncs time with Gazebo
            arguments=['-d', rviz_config_file]
        )
    ])
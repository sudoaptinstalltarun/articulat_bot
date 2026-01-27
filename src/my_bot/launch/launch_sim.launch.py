import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Name
    package_name = 'my_bot'

    # Include the Robot State Publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

# ... inside generate_launch_description ...
    
    # Path to our new world file
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacles.sdf')

    # Launch Gazebo Harmony
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    
                    # CHANGE: Pass the world file path to gz_args
                    launch_arguments={'gz_args': f'-r {world_path}'}.items()
             )

    # Spawn the Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.1'], # Spawn slightly above ground to prevent clipping
        output='screen'
    )

# Bridge ROS 2 <-> Gazebo Harmony
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint States (Gazebo -> ROS)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Cmd Vel (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # TF (Gazebo -> ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Lidar (Gazebo -> ROS) 
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',    # <--- MAKE SURE COMMA IS HERE
            
            # Camera (Gazebo -> ROS)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge
    ])

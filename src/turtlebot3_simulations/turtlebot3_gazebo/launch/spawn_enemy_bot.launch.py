import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_gazebo')

    # Hardcoded, reliable paths
    urdf_path = os.path.join(pkg_dir, 'models', 'turtlebot3_burger_cam', 'turtlebot3_enemy_bot.sdf')
    bridge_params = os.path.join(pkg_dir, 'params', 'turtlebot3_enemy_bot_bridge.yaml')
    
    yaw_pose = LaunchConfiguration('yaw_pose', default='3.14159')
    declare_yaw_cmd = DeclareLaunchArgument('yaw_pose', default_value='3.14159')


    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'enemy_bot',
            '-file', urdf_path,
            '-x', '2.0',  # Spawning away from our_bot
            '-y', '0.0',
            '-z', '0.01',
            '-Y', yaw_pose,
            '-yaw', yaw_pose
        ],
        output='screen',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='enemy_bot',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace='enemy_bot',
        arguments=['/enemy_bot/camera/image_raw'],
        output='screen',
    )
    
    return LaunchDescription([
        declare_yaw_cmd,
        start_gazebo_ros_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
        # start_gazebo_ros_image_bridge_cmd
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_gazebo')

    # Hardcoded, reliable paths
    urdf_path = os.path.join(pkg_dir, 'models', 'turtlebot3_burger_cam', 'turtlebot3_our_bot.sdf')
    bridge_params = os.path.join(pkg_dir, 'params', 'turtlebot3_burger_cam_bridge.yaml')

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'our_bot',
            '-file', urdf_path,
            '-x', '-2.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='our_bot',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # start_gazebo_ros_image_bridge_cmd = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     namespace='our_bot',
    #     arguments=['/our_bot/camera/image_raw'],
    #     output='screen',
    # )
    
    return LaunchDescription([
        start_gazebo_ros_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
        # start_gazebo_ros_image_bridge_cmd
    ])
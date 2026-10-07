import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Hardcoded path to the our_bot URDF
    urdf_path = os.path.join(pkg_dir, 'urdf', 'turtlebot3_our_bot.urdf')

    # Safely read the file contents at parse time
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='our_bot',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': 'our_bot/'
            }],
        ),
    ])
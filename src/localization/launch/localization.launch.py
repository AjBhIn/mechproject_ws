#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    loc_pkg_share = get_package_share_directory('localization')
    
    # Map file path resolve from share directory
    map_file_path = 'src/map_server/my_nav_map.yaml'
    rviz_config_path = os.path.expanduser('~/mechproject_ws/rvizfiles/rvizfortwobot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map file to load'
    )

    # 1. Map Server & Lifecycle
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': LaunchConfiguration('map')}]
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}]
    )

    # 2. Our Bot AMCL & Lifecycle
    our_bot_amcl = Node(
        namespace='our_bot',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(loc_pkg_share, 'config', 'our_bot_amcl.yaml')],
        remappings=[('map', '/map')]
    )

    lifecycle_our_bot = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_our_bot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['our_bot/amcl']
        }]
    )

    # 3. Enemy Bot AMCL & Lifecycle
    enemy_bot_amcl = Node(
        namespace='enemy_bot',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(loc_pkg_share, 'config', 'enemy_bot_amcl.yaml')],
        remappings=[('map', '/map')]
    )

    lifecycle_enemy_bot = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_enemy_bot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['enemy_bot/amcl']
        }]
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)
    ld.add_action(our_bot_amcl)
    ld.add_action(lifecycle_our_bot)
    ld.add_action(enemy_bot_amcl)
    ld.add_action(lifecycle_enemy_bot)

    return ld
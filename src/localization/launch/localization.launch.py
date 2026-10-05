#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_namespace1 =  LaunchConfiguration('our_bot', default='robot1')
    robot_namespace2 =  LaunchConfiguration('enemy_bot', default='robot2')
    pkg_share = get_package_share_directory('localization')
    map_file = "src/map_server/my_nav_map.yaml"

    # Our Bot Localization Nodes
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'autostart': True, 
            'node_names': ['map_server']
        }]
    )

    our_bot_amcl = Node(
        namespace=robot_namespace1, package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_amcl.yaml')],
        remappings=[('map', '/map')]
    )

    our_bot_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_our_bot', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': ['our_bot/amcl']}]
    )

    enemy_bot_amcl = Node(
        namespace=robot_namespace2, package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_amcl.yaml')],
        remappings=[('map', '/map')]
    )


    enemy_bot_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_enemy_bot', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': ['enemy_bot/amcl']}]
    )


    ld = LaunchDescription()

    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)

    ld.add_action(our_bot_amcl)
    ld.add_action(our_bot_lifecycle)
    ld.add_action(enemy_bot_amcl)
    ld.add_action(enemy_bot_lifecycle)

    return ld
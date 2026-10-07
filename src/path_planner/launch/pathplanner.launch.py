#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('path_planner')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    # Behavior Tree XML Paths
    our_bot_bt_xml = os.path.join(pkg_share, 'config', 'our_bot_behavior.xml')
    enemy_bot_bt_xml = os.path.join(pkg_share, 'config', 'our_bot_behavior.xml')

    # ==============================================================================
    # 1. OUR_BOT NAVIGATION NODES
    # ==============================================================================
    our_bot_planner = Node(
        namespace='our_bot',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_planner_server.yaml')],
        remappings=[('map', '/map'), ('global_costmap/map', '/map')]
    )

    our_bot_controller = Node(
        namespace='our_bot',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_controller.yaml')]
    )

    our_bot_behavior = Node(
        namespace='our_bot',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_recovery.yaml')]
    )

    our_bot_navigator = Node(
        namespace='our_bot',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'our_bot_bt_navigator.yaml'),
            {'default_nav_to_pose_bt_xml': our_bot_bt_xml}
        ],
        remappings=[('goal_pose', '/our_bot/goal_pose')]
    )

    our_bot_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_path_our_bot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'our_bot/planner_server',
                'our_bot/controller_server',
                'our_bot/behavior_server',
                'our_bot/bt_navigator'
            ]
        }]
    )

    # ==============================================================================
    # 2. ENEMY_BOT NAVIGATION NODES
    # ==============================================================================
    enemy_bot_planner = Node(
        namespace='enemy_bot',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_planner_server.yaml')],
        remappings=[('map', '/map'), ('global_costmap/map', '/map')]
    )

    enemy_bot_controller = Node(
        namespace='enemy_bot',
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_controller.yaml')]
    )

    enemy_bot_behavior = Node(
        namespace='enemy_bot',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_recovery.yaml')]
    )

    enemy_bot_navigator = Node(
        namespace='enemy_bot',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'enemy_bot_bt_navigator.yaml'),
            {'default_nav_to_pose_bt_xml': enemy_bot_bt_xml}
        ],
        remappings=[('goal_pose', '/enemy_bot/goal_pose')]
    )

    enemy_bot_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_path_enemy_bot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'enemy_bot/planner_server',
                'enemy_bot/controller_server',
                'enemy_bot/behavior_server',
                'enemy_bot/bt_navigator'
            ]
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    # Our Bot Stack
    ld.add_action(our_bot_planner)
    ld.add_action(our_bot_controller)
    ld.add_action(our_bot_behavior)
    ld.add_action(our_bot_navigator)
    ld.add_action(our_bot_lifecycle)

    # Enemy Bot Stack
    ld.add_action(enemy_bot_planner)
    ld.add_action(enemy_bot_controller)
    ld.add_action(enemy_bot_behavior)
    ld.add_action(enemy_bot_navigator)
    ld.add_action(enemy_bot_lifecycle)

    return ld
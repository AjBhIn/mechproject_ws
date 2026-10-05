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
    pkg_share = get_package_share_directory('path_planner')


    our_bot_planner = Node(
        namespace=robot_namespace1, package='nav2_planner', executable='planner_server', name='planner_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_planner_server.yaml')], 
        remappings=[('map', '/map'), ('global_costmap/map', '/map')]
    )
    our_bot_controller = Node(
        namespace=robot_namespace1, package='nav2_controller', executable='controller_server', name='controller_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_controller.yaml')]
    )
    our_bot_behavior = Node(
        namespace=robot_namespace1, package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_recovery.yaml')]
    )
    our_bot_navigator = Node(
        namespace=robot_namespace1, package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_bt_navigator.yaml'), 
                    {'default_nav_to_pose_bt_xml': os.path.join(pkg_share, 'config', 'our_bot_behavior.xml')}],
        remappings=[('goal_pose', f'/{robot_namespace1}/goal_pose')]
    )
    our_bot_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_our_bot', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': [f'{robot_namespace1}/planner_server', f'{robot_namespace1}/controller_server', f'{robot_namespace1}/behavior_server', f'{robot_namespace1}/bt_navigator']}]
    )



    enemy_bot_planner = Node(
        namespace=robot_namespace2, package='nav2_planner', executable='planner_server', name='planner_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_planner_server.yaml')], 
        remappings=[('map', '/map'), ('global_costmap/map', '/map')]
    )
    enemy_bot_controller = Node(
        namespace=robot_namespace2, package='nav2_controller', executable='controller_server', name='controller_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_controller.yaml')]
    )
    enemy_bot_behavior = Node(
        namespace=robot_namespace2, package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_recovery.yaml')]
    )
    enemy_bot_navigator = Node(
        namespace=robot_namespace2, package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_bt_navigator.yaml'), 
                    {'default_nav_to_pose_bt_xml': os.path.join(pkg_share, 'config', 'our_bot_behavior.xml')}],
        remappings=[('goal_pose', f'/{robot_namespace2}/goal_pose')]
    )
    enemy_bot_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_enemy_bot', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': [f'{robot_namespace2}/planner_server', f'{robot_namespace2}/controller_server', f'{robot_namespace2}/behavior_server', f'{robot_namespace2}/bt_navigator']}]
    )


    ld = LaunchDescription()

    ld.add_action(our_bot_planner)
    ld.add_action(our_bot_controller)
    ld.add_action(our_bot_behavior)
    ld.add_action(our_bot_navigator)
    ld.add_action(our_bot_lifecycle)

    ld.add_action(enemy_bot_planner)
    ld.add_action(enemy_bot_controller)
    ld.add_action(enemy_bot_behavior)
    ld.add_action(enemy_bot_navigator)
    ld.add_action(enemy_bot_lifecycle)  

    return ld
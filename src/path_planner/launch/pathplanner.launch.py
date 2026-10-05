#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    ns1 = LaunchConfiguration('namespace1')
    ns2 = LaunchConfiguration('namespace2')
    
    declare_ns1 = DeclareLaunchArgument('namespace1', default_value='our_bot')
    declare_ns2 = DeclareLaunchArgument('namespace2', default_value='enemy_bot')
    
    plan_pkg_share = get_package_share_directory('path_planner')

    # ==============================================================================
    # DYNAMIC PARAMETER REWRITES (Fixes the Costmap TF Timeout)
    # ==============================================================================
    # This explicitly searches the YAMLs and injects the dynamic namespaces into the frames
    planner_rewrites_1 = {'robot_base_frame': [ns1, '/base_link']}
    controller_rewrites_1 = {'robot_base_frame': [ns1, '/base_link'], 'global_frame': [ns1, '/odom'], 'enemy_frame': [ns2, '/base_link']}
    behavior_rewrites_1 = {'robot_base_frame': [ns1, '/base_link'], 'local_frame': [ns1, '/odom']}
    navigator_rewrites_1 = {'robot_base_frame': [ns1, '/base_link']}

    planner_rewrites_2 = {'robot_base_frame': [ns2, '/base_link']}
    controller_rewrites_2 = {'robot_base_frame': [ns2, '/base_link'], 'global_frame': [ns2, '/odom']}
    behavior_rewrites_2 = {'robot_base_frame': [ns2, '/base_link'], 'local_frame': [ns2, '/odom']}
    navigator_rewrites_2 = {'robot_base_frame': [ns2, '/base_link']}

    # ==============================================================================
    # REWRITTEN YAMLS
    # ==============================================================================
    planner_yaml_ns1 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'our_bot_planner_server.yaml'), root_key=ns1, param_rewrites=planner_rewrites_1, convert_types=True)
    controller_yaml_ns1 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'our_bot_controller.yaml'), root_key=ns1, param_rewrites=controller_rewrites_1, convert_types=True)
    behavior_yaml_ns1 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'our_bot_recovery.yaml'), root_key=ns1, param_rewrites=behavior_rewrites_1, convert_types=True)
    navigator_yaml_ns1 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'our_bot_bt_navigator.yaml'), root_key=ns1, param_rewrites=navigator_rewrites_1, convert_types=True)

    planner_yaml_ns2 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'enemy_bot_planner_server.yaml'), root_key=ns2, param_rewrites=planner_rewrites_2, convert_types=True)
    controller_yaml_ns2 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'enemy_bot_controller.yaml'), root_key=ns2, param_rewrites=controller_rewrites_2, convert_types=True)
    behavior_yaml_ns2 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'enemy_bot_recovery.yaml'), root_key=ns2, param_rewrites=behavior_rewrites_2, convert_types=True)
    navigator_yaml_ns2 = RewrittenYaml(source_file=os.path.join(plan_pkg_share, 'config', 'enemy_bot_bt_navigator.yaml'), root_key=ns2, param_rewrites=navigator_rewrites_2, convert_types=True)

    # ==============================================================================
    # NAMESPACE 1 NAVIGATION (Our Bot)
    # ==============================================================================
    planner_ns1 = Node(
        namespace=ns1, package='nav2_planner', executable='planner_server', name='planner_server', output='screen', 
        parameters=[planner_yaml_ns1], 
        remappings=[('map', '/map')]
    )
    controller_ns1 = Node(
        namespace=ns1, package='nav2_controller', executable='controller_server', name='controller_server', output='screen', 
        parameters=[controller_yaml_ns1]
    )
    behavior_ns1 = Node(
        namespace=ns1, package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', 
        parameters=[behavior_yaml_ns1]
    )
    navigator_ns1 = Node(
        namespace=ns1, package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', 
        parameters=[
            navigator_yaml_ns1, 
            {'default_nav_to_pose_bt_xml': os.path.join(plan_pkg_share, 'config', 'our_bot_behavior.xml'), 'odom_topic': 'odom'}
        ],
        remappings=[('goal_pose', 'goal_pose')]
    )
    lifecycle_ns1 = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav_ns1', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': [[ns1, '/planner_server'], [ns1, '/controller_server'], [ns1, '/behavior_server'], [ns1, '/bt_navigator']]}]
    )

    # ==============================================================================
    # NAMESPACE 2 NAVIGATION (Enemy Bot)
    # ==============================================================================
    planner_ns2 = Node(
        namespace=ns2, package='nav2_planner', executable='planner_server', name='planner_server', output='screen', 
        parameters=[planner_yaml_ns2], 
        remappings=[('map', '/map')]
    )
    controller_ns2 = Node(
        namespace=ns2, package='nav2_controller', executable='controller_server', name='controller_server', output='screen', 
        parameters=[controller_yaml_ns2]
    )
    behavior_ns2 = Node(
        namespace=ns2, package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', 
        parameters=[behavior_yaml_ns2]
    )
    navigator_ns2 = Node(
        namespace=ns2, package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', 
        parameters=[
            navigator_yaml_ns2, 
            {'default_nav_to_pose_bt_xml': os.path.join(plan_pkg_share, 'config', 'our_bot_behavior.xml'), 'odom_topic': 'odom'}
        ],
        remappings=[('goal_pose', 'goal_pose')]
    )
    lifecycle_ns2 = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav_ns2', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': [[ns2, '/planner_server'], [ns2, '/controller_server'], [ns2, '/behavior_server'], [ns2, '/bt_navigator']]}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_ns1)
    ld.add_action(declare_ns2)

    ld.add_action(planner_ns1)
    ld.add_action(controller_ns1)
    ld.add_action(behavior_ns1)
    ld.add_action(navigator_ns1)
    ld.add_action(lifecycle_ns1)

    ld.add_action(planner_ns2)
    ld.add_action(controller_ns2)
    ld.add_action(behavior_ns2)
    ld.add_action(navigator_ns2)
    ld.add_action(lifecycle_ns2)

    return ld
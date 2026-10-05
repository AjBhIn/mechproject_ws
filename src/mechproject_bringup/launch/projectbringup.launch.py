#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 

def generate_launch_description():
    # 1. Global Namespace Arguments
    ns1 = LaunchConfiguration('namespace1')
    ns2 = LaunchConfiguration('namespace2')
    
    declare_ns1 = DeclareLaunchArgument('namespace1', default_value='our_bot', description='Namespace for robot 1')
    declare_ns2 = DeclareLaunchArgument('namespace2', default_value='enemy_bot', description='Namespace for robot 2')

    # 2. Locate the sub-launch files
    localization_dir = get_package_share_directory('localization')
    path_planner_dir = get_package_share_directory('path_planner')

    localization_launch_file = os.path.join(localization_dir, 'launch', 'localization.launch.py')
    path_planner_launch_file = os.path.join(path_planner_dir, 'launch', 'pathplanner.launch.py')

    # 3. Include Localization Launch
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={
            'namespace1': ns1,
            'namespace2': ns2
        }.items()
    )

    # 4. Include Path Planner Launch
    include_path_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_planner_launch_file),
        launch_arguments={
            'namespace1': ns1,
            'namespace2': ns2
        }.items()
    )

    # ==============================================================================
    # 5. RVIZ GLOBAL NODE
    # ==============================================================================
    rviz_config_file = LaunchConfiguration('rviz_config')
    # Make sure this path matches your current workspace name (e.g., mechproject_ws vs namespacedrobot_ws)
    default_rviz_path = os.path.expanduser('~/mechproject_ws/rvizfiles/rvizfortwobot.rviz')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_path, 
        description='Full path to the RViz config file to use'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ==============================================================================
    # 5. ADD FUTURE GLOBAL NODES HERE (e.g., RViz, Camera Drivers, Game Logic)
    # ==============================================================================
    # rviz_node = Node(...)

    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_ns1)
    ld.add_action(declare_ns2)
    ld.add_action(declare_rviz_config_cmd)
    
    # Add includes
    ld.add_action(include_localization)
    ld.add_action(include_path_planner)
    ld.add_action(rviz_node)
    
    return ld
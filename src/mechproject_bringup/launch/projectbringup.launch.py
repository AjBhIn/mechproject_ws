#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Global Launch Arguments
    ns1 = LaunchConfiguration('namespace1')
    ns2 = LaunchConfiguration('namespace2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_ns1 = DeclareLaunchArgument('namespace1', default_value='our_bot', description='Namespace for robot 1')
    declare_ns2 = DeclareLaunchArgument('namespace2', default_value='enemy_bot', description='Namespace for robot 2')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')

    # 2. Locate the sub-launch files
    localization_dir = get_package_share_directory('localization')
    path_planner_dir = get_package_share_directory('path_planner')
    controller_dir = get_package_share_directory('controller')

    # Ensure these filenames match exactly what they are named in your workspace
    localization_launch_file = os.path.join(localization_dir, 'launch', 'localization.launch.py')
    path_planner_launch_file = os.path.join(path_planner_dir, 'launch', 'pathplanner.launch.py')
    controller_launch_file = os.path.join(controller_dir, 'launch', 'controller_bringup.launch.py')

    # 3. Include Localization (Passing arguments down)
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={
            'namespace1': ns1,
            'namespace2': ns2,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 4. Include Path Planner (Passing arguments down)
    include_path_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_planner_launch_file),
        launch_arguments={
            'namespace1': ns1,
            'namespace2': ns2,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 5. Include Autonomy Controller (Passing arguments down)
    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 6. RViz Global Node
    rviz_config_file = LaunchConfiguration('rviz_config')
    default_rviz_path = os.path.expanduser('~/mechproject_ws/rvizfiles/rvizfortwobot.rviz')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_path, description='Path to RViz config'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_ns1)
    ld.add_action(declare_ns2)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config_cmd)
    
    # Add includes
    ld.add_action(include_localization)
    ld.add_action(include_path_planner)
    ld.add_action(include_controller)
    
    # Add RViz
    ld.add_action(rviz_node)
    
    return ld
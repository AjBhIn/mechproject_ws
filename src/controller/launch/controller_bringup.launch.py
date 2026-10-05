#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true, hardware clock if false'
    )

    # 1. Target Broadcaster (The Carrot)
    target_broadcaster = Node(
        package='controller', 
        executable='target_broadcaster',
        name='target_broadcaster',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Evasion Calculator (The Strategist)
    evasion_calculator = Node(
        package='controller', 
        executable='evasion_calculator',
        name='evasion_calculator',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. Chaser Controller (The Tracker)
    chaser_controller = Node(
        package='controller', 
        executable='chaser',
        name='chaser_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 4. Escape Goal Sender (The Panic Button)
    escape_goal_sender = Node(
        package='controller', 
        executable='escape_goal_sender',
        name='escape_goal_sender',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(target_broadcaster)
    ld.add_action(evasion_calculator)
    ld.add_action(chaser_controller)
    ld.add_action(escape_goal_sender)

    return ld
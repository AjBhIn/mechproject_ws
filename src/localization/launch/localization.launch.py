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
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_ns1 = DeclareLaunchArgument('namespace1', default_value='our_bot', description='Namespace for robot 1')
    declare_ns2 = DeclareLaunchArgument('namespace2', default_value='enemy_bot', description='Namespace for robot 2')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    
    loc_pkg_share = get_package_share_directory('localization')
    map_file = 'src/map_server/my_nav_map.yaml'

    amcl_yaml_ns1 = RewrittenYaml(source_file=os.path.join(loc_pkg_share, 'config', 'our_bot_amcl.yaml'), root_key=ns1, param_rewrites={}, convert_types=True)
    amcl_yaml_ns2 = RewrittenYaml(source_file=os.path.join(loc_pkg_share, 'config', 'enemy_bot_amcl.yaml'), root_key=ns2, param_rewrites={}, convert_types=True)


    # ==============================================================================
    # SHARED RESOURCES
    # ==============================================================================
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_file}]
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}]
    )

    # ==============================================================================
    # NAMESPACE 1 LOCALIZATION (Our Bot)
    # ==============================================================================
    amcl_ns1 = Node(
        namespace=ns1, package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
        parameters=[
            amcl_yaml_ns1,
            {'base_frame_id': [ns1, '/base_link'], 'odom_frame_id': [ns1, '/odom'], 'map_topic': '/map', 'scan_topic': 'scan'}
        ],
        remappings=[('map', '/map')]
    )
    lifecycle_ns1 = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_loc_ns1', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 'node_names': [[ns1, '/amcl']]}]
    )

    # ==============================================================================
    # NAMESPACE 2 LOCALIZATION (Enemy Bot)
    # ==============================================================================
    amcl_ns2 = Node(
        namespace=ns2, package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
        parameters=[
            amcl_yaml_ns2,
            {'base_frame_id': [ns2, '/base_link'], 'odom_frame_id': [ns2, '/odom'], 'map_topic': '/map', 'scan_topic': 'scan'}
        ],
        remappings=[('map', '/map')]
    )
    lifecycle_ns2 = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_loc_ns2', output='screen', 
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 0.0, 'node_names': [[ns2, '/amcl']]}]
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
    ld.add_action(declare_ns1)
    ld.add_action(declare_ns2)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(rviz_node)
    
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)
    
    ld.add_action(amcl_ns1)
    ld.add_action(lifecycle_ns1)
    ld.add_action(amcl_ns2)
    ld.add_action(lifecycle_ns2)

    return ld
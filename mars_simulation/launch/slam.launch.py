#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')
    mars_simulation_pkg_dir = get_package_share_directory('mars_simulation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM parameters for each robot - using default params
    slam_params_file = os.path.join(
        slam_toolbox_pkg_dir,
        'config',
        'mapper_params_online_async.yaml'
    )
    
    # SLAM for robot1
    slam_robot1 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot1',
        output='screen',
        parameters=[
            os.path.join(mars_simulation_pkg_dir, 'config', 'slam_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/robot1/scan'),
            ('/map', '/robot1/map'),
            ('/map_metadata', '/robot1/map_metadata'),
        ]
    )
    
    # SLAM for robot2
    slam_robot2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        output='screen',
        parameters=[
            os.path.join(mars_simulation_pkg_dir, 'config', 'slam_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/robot2/scan'),
            ('/map', '/robot2/map'),
            ('/map_metadata', '/robot2/map_metadata'),
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Nodes
        slam_robot1,
        slam_robot2,
    ]) 
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    mars_sim_pkg_dir = get_package_share_directory('mars_simulation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch files
    two_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(mars_sim_pkg_dir, 'launch', 'two_robots.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(mars_sim_pkg_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(mars_sim_pkg_dir, 'launch', 'teleop.launch.py')
        ])
    )
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(mars_sim_pkg_dir, 'launch', 'rviz.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Return the launch description
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Launch Files
        two_robots_launch,
        slam_launch,
        teleop_launch,
        rviz_launch,
    ]) 
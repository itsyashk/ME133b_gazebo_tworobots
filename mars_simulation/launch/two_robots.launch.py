#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    mars_world_pkg_dir = get_package_share_directory('mars_world')
    mars_robots_pkg_dir = get_package_share_directory('mars_robots')
    
    # Gazebo launch
    gazebo_world_path = os.path.join(mars_world_pkg_dir, 'worlds', 'mars.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot URDF file
    robot_urdf_file = os.path.join(mars_robots_pkg_dir, 'urdf', 'mars_robot.xacro')
    
    # Robot URDF via xacro
    robot1_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_urdf_file,
        ' robot_color:=Red'
    ])
    
    robot2_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_urdf_file,
        ' robot_color:=Blue'
    ])
    
    # Robot state publishers
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[
            {'robot_description': robot1_description_content},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/robot_description', '/robot1/robot_description'),
            ('/joint_states', '/robot1/joint_states')
        ]
    )
    
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[
            {'robot_description': robot2_description_content},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/robot_description', '/robot2/robot_description'),
            ('/joint_states', '/robot2/joint_states')
        ]
    )
    
    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': gazebo_world_path,
            'verbose': 'true',
        }.items()
    )
    
    # Spawn robot1
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-topic', '/robot1/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )
    
    # Spawn robot2
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot2',
            '-topic', '/robot2/robot_description',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-robot_namespace', 'robot2'
        ],
        output='screen'
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
        gazebo,
        robot1_state_publisher,
        robot2_state_publisher,
        spawn_robot1,
        spawn_robot2
    ]) 
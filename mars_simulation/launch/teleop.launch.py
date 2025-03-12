#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch teleop for robot1 (WASD keys)
    teleop_robot1 = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_robot1',
        output='screen',
        prefix='xterm -T "Robot 1 Teleop (WASD)" -e',
        remappings=[('/cmd_vel', '/robot1/cmd_vel')],
        parameters=[
            {'key_timeout': 0.0},  # No timeout for keypresses
        ],
    )
    
    # Launch teleop for robot2 (Arrow keys)
    teleop_robot2 = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_robot2',
        output='screen',
        prefix='xterm -T "Robot 2 Teleop (Arrow Keys)" -e',
        remappings=[('/cmd_vel', '/robot2/cmd_vel')],
        parameters=[
            {'key_timeout': 0.0},  # No timeout for keypresses
        ],
    )
    
    # Return the launch description
    return LaunchDescription([
        teleop_robot1,
        teleop_robot2,
    ]) 
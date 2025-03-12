#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch teleop for robot1 (arrow keys)
    teleop_robot1 = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_robot1',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/robot1/cmd_vel')],
    )
    
    # Launch teleop for robot2 (WASD keys)
    teleop_robot2 = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_robot2',
        output='screen',
        prefix='xterm -T "Robot 2 Teleop (WASD)" -e',
        remappings=[('/cmd_vel', '/robot2/cmd_vel')],
    )
    
    # Return the launch description
    return LaunchDescription([
        teleop_robot1,
        teleop_robot2,
    ]) 
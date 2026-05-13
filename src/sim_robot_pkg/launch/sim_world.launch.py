#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'sim_robot_pkg', 'sim_world'],
            output='screen',
            shell=False,
        ),
    ])

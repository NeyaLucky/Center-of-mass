#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of the xArm7 controller box.",
        ),
        DeclareLaunchArgument(
            "hw_ns",
            default_value="xarm",
            description="ROS namespace used by the xArm driver.",
        ),
        DeclareLaunchArgument(
            "report_type",
            default_value="normal",
            description="xArm TCP report type: normal / rich / dev.",
        ),
        DeclareLaunchArgument(
            "positions",
            default_value="noisy",
            description="Pose set for CoM estimation: clean (6) or noisy (21).",
        ),
        DeclareLaunchArgument(
            "method",
            default_value="ransac",
            description="CoM estimation method: lsq / huber / ransac.",
        ),
        DeclareLaunchArgument(
            "compare",
            default_value="false",
            description="If true, print comparison table for lsq / huber / ransac.",
        ),
        DeclareLaunchArgument(
            "samples",
            default_value="15",
            description="Torque samples per pose.",
        ),
        DeclareLaunchArgument(
            "settle",
            default_value="1.0",
            description="Seconds to wait after each move before reading torques.",
        ),
        DeclareLaunchArgument(
            "convert_current",
            default_value="false",
            description=(
                "Set true only when the xArm driver is configured to report "
                "electric current instead of torque (set_report_tau_or_i=1)."
            ),
        ),
    ]

    xarm_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_planner"),
                "launch",
                "xarm7_planner_realmove.launch.py",
            ])
        ),
        launch_arguments={
            "robot_ip":   LaunchConfiguration("robot_ip"),
            "hw_ns":      LaunchConfiguration("hw_ns"),
            "add_gripper": "true",
            "report_type": LaunchConfiguration("report_type"),
            "show_rviz":  "false",
        }.items(),
    )

    com_node = Node(
        package="real_robot_pkg",
        executable="main",
        name="real_robot_com_runner",
        output="screen",
        emulate_tty=True, 
        arguments=[
            "--positions", LaunchConfiguration("positions"),
            "--method",    LaunchConfiguration("method"),
            "--compare",   LaunchConfiguration("compare"),
            "--samples",   LaunchConfiguration("samples"),
            "--settle",    LaunchConfiguration("settle"),
        ],
    )

    return LaunchDescription(declared + [xarm_planner_launch, com_node])

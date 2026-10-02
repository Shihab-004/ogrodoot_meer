#!/usr/bin/env python3
"""
display.launch.py — View MEER rover in RViz2 without Gazebo.

Useful for URDF validation and TF tree inspection during development.

Usage:
  ros2 launch meer_description display.launch.py
  ros2 launch meer_description display.launch.py use_joint_state_pub_gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_meer_description = get_package_share_directory("meer_description")

    declare_use_gui_arg = DeclareLaunchArgument(
        "use_joint_state_pub_gui",
        default_value="true",
        description="Launch joint_state_publisher_gui for interactive joint sliders.",
    )

    use_gui = LaunchConfiguration("use_joint_state_pub_gui")

    xacro_file = os.path.join(pkg_meer_description, "urdf", "meer_rover.urdf.xacro")

    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " use_sim:=false"]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(use_gui),
        output="screen",
    )

    rviz_config = os.path.join(pkg_meer_description, "config", "meer_rover.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_gui_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz2,
        ]
    )

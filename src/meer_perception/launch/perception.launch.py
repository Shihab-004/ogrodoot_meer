#!/usr/bin/env python3
"""
perception.launch.py — Launches the MEER perception pipeline.

Nodes:
  - obstacle_detector  (meer_perception)

Usage:
  ros2 launch meer_perception perception.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time.")

    use_sim_time = LaunchConfiguration("use_sim_time")

    obstacle_detector = Node(
        package="meer_perception",
        executable="obstacle_detector",
        name="obstacle_detector",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"scan_topic": "/scan"},
            {"min_obstacle_dist": 0.5},
        ],
        output="screen",
        remappings=[
            ("/scan", "/lidar/scan"),  # Remap to your actual LiDAR topic
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        obstacle_detector,
    ])

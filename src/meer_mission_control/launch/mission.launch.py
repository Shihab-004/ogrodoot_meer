#!/usr/bin/env python3
"""
mission.launch.py — Launches the MEER mission control node.

Usage:
  ros2 launch meer_mission_control mission.launch.py
  ros2 launch meer_mission_control mission.launch.py auto_start:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time.")

    declare_auto_start = DeclareLaunchArgument(
        "auto_start", default_value="false",
        description="Automatically start mission on node launch.")

    declare_mission_file = DeclareLaunchArgument(
        "mission_file", default_value="",
        description="Path to mission YAML waypoint file.")

    use_sim_time  = LaunchConfiguration("use_sim_time")
    auto_start    = LaunchConfiguration("auto_start")
    mission_file  = LaunchConfiguration("mission_file")

    mission_control = Node(
        package="meer_mission_control",
        executable="mission_control_node",
        name="mission_control_node",
        parameters=[
            {"use_sim_time":  use_sim_time},
            {"auto_start":    auto_start},
            {"mission_file":  mission_file},
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_auto_start,
        declare_mission_file,
        mission_control,
    ])

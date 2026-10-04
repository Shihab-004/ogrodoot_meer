#!/usr/bin/env python3
"""
tree_vitality.launch.py — Launches MEER Rover Tree Vitality Vision Subsystem
Alias / dedicated launch file for NAGC 2026 Autonomous Navigation Mission
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vision = get_package_share_directory("rover_vision")
    default_config = os.path.join(pkg_vision, "config", "vision_params.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock (/clock)."
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_config,
        description="Full path to vision parameters file."
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    tree_vitality_node = Node(
        package="rover_vision",
        executable="tree_vitality_node",
        name="tree_vitality_node",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        tree_vitality_node,
    ])

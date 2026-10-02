#!/usr/bin/env python3
# =============================================================================
# sim.launch.py — MEER Rover Simulation Bringup (Gazebo Harmonic + ROS 2 Jazzy)
# =============================================================================
#
# Launches:
#   1. Gazebo Harmonic (gz sim) with empty arena world
#   2. robot_state_publisher (URDF with ROS REP-103 kinematics & sensors)
#   3. ros_gz_sim create (spawns rover into Gazebo)
#   4. ros_gz_bridge (bridges clock, cmd_vel, odom, tf, joint_states, and all sensors)
#   5. RViz2 (optional visualization)
#
# Usage:
#   ros2 launch meer_bringup sim.launch.py
#   ros2 launch meer_bringup sim.launch.py use_rviz:=true
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_meer_description = get_package_share_directory("meer_description")
    pkg_meer_bringup     = get_package_share_directory("meer_bringup")
    pkg_ros_gz_sim       = get_package_share_directory("ros_gz_sim")

    default_world = os.path.join(pkg_meer_bringup, "worlds", "nagc_arena.sdf")

    # ── Arguments ────────────────────────────────────────────────────────
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file name or path.",
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 alongside Gazebo.",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock (/clock).",
    )

    declare_robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="meer_rover",
        description="Model name in Gazebo.",
    )

    declare_spawn_x_arg = DeclareLaunchArgument(
        "spawn_x", default_value="0.0", description="Spawn X [m]."
    )
    declare_spawn_y_arg = DeclareLaunchArgument(
        "spawn_y", default_value="0.0", description="Spawn Y [m]."
    )
    declare_spawn_z_arg = DeclareLaunchArgument(
        "spawn_z", default_value="0.25", description="Spawn Z [m]."
    )

    world        = LaunchConfiguration("world")
    use_rviz     = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name   = LaunchConfiguration("robot_name")
    spawn_x      = LaunchConfiguration("spawn_x")
    spawn_y      = LaunchConfiguration("spawn_y")
    spawn_z      = LaunchConfiguration("spawn_z")

    # ── URDF via xacro ───────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_meer_description, "urdf", "meer_rover.urdf.xacro")

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_sim:=true",
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # =========================================================================
    # 1. Gazebo Harmonic
    # =========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # =========================================================================
    # 2. robot_state_publisher
    # =========================================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # 3. Spawn robot in Gazebo
    # =========================================================================
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_meer_rover",
        arguments=[
            "-topic", "robot_description",
            "-name",  robot_name,
            "-x",     spawn_x,
            "-y",     spawn_y,
            "-z",     spawn_z,
        ],
        output="screen",
    )

    # =========================================================================
    # 4. ros_gz_bridge — Bi-directional bridge between Gazebo and ROS 2
    # =========================================================================
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            # Simulation Clock: Gazebo → ROS 2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # Velocity Commands: ROS 2 → Gazebo
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",

            # Odometry: Gazebo → ROS 2
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # Transforms: Gazebo → ROS 2
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            # Joint States: Gazebo → ROS 2
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",

            # ── Sensors ──────────────────────────────────────────────────
            # 2D LiDAR: Gazebo → ROS 2
            "/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # IMU: Gazebo → ROS 2
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",

            # GPS / NavSat: Gazebo → ROS 2
            "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",

            # RGB Camera: Gazebo → ROS 2
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # =========================================================================
    # 5. RViz2 (optional)
    # =========================================================================
    rviz_config = os.path.join(pkg_meer_description, "config", "meer_rover.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # =========================================================================
    # Assemble LaunchDescription
    # =========================================================================
    return LaunchDescription(
        [
            declare_world_arg,
            declare_use_rviz_arg,
            declare_use_sim_time_arg,
            declare_robot_name_arg,
            declare_spawn_x_arg,
            declare_spawn_y_arg,
            declare_spawn_z_arg,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            rviz2,
        ]
    )

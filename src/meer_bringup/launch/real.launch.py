#!/usr/bin/env python3
# =============================================================================
# real.launch.py — MEER Rover Physical Hardware Bringup (ROS 2 Humble / Jazzy)
# =============================================================================
#
# Launches:
#   1. robot_state_publisher (URDF via xacro, with use_sim:=false)
#   2. esp32_bridge (Dual-ESP32 serial bridge: motor_port & encoder_port)
#   3. (Optional) 2D LiDAR driver (e.g., RPLiDAR / SLLiDAR)
#   4. (Optional) Nav2 Autonomous Navigation Stack (outdoor A*, costmaps)
#   5. (Optional) Tree Vitality AI Computer Vision Subsystem
#   6. (Optional) RViz2 Visualization
#
# Hardware Architecture:
#   - Motor ESP32 (Actuator): /dev/ttyUSB0 (Cytron MDDS30, cmd_vel subscriber)
#   - Encoder ESP32 (Sensor):  /dev/ttyUSB1 (Wheel encoders, /odom & TF publisher)
#
# Usage:
#   ros2 launch meer_bringup real.launch.py
#   ros2 launch meer_bringup real.launch.py motor_port:=/dev/ttyUSB0 encoder_port:=/dev/ttyUSB1
#   ros2 launch meer_bringup real.launch.py launch_nav:=false
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
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
    pkg_meer_nav         = get_package_share_directory("meer_navigation")
    pkg_meer_vision      = get_package_share_directory("rover_vision")

    # ── 1. Declare Launch Arguments ──────────────────────────────────────────
    # Dual ESP32 Serial Ports
    declare_motor_port_arg = DeclareLaunchArgument(
        "motor_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Motor ESP32 actuator (Cytron MDDS30).",
    )

    declare_encoder_port_arg = DeclareLaunchArgument(
        "encoder_port",
        default_value="/dev/ttyUSB1",
        description="Serial port for Encoder ESP32 sensor (Wheel Odometry).",
    )

    declare_baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Serial communication baud rate for both ESP32 boards.",
    )

    # Simulation / Clock Argument
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (always false for real hardware).",
    )

    # Kinematics Parameters (Matching URDF CAD specifications)
    declare_wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.1155",
        description="Wheel radius in meters (115.5 mm).",
    )

    declare_track_width_arg = DeclareLaunchArgument(
        "track_width",
        default_value="0.8184",
        description="Wheel track separation in meters (818.4 mm).",
    )

    declare_ticks_per_rev_arg = DeclareLaunchArgument(
        "ticks_per_rev",
        default_value="2000.0",
        description="Encoder ticks per full wheel revolution.",
    )

    declare_publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="Whether esp32_bridge broadcasts odom -> base_footprint TF.",
    )

    # Subsystem Flags
    declare_launch_nav_arg = DeclareLaunchArgument(
        "launch_nav",
        default_value="true",
        description="Automatically launch Nav2 autonomous navigation stack.",
    )

    declare_launch_lidar_arg = DeclareLaunchArgument(
        "launch_lidar",
        default_value="false",
        description="Launch hardware LiDAR driver (e.g. RPLiDAR / SLLiDAR).",
    )

    declare_lidar_port_arg = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB2",
        description="Serial port for 2D LiDAR.",
    )

    declare_launch_vision_arg = DeclareLaunchArgument(
        "launch_vision",
        default_value="false",
        description="Launch tree vitality AI vision subsystem.",
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 on rover (set false for headless Pi).",
    )

    # ── 2. Read Launch Configurations ────────────────────────────────────────
    motor_port       = LaunchConfiguration("motor_port")
    encoder_port     = LaunchConfiguration("encoder_port")
    baud_rate        = LaunchConfiguration("baud_rate")
    use_sim_time     = LaunchConfiguration("use_sim_time")
    wheel_radius     = LaunchConfiguration("wheel_radius")
    track_width      = LaunchConfiguration("track_width")
    ticks_per_rev    = LaunchConfiguration("ticks_per_rev")
    publish_tf       = LaunchConfiguration("publish_tf")
    launch_nav       = LaunchConfiguration("launch_nav")
    launch_lidar     = LaunchConfiguration("launch_lidar")
    lidar_port       = LaunchConfiguration("lidar_port")
    launch_vision    = LaunchConfiguration("launch_vision")
    use_rviz         = LaunchConfiguration("use_rviz")

    # ── 3. Robot Description (URDF via Xacro, Real Hardware Mode) ───────────
    xacro_file = os.path.join(pkg_meer_description, "urdf", "meer_rover.urdf.xacro")

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_sim:=false",
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ── 4. Nodes ─────────────────────────────────────────────────────────────

    # A. Robot State Publisher (publishes static & joint TF frames from URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    # B. Dual-ESP32 Serial Bridge Node (Actuator + Sensor)
    esp32_bridge_node = Node(
        package="meer_hardware",
        executable="esp32_bridge.py",
        name="esp32_bridge",
        output="screen",
        parameters=[
            {
                "motor_port": motor_port,
                "encoder_port": encoder_port,
                "baud_rate": ParameterValue(baud_rate, value_type=int),
                "wheel_radius": ParameterValue(wheel_radius, value_type=float),
                "track_width": ParameterValue(track_width, value_type=float),
                "ticks_per_rev": ParameterValue(ticks_per_rev, value_type=float),
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "cmd_vel_timeout": 0.5,
                "command_format": "vel",
            }
        ],
    )

    # C. Nav2 Autonomous Navigation Stack
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_meer_nav, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
        condition=IfCondition(launch_nav),
    )

    # D. Optional 2D LiDAR Driver (e.g. sllidar_ros2 / rplidar_ros)
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": 115200,
                "frame_id": "lidar_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }
        ],
        condition=IfCondition(launch_lidar),
    )

    # E. Optional Tree Vitality AI Vision Subsystem
    tree_vitality_node = Node(
        package="rover_vision",
        executable="tree_vitality_node",
        name="tree_vitality_node",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(launch_vision),
    )

    # F. Optional RViz2
    rviz_config = os.path.join(pkg_meer_description, "config", "meer_rover.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    stdout_linebuf = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # ── 5. Assemble Launch Description ───────────────────────────────────────
    return LaunchDescription(
        [
            stdout_linebuf,
            declare_motor_port_arg,
            declare_encoder_port_arg,
            declare_baud_rate_arg,
            declare_use_sim_time_arg,
            declare_wheel_radius_arg,
            declare_track_width_arg,
            declare_ticks_per_rev_arg,
            declare_publish_tf_arg,
            declare_launch_nav_arg,
            declare_launch_lidar_arg,
            declare_lidar_port_arg,
            declare_launch_vision_arg,
            declare_use_rviz_arg,
            robot_state_publisher,
            esp32_bridge_node,
            lidar_node,
            tree_vitality_node,
            nav2_stack,
            rviz2,
        ]
    )

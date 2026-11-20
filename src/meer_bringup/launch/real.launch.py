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
#   - Motor ESP (NodeMCU):  /dev/rover_motor   ("x,y" -100..100 over USB serial, 115200)
#   - Encoder+GPS ESP:      /dev/rover_encoder ("L,R" tick lines, /odom & TF publisher)
#   - LiDAR:                /dev/rplidar       (published as /lidar/scan)
#   Symlinks come from src/meer_hardware/udev/99-meer-rover.rules
#
# Usage:
#   ros2 launch meer_bringup real.launch.py
#   ros2 launch meer_bringup real.launch.py launch_lidar:=true
#   ros2 launch meer_bringup real.launch.py motor_port:=/dev/ttyUSB0 encoder_port:=/dev/ttyUSB1
#   ros2 launch meer_bringup real.launch.py launch_nav:=false
# =============================================================================

import glob
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


def _find_port(patterns, fallback):
    """Pick a serial port by USB chip name (stable even if ttyUSB numbers change)."""
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if hits:
            return os.path.realpath(hits[0])
    return fallback


# Motor NodeMCU uses a CH340 chip (1a86), RPLidar A2M12 uses a CP2102 (Silicon Labs).
DEFAULT_MOTOR_PORT = _find_port(
    ["/dev/rover_motor", "/dev/serial/by-id/*1a86*", "/dev/serial/by-id/*CH340*"], "/dev/ttyUSB0")
DEFAULT_LIDAR_PORT = _find_port(
    ["/dev/rplidar", "/dev/serial/by-id/*Silicon_Labs*", "/dev/serial/by-id/*CP210*"], "/dev/ttyUSB1")
DEFAULT_ENCODER_PORT = _find_port(["/dev/rover_encoder"], "/dev/rover_encoder")


def _pkg_available(name):
    """True if a ROS package is installed (so a missing driver never crashes the launch)."""
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        print(f"[real.launch] WARNING: package '{name}' not installed -> skipping its node.")
        return False


def generate_launch_description():
    pkg_meer_description = get_package_share_directory("meer_description")
    pkg_meer_nav         = get_package_share_directory("meer_navigation")
    pkg_meer_vision      = get_package_share_directory("rover_vision")

    # ── 1. Declare Launch Arguments ──────────────────────────────────────────
    # Dual ESP32 Serial Ports
    declare_motor_port_arg = DeclareLaunchArgument(
        "motor_port",
        default_value=DEFAULT_MOTOR_PORT,
        description="Serial port for Motor ESP32 actuator (Cytron MDDS30).",
    )

    declare_encoder_port_arg = DeclareLaunchArgument(
        "encoder_port",
        default_value=DEFAULT_ENCODER_PORT,
        description="Serial port for Encoder ESP32 sensor (Wheel Odometry).",
    )

    declare_motor_transport_arg = DeclareLaunchArgument(
        "motor_transport",
        default_value="serial",
        description="'serial' (\"x,y\" over USB cable to motor_port) or 'udp' (Pi must be on ESP WiFi).",
    )

    declare_esp_ip_arg = DeclareLaunchArgument(
        "esp_ip",
        default_value="192.168.1.82",
        description="Static IP of the Motor ESP (NodeMCU) on the Team_Ogrodoot WiFi.",
    )

    declare_max_wheel_speed_arg = DeclareLaunchArgument(
        "max_wheel_speed",
        default_value="1.0",
        description="Wheel speed (m/s) the rover reaches at 100% command. Measure and set.",
    )

    declare_min_cmd_percent_arg = DeclareLaunchArgument(
        "min_cmd_percent",
        default_value="40",
        description="Smallest motor % that actually moves the rover (deadband).",
    )

    declare_odom_source_arg = DeclareLaunchArgument(
        "odom_source",
        default_value="cmd_vel",
        description="'encoder', 'cmd_vel' (open-loop) or 'auto' (encoder if data arrives).",
    )

    declare_baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Serial communication baud rate for both ESP32 boards.",
    )

    # Simulation / Clock Argument
    declare_encoder_baud_arg = DeclareLaunchArgument(
        "encoder_baud",
        default_value="115200",
        description="Baud rate of the Encoder + GPS ESP.",
    )

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

    declare_ticks_left_arg = DeclareLaunchArgument(
        "ticks_per_rev_left",
        default_value="14.0",
        description="Left tooth-wheel encoder pulses per wheel turn (14 teeth).",
    )

    declare_ticks_right_arg = DeclareLaunchArgument(
        "ticks_per_rev_right",
        default_value="15.0",
        description="Right tooth-wheel encoder pulses per wheel turn (15 teeth).",
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
        default_value="true",
        description="Launch hardware LiDAR driver (e.g. RPLiDAR / SLLiDAR).",
    )

    declare_lidar_port_arg = DeclareLaunchArgument(
        "lidar_port",
        default_value=DEFAULT_LIDAR_PORT,
        description="Serial port for 2D LiDAR.",
    )

    declare_lidar_baud_arg = DeclareLaunchArgument(
        "lidar_baud",
        default_value="256000",
        description="LiDAR baud: RPLIDAR A2M12/A3/S1 = 256000, A1/A2M8 = 115200, C1 = 460800.",
    )

    declare_launch_camera_arg = DeclareLaunchArgument(
        "launch_camera",
        default_value="true",
        description="Launch USB webcam driver (v4l2_camera) -> /camera/image_raw.",
    )

    declare_camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="V4L2 webcam device.",
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
    encoder_baud     = LaunchConfiguration("encoder_baud")
    use_sim_time     = LaunchConfiguration("use_sim_time")
    wheel_radius     = LaunchConfiguration("wheel_radius")
    track_width      = LaunchConfiguration("track_width")
    ticks_per_rev    = LaunchConfiguration("ticks_per_rev")
    ticks_left       = LaunchConfiguration("ticks_per_rev_left")
    ticks_right      = LaunchConfiguration("ticks_per_rev_right")
    publish_tf       = LaunchConfiguration("publish_tf")
    launch_nav       = LaunchConfiguration("launch_nav")
    launch_lidar     = LaunchConfiguration("launch_lidar")
    lidar_port       = LaunchConfiguration("lidar_port")
    lidar_baud       = LaunchConfiguration("lidar_baud")
    launch_camera    = LaunchConfiguration("launch_camera")
    camera_device    = LaunchConfiguration("camera_device")
    launch_vision    = LaunchConfiguration("launch_vision")
    use_rviz         = LaunchConfiguration("use_rviz")
    motor_transport  = LaunchConfiguration("motor_transport")
    esp_ip           = LaunchConfiguration("esp_ip")
    max_wheel_speed  = LaunchConfiguration("max_wheel_speed")
    min_cmd_percent  = LaunchConfiguration("min_cmd_percent")
    odom_source      = LaunchConfiguration("odom_source")

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
                "encoder_baud": ParameterValue(encoder_baud, value_type=int),
                "wheel_radius": ParameterValue(wheel_radius, value_type=float),
                "track_width": ParameterValue(track_width, value_type=float),
                "ticks_per_rev": ParameterValue(ticks_per_rev, value_type=float),
                "ticks_per_rev_left": ParameterValue(ticks_left, value_type=float),
                "ticks_per_rev_right": ParameterValue(ticks_right, value_type=float),
                "encoder_signed": False,
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "cmd_vel_timeout": 0.5,
                "motor_transport": motor_transport,
                "esp_ip": esp_ip,
                "esp_udp_port": 4210,
                "max_wheel_speed": ParameterValue(max_wheel_speed, value_type=float),
                "min_cmd_percent": ParameterValue(min_cmd_percent, value_type=int),
                "heartbeat_hz": 10.0,
                "odom_source": odom_source,
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
                "serial_baudrate": ParameterValue(lidar_baud, value_type=int),
                "frame_id": "lidar_link",
                "inverted": False,
                "angle_compensate": True,
            }
        ],
        # Nav2 costmaps listen on /lidar/scan (same as simulation)
        remappings=[("scan", "/lidar/scan")],
        condition=IfCondition(launch_lidar),
    )

    # D2. USB Webcam driver -> /camera/image_raw (used by vision + mission runner)
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "video_device": camera_device,
                "image_size": [1920, 1080],
                "camera_frame_id": "camera_optical_link",
            }
        ],
        remappings=[
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
        condition=IfCondition(launch_camera),
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
            declare_encoder_baud_arg,
            declare_motor_transport_arg,
            declare_esp_ip_arg,
            declare_max_wheel_speed_arg,
            declare_min_cmd_percent_arg,
            declare_odom_source_arg,
            declare_use_sim_time_arg,
            declare_wheel_radius_arg,
            declare_track_width_arg,
            declare_ticks_per_rev_arg,
            declare_ticks_left_arg,
            declare_ticks_right_arg,
            declare_publish_tf_arg,
            declare_launch_nav_arg,
            declare_launch_lidar_arg,
            declare_lidar_port_arg,
            declare_lidar_baud_arg,
            declare_launch_camera_arg,
            declare_camera_device_arg,
            declare_launch_vision_arg,
            declare_use_rviz_arg,
            robot_state_publisher,
            esp32_bridge_node,
            *([lidar_node] if _pkg_available("sllidar_ros2") else []),
            *([camera_node] if _pkg_available("v4l2_camera") else []),
            tree_vitality_node,
            nav2_stack,
            rviz2,
        ]
    )

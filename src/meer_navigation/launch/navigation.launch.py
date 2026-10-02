#!/usr/bin/env python3
# =============================================================================
# navigation.launch.py — MEER Rover Autonomous Navigation Stack
# =============================================================================
#
# Launches the core Nav2 stack configured for the 53m x 34m outdoor arena:
#   1. controller_server  (RegulatedPurePursuitController for 4WD skid-steer)
#   2. smoother_server    (SimpleSmoother for smooth paths)
#   3. planner_server     (NavfnPlanner for global path search)
#   4. behavior_server    (Spin, Backup, Wait recovery behaviors)
#   5. bt_navigator       (Behavior Tree engine for NavigateToPose)
#   6. waypoint_follower  (Multi-waypoint sequencer)
#   7. velocity_smoother  (Acceleration/deceleration limiter)
#   8. collision_monitor  (LiDAR dynamic obstacle safety shield)
#   9. lifecycle_manager  (Transitions all navigation nodes to ACTIVE)
#  10. waypoint_navigator (Competition waypoint coordinator)
#
# Note: Does NOT load docking_server — not needed for outdoor rover missions.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_meer_nav = get_package_share_directory("meer_navigation")

    # ── Arguments ────────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock.")

    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="Automatically transition all lifecycle nodes to active.")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_meer_nav, "config", "nav2_params.yaml"),
        description="Full path to the ROS 2 parameters file.")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart    = LaunchConfiguration("autostart")
    params_file  = LaunchConfiguration("params_file")

    # Lifecycle nodes managed by lifecycle_manager (docking_server excluded!)
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # =========================================================================
    # Navigation Nodes
    # =========================================================================
    nav_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                    {"bond_timeout": 4.0},
                ],
            ),
            Node(
                package="meer_navigation",
                executable="waypoint_navigator",
                name="waypoint_navigator",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_use_sim_time,
            declare_autostart,
            declare_params_file,
            nav_nodes,
        ]
    )

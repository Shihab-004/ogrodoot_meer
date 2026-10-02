#!/usr/bin/env python3
"""
mission_markers_node.py — RViz2 Arena & Mission Objectives Visualizer
Publishes 3D markers for Base Station, Waypoints (Tree & Panorama), and Arena boundaries.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class MissionMarkersNode(Node):
    def __init__(self):
        super().__init__('mission_markers_node')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(MarkerArray, '/mission_markers', qos)

        # Publish markers periodically to ensure RViz receives them
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info("Mission Markers Node started. Publishing arena objectives to /mission_markers")

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ── 1. Base Station Pad (2m x 2m Yellow Box) ──────────────────────────
        pad = Marker()
        pad.header.frame_id = "odom"
        pad.header.stamp = now
        pad.ns = "base_station"
        pad.id = 0
        pad.type = Marker.CUBE
        pad.action = Marker.ADD
        pad.pose.position.x = 0.0
        pad.pose.position.y = 0.0
        pad.pose.position.z = 0.01
        pad.scale.x = 2.0
        pad.scale.y = 2.0
        pad.scale.z = 0.02
        pad.color.r = 0.95
        pad.color.g = 0.85
        pad.color.b = 0.1
        pad.color.a = 0.85
        marker_array.markers.append(pad)

        # ── 2. Base Station Label ─────────────────────────────────────────────
        pad_text = Marker()
        pad_text.header.frame_id = "odom"
        pad_text.header.stamp = now
        pad_text.ns = "base_station"
        pad_text.id = 1
        pad_text.type = Marker.TEXT_VIEW_FACING
        pad_text.action = Marker.ADD
        pad_text.pose.position.x = 0.0
        pad_text.pose.position.y = 0.0
        pad_text.pose.position.z = 1.2
        pad_text.scale.z = 0.7
        pad_text.color.r = 1.0
        pad_text.color.g = 0.9
        pad_text.color.b = 0.2
        pad_text.color.a = 1.0
        pad_text.text = "🏁 BASE STATION (0, 0)"
        marker_array.markers.append(pad_text)

        # ── 3. Waypoint 1: Tree Target (Green Cylinder at 16.0, 4.0) ──────────
        tree = Marker()
        tree.header.frame_id = "odom"
        tree.header.stamp = now
        tree.ns = "waypoints"
        tree.id = 2
        tree.type = Marker.CYLINDER
        tree.action = Marker.ADD
        tree.pose.position.x = 16.0
        tree.pose.position.y = 4.0
        tree.pose.position.z = 1.5
        tree.scale.x = 1.0
        tree.scale.y = 1.0
        tree.scale.z = 3.0
        tree.color.r = 0.1
        tree.color.g = 0.85
        tree.color.b = 0.2
        tree.color.a = 0.7
        marker_array.markers.append(tree)

        # ── 4. Waypoint 1 Label ───────────────────────────────────────────────
        tree_text = Marker()
        tree_text.header.frame_id = "odom"
        tree_text.header.stamp = now
        tree_text.ns = "waypoints"
        tree_text.id = 3
        tree_text.type = Marker.TEXT_VIEW_FACING
        tree_text.action = Marker.ADD
        tree_text.pose.position.x = 16.0
        tree_text.pose.position.y = 4.0
        tree_text.pose.position.z = 3.6
        tree_text.scale.z = 0.8
        tree_text.color.r = 0.2
        tree_text.color.g = 1.0
        tree_text.color.b = 0.4
        tree_text.color.a = 1.0
        tree_text.text = "🌲 WP 1: Tree Target (16.0m, 4.0m)\n[Task: Alive/Dead Detection]"
        marker_array.markers.append(tree_text)

        # ── 5. Waypoint 2: Panorama Station (Cyan Cylinder at 30.0, -6.0) ─────
        pano = Marker()
        pano.header.frame_id = "odom"
        pano.header.stamp = now
        pano.ns = "waypoints"
        pano.id = 4
        pano.type = Marker.CYLINDER
        pano.action = Marker.ADD
        pano.pose.position.x = 30.0
        pano.pose.position.y = -6.0
        pano.pose.position.z = 1.25
        pano.scale.x = 1.0
        pano.scale.y = 1.0
        pano.scale.z = 2.5
        pano.color.r = 0.1
        pano.color.g = 0.8
        pano.color.b = 0.95
        pano.color.a = 0.7
        marker_array.markers.append(pano)

        # ── 6. Waypoint 2 Label ───────────────────────────────────────────────
        pano_text = Marker()
        pano_text.header.frame_id = "odom"
        pano_text.header.stamp = now
        pano_text.ns = "waypoints"
        pano_text.id = 5
        pano_text.type = Marker.TEXT_VIEW_FACING
        pano_text.action = Marker.ADD
        pano_text.pose.position.x = 30.0
        pano_text.pose.position.y = -6.0
        pano_text.pose.position.z = 3.2
        pano_text.scale.z = 0.8
        pano_text.color.r = 0.2
        pano_text.color.g = 0.9
        pano_text.color.b = 1.0
        pano_text.color.a = 1.0
        pano_text.text = "📷 WP 2: Panorama Station (30.0m, -6.0m)\n[Task: 360° Panorama Capture]"
        marker_array.markers.append(pano_text)

        # ── 7. Arena Boundary Line (Orange 53m x 34m) ─────────────────────────
        bound = Marker()
        bound.header.frame_id = "odom"
        bound.header.stamp = now
        bound.ns = "arena_bounds"
        bound.id = 6
        bound.type = Marker.LINE_STRIP
        bound.action = Marker.ADD
        bound.scale.x = 0.15
        bound.color.r = 1.0
        bound.color.g = 0.4
        bound.color.b = 0.0
        bound.color.a = 0.9

        corners = [
            (-3.0, -17.0),
            (50.0, -17.0),
            (50.0,  17.0),
            (-3.0,  17.0),
            (-3.0, -17.0),
        ]
        for cx, cy in corners:
            p = Point()
            p.x = float(cx)
            p.y = float(cy)
            p.z = 0.05
            bound.points.append(p)
        marker_array.markers.append(bound)

        self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

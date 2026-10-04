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
        # Declare coordinate parameters (defaults to official competition waypoints)
        self.declare_parameter('tree_x', 16.0)
        self.declare_parameter('tree_y', 4.0)
        self.declare_parameter('pano_x', 30.0)
        self.declare_parameter('pano_y', -6.0)

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

        tree_x = float(self.get_parameter('tree_x').value)
        tree_y = float(self.get_parameter('tree_y').value)
        pano_x = float(self.get_parameter('pano_x').value)
        pano_y = float(self.get_parameter('pano_y').value)

        # ── 3. GPS Point 1: Designated Tree Trunk ─────────────────────────────
        tree = Marker()
        tree.header.frame_id = "odom"
        tree.header.stamp = now
        tree.ns = "waypoints"
        tree.id = 2
        tree.type = Marker.CYLINDER
        tree.action = Marker.ADD
        tree.pose.position.x = tree_x
        tree.pose.position.y = tree_y
        tree.pose.position.z = 1.5
        tree.scale.x = 0.5
        tree.scale.y = 0.5
        tree.scale.z = 3.0
        tree.color.r = 0.45
        tree.color.g = 0.30
        tree.color.b = 0.18
        tree.color.a = 0.9
        marker_array.markers.append(tree)

        # ── 3b. GPS Point 1: 1.5m Tolerance Ring ─────────────────────────────
        tree_ring = Marker()
        tree_ring.header.frame_id = "odom"
        tree_ring.header.stamp = now
        tree_ring.ns = "waypoints"
        tree_ring.id = 12
        tree_ring.type = Marker.CYLINDER
        tree_ring.action = Marker.ADD
        tree_ring.pose.position.x = tree_x
        tree_ring.pose.position.y = tree_y
        tree_ring.pose.position.z = 0.02
        tree_ring.scale.x = 3.0  # 1.5m radius = 3.0m diameter
        tree_ring.scale.y = 3.0
        tree_ring.scale.z = 0.02
        tree_ring.color.r = 0.1
        tree_ring.color.g = 0.85
        tree_ring.color.b = 0.2
        tree_ring.color.a = 0.35
        marker_array.markers.append(tree_ring)

        # ── 4. GPS Point 1 Label ──────────────────────────────────────────────
        tree_text = Marker()
        tree_text.header.frame_id = "odom"
        tree_text.header.stamp = now
        tree_text.ns = "waypoints"
        tree_text.id = 3
        tree_text.type = Marker.TEXT_VIEW_FACING
        tree_text.action = Marker.ADD
        tree_text.pose.position.x = tree_x
        tree_text.pose.position.y = tree_y
        tree_text.pose.position.z = 3.6
        tree_text.scale.z = 0.8
        tree_text.color.r = 0.2
        tree_text.color.g = 1.0
        tree_text.color.b = 0.4
        tree_text.color.a = 1.0
        tree_text.text = f"🌳 GPS Point 1: Designated Tree ({tree_x:.1f}m, {tree_y:.1f}m)\n[Task: Alive/Dead Detection (±1.5m Zone)]"
        marker_array.markers.append(tree_text)

        # ── 5. GPS Point 2: Panorama Station Marker ──────────────────────────
        pano = Marker()
        pano.header.frame_id = "odom"
        pano.header.stamp = now
        pano.ns = "waypoints"
        pano.id = 4
        pano.type = Marker.CYLINDER
        pano.action = Marker.ADD
        pano.pose.position.x = pano_x
        pano.pose.position.y = pano_y
        pano.pose.position.z = 1.25
        pano.scale.x = 0.3
        pano.scale.y = 0.3
        pano.scale.z = 2.5
        pano.color.r = 0.1
        pano.color.g = 0.8
        pano.color.b = 0.95
        pano.color.a = 0.9
        marker_array.markers.append(pano)

        # ── 5b. GPS Point 2: 1.5m Tolerance Ring ─────────────────────────────
        pano_ring = Marker()
        pano_ring.header.frame_id = "odom"
        pano_ring.header.stamp = now
        pano_ring.ns = "waypoints"
        pano_ring.id = 13
        pano_ring.type = Marker.CYLINDER
        pano_ring.action = Marker.ADD
        pano_ring.pose.position.x = pano_x
        pano_ring.pose.position.y = pano_y
        pano_ring.pose.position.z = 0.02
        pano_ring.scale.x = 3.0  # 1.5m radius = 3.0m diameter
        pano_ring.scale.y = 3.0
        pano_ring.scale.z = 0.02
        pano_ring.color.r = 0.1
        pano_ring.color.g = 0.8
        pano_ring.color.b = 0.95
        pano_ring.color.a = 0.35
        marker_array.markers.append(pano_ring)

        # ── 6. GPS Point 2 Label ──────────────────────────────────────────────
        pano_text = Marker()
        pano_text.header.frame_id = "odom"
        pano_text.header.stamp = now
        pano_text.ns = "waypoints"
        pano_text.id = 5
        pano_text.type = Marker.TEXT_VIEW_FACING
        pano_text.action = Marker.ADD
        pano_text.pose.position.x = pano_x
        pano_text.pose.position.y = pano_y
        pano_text.pose.position.z = 3.2
        pano_text.scale.z = 0.8
        pano_text.color.r = 0.2
        pano_text.color.g = 0.9
        pano_text.color.b = 1.0
        pano_text.color.a = 1.0
        pano_text.text = f"📷 GPS Point 2: Panorama Station ({pano_x:.1f}m, {pano_y:.1f}m)\n[Task: 360° Panorama Scan (±1.5m Zone)]"
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

        # ── 8. Designated Tree 3D Foliar Canopy Sphere ────────────────────────
        canopy_alive = Marker()
        canopy_alive.header.frame_id = "odom"
        canopy_alive.header.stamp = now
        canopy_alive.ns = "waypoints"
        canopy_alive.id = 20
        canopy_alive.type = Marker.SPHERE
        canopy_alive.action = Marker.ADD
        canopy_alive.pose.position.x = tree_x
        canopy_alive.pose.position.y = tree_y
        canopy_alive.pose.position.z = 2.8
        canopy_alive.scale.x = 2.4
        canopy_alive.scale.y = 2.4
        canopy_alive.scale.z = 1.8
        canopy_alive.color.r = 0.12
        canopy_alive.color.g = 0.75
        canopy_alive.color.b = 0.18
        canopy_alive.color.a = 0.85
        marker_array.markers.append(canopy_alive)

        # ── 9. Terrain Obstacles ──────────────────────────────────────────────
        obstacles = [
            ("boulder_1", 30, Marker.CYLINDER, 6.5, 1.4, 0.45, 1.3, 1.3, 0.9, 0.42, 0.40, 0.38),
            ("ditch_2",   31, Marker.CUBE,     9.5, -2.2, 0.25, 2.2, 0.5, 0.5, 0.85, 0.35, 0.15),
            ("rock_3",    32, Marker.CUBE,     11.5, -0.6, 0.5,  1.4, 0.95, 1.0, 0.45, 0.42, 0.40),
            ("boulder_6", 34, Marker.CYLINDER, 22.5, -1.2, 0.55, 1.8, 1.8, 1.1, 0.44, 0.40, 0.36),
            ("ridge_7",   35, Marker.CUBE,     26.5, 2.5, 0.5,  1.7, 0.85, 1.0, 0.36, 0.36, 0.36),
            ("boulder_8", 36, Marker.CYLINDER, 28.0, -3.5, 0.45, 1.2, 1.2, 0.9, 0.42, 0.38, 0.36),
        ]
        for name, m_id, m_type, ox, oy, oz, sx, sy, sz, cr, cg, cb in obstacles:
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = now
            m.ns = "terrain_obstacles"
            m.id = m_id
            m.type = m_type
            m.action = Marker.ADD
            m.pose.position.x = float(ox)
            m.pose.position.y = float(oy)
            m.pose.position.z = float(oz)
            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = float(sz)
            m.color.r = float(cr)
            m.color.g = float(cg)
            m.color.b = float(cb)
            m.color.a = 0.85
            marker_array.markers.append(m)

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

#!/usr/bin/env python3
"""
tree_vitality_node.py — Tree Vitality Detection ROS 2 Node
MEER Rover — NAGC 2026 Autonomous Navigation Mission

Subscribes to onboard camera, runs tree localization and vitality classification,
publishes results on /tree/vitality, publishes annotated debug stream, provides
/tree/inspect service for the mission sequencer, and writes official competition logs.
"""

import os
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image

from rover_msgs.msg import TreeVitality
from rover_msgs.srv import InspectTree
from rover_vision.classifier_interface import create_classifier, VitalityResult
from rover_vision.tree_detector import TreeDetector, DetectionResult


class TreeVitalityNode(Node):
    def __init__(self):
        super().__init__('tree_vitality_node')

        # ── Declare Parameters ───────────────────────────────────────────────
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('classifier_type', 'spectral')
        self.declare_parameter('model_path', '')
        self.declare_parameter('auto_publish_continuous', True)
        self.declare_parameter('continuous_rate_hz', 2.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('output_dir', os.path.expanduser('~/autonomous_meer/log/mission_submissions'))
        self.declare_parameter('verdict_file_name', 'tree_verdict.log')
        self.declare_parameter('snapshot_file_name', 'tree_snapshot.jpg')

        # ── Retrieve Parameters ──────────────────────────────────────────────
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.classifier_type = str(self.get_parameter('classifier_type').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.auto_publish = bool(self.get_parameter('auto_publish_continuous').value)
        self.rate_hz = float(self.get_parameter('continuous_rate_hz').value)
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.verdict_file = str(self.get_parameter('verdict_file_name').value)
        self.snapshot_file = str(self.get_parameter('snapshot_file_name').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # ── Internal Components ──────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_cv_frame: Optional[cv2.Mat] = None
        self.latest_header = None
        self.latest_result: Optional[VitalityResult] = None
        self.latest_detection: Optional[DetectionResult] = None

        self.detector = TreeDetector()
        self.classifier = create_classifier(
            classifier_type=self.classifier_type,
            model_path=self.model_path if self.model_path else None
        )

        # ── Publishers ───────────────────────────────────────────────────────
        # Latched / Transient Local QoS for mission reliability
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_vitality = self.create_publisher(TreeVitality, '/tree/vitality', latched_qos)
        self.pub_annotated_img = self.create_publisher(Image, '/tree/detection_image', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.sub_cam = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # ── Services ─────────────────────────────────────────────────────────
        self.srv_inspect = self.create_service(
            InspectTree,
            '/tree/inspect',
            self.handle_inspect_request
        )

        # ── Continuous Analysis Timer ─────────────────────────────────────────
        if self.auto_publish and self.rate_hz > 0:
            timer_period = 1.0 / self.rate_hz
            self.timer = self.create_timer(timer_period, self.continuous_tick)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🌲 MEER Rover Tree Vitality Subsystem Initialized")
        self.get_logger().info(f"📷 Listening on camera topic: {self.camera_topic}")
        self.get_logger().info(f"🧠 Classifier pipeline: {self.classifier_type.upper()}")
        self.get_logger().info(f"📁 Deliverables will be saved to: {self.output_dir}")
        self.get_logger().info("=" * 60)

    def image_callback(self, msg: Image):
        """Buffers the latest incoming camera frame."""
        try:
            self.latest_cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_header = msg.header
        except Exception:
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                if msg.encoding == 'rgb8':
                    self.latest_cv_frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                else:
                    self.latest_cv_frame = arr
                self.latest_header = msg.header
            except Exception as e2:
                self.get_logger().warn(f"Failed to decode image message: {e2}")

    def process_frame(self, frame_bgr: cv2.Mat):
        """Executes full detection -> ROI -> vitality classification pipeline."""
        # 1. Tree Detection and Localization
        detection = self.detector.detect(frame_bgr)
        self.latest_detection = detection

        # 2. Vitality Classification on Tree ROI
        vitality_res = self.classifier.classify(frame_bgr, detection.bbox)
        self.latest_result = vitality_res

        # 3. Construct Annotated Visualization Image
        annotated = detection.annotated_frame if detection.annotated_frame is not None else frame_bgr.copy()
        bx, by, bw, bh = detection.bbox

        # Banner color: Green for ALIVE, Red for DEAD, Grey for UNKNOWN
        banner_color = (0, 180, 0) if vitality_res.verdict == "ALIVE" else (0, 0, 200)
        if vitality_res.verdict == "UNKNOWN":
            banner_color = (120, 120, 120)

        # Header overlay banner
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 42), (20, 20, 20), -1)
        verdict_str = f"VERDICT: {vitality_res.verdict} | CONFIDENCE: {vitality_res.confidence*100:.1f}%"
        cv2.putText(
            annotated,
            verdict_str,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            banner_color,
            2,
            cv2.LINE_AA
        )

        metrics_str = f"VARI: {vitality_res.greenness_index:.3f} | Foliage: {vitality_res.foliage_ratio*100:.1f}%"
        cv2.putText(
            annotated,
            metrics_str,
            (annotated.shape[1] - 250, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (220, 220, 220),
            1,
            cv2.LINE_AA
        )

        return vitality_res, detection, annotated

    def continuous_tick(self):
        """Runs periodic analysis on buffered frames and publishes topics."""
        if self.latest_cv_frame is None:
            return

        vitality_res, detection, annotated = self.process_frame(self.latest_cv_frame)

        # Publish TreeVitality message
        msg = self._build_vitality_msg(vitality_res, detection)
        self.pub_vitality.publish(msg)

        # Publish Annotated Image (Safe from cv_bridge KeyError: 16 bug in ROS 2 Jazzy)
        try:
            annotated_msg = Image()
            if self.latest_header is not None:
                annotated_msg.header = self.latest_header
            else:
                annotated_msg.header.stamp = self.get_clock().now().to_msg()
                annotated_msg.header.frame_id = 'camera_optical_link'
            annotated_msg.height = int(annotated.shape[0])
            annotated_msg.width = int(annotated.shape[1])
            annotated_msg.encoding = 'bgr8'
            annotated_msg.is_bigendian = False
            annotated_msg.step = int(3 * annotated.shape[1])
            annotated_msg.data = bytes(np.ascontiguousarray(annotated, dtype=np.uint8))
            self.pub_annotated_img.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")

    def handle_inspect_request(self, request: InspectTree.Request, response: InspectTree.Response):
        """Service callback for on-demand inspection triggered at GPS Point 1."""
        self.get_logger().info("🔍 On-demand Tree Inspection requested via /tree/inspect service...")

        if self.latest_cv_frame is None:
            response.success = False
            response.message = "No camera frame received from sensor."
            self.get_logger().error(f"❌ {response.message}")
            return response

        # Run fresh analysis on latest frame
        vitality_res, detection, annotated = self.process_frame(self.latest_cv_frame)

        # Build ROS 2 message
        vitality_msg = self._build_vitality_msg(vitality_res, detection)
        response.vitality = vitality_msg
        response.success = True
        response.message = f"Inspection complete: {vitality_res.verdict} ({vitality_res.confidence*100:.1f}% confidence)"

        # Save deliverables if requested
        if request.save_snapshot:
            snapshot_path = os.path.join(self.output_dir, self.snapshot_file)
            cv2.imwrite(snapshot_path, annotated)
            self.get_logger().info(f"📸 Saved annotated tree inspection snapshot to: {snapshot_path}")

        # Always write the mandatory competition log: tree_verdict.log
        self.write_verdict_log(vitality_res, detection)

        # Publish to latched topic
        self.pub_vitality.publish(vitality_msg)

        self.get_logger().info(f"✅ {response.message}")
        return response

    def _build_vitality_msg(self, res: VitalityResult, det: DetectionResult) -> TreeVitality:
        """Constructs ROS 2 TreeVitality message."""
        msg = TreeVitality()
        if self.latest_header is not None:
            msg.header = self.latest_header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_link'

        msg.vitality = res.verdict
        msg.confidence = float(res.confidence)
        msg.tree_detected = bool(det.tree_detected)
        msg.foliage_ratio = float(res.foliage_ratio)
        msg.greenness_index = float(res.greenness_index)
        bx, by, bw, bh = det.bbox
        msg.bounding_box = [int(bx), int(by), int(bw), int(bh)]
        msg.details = str(res.details)
        return msg

    def write_verdict_log(self, res: VitalityResult, det: DetectionResult):
        """Generates the official tree_verdict.log deliverable."""
        log_path = os.path.join(self.output_dir, self.verdict_file)
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S')
        bx, by, bw, bh = det.bbox

        content = [
            "=" * 60,
            "NAGC 2026 — Official Tree Vitality Inspection Verdict",
            "=" * 60,
            f"Timestamp: {timestamp_str}",
            f"Target Location: GPS Point 1 (16.0m, 4.0m) [Tolerance: ±1.5m]",
            f"Tree Detected: {det.tree_detected} (Detection Confidence: {det.confidence*100:.1f}%)",
            f"Bounding Box ROI: [x={bx}, y={by}, width={bw}, height={bh}]",
            f"Tree Verdict: {res.verdict}",
            f"Confidence Score: {res.confidence * 100:.1f}%",
            f"Foliar Canopy Coverage: {res.foliage_ratio * 100:.2f}%",
            f"Vegetative Index (VARI): {res.greenness_index:.4f}",
            f"Sensor Analysis: {res.details}",
            "Sensor Modalities: Onboard RGB Optical Camera [Thermal/Multispectral ready]",
            "Inspection Status: SUCCESS",
            "=" * 60,
            ""
        ]

        with open(log_path, "w") as f:
            f.write("\n".join(content))

        self.get_logger().info(f"📄 Official tree verdict log written: {log_path}")


def main(args=None):
    rclpy.init(args=args)
    node = TreeVitalityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TreeVitalityNode shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# =============================================================================
# esp32_bridge.py — Dual-ESP32 Serial ROS 2 Bridge for MEER Rover
# =============================================================================
#
# Hardware Architecture:
#   1. Motor ESP32 (Actuator, default: /dev/ttyUSB0):
#      - Subscribes to /cmd_vel.
#      - Converts linear/angular velocity to differential/skid-steer commands.
#      - Transmits commands ONLY to motor_port (connected to Cytron MDDS30).
#      - Built-in watchdog timer sends stop command if /cmd_vel ceases.
#
#   2. Encoder ESP32 (Sensor, default: /dev/ttyUSB1):
#      - Dedicated background reading thread.
#      - Reads incoming tick packets ONLY from encoder_port.
#      - Calculates dead-reckoning odometry & wheel velocities.
#      - Publishes /odom (nav_msgs/Odometry) and TF (odom -> base_footprint).
#
# Resilience & Error Handling:
#   - Independent try/catch blocks and state machines for both serial ports.
#   - If one ESP32 disconnects, the node logs a specific error and continues
#     servicing the other port while attempting periodic auto-reconnection.
#
# Compatibility:
#   - Fully compatible with ROS 2 Humble & ROS 2 Jazzy.
# =============================================================================

import math
import threading
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import tf2_ros

try:
    import serial
except ImportError:
    serial = None


class DualESP32Bridge(Node):
    """
    ROS 2 Bridge Node for Dual-ESP32 Rover Architecture.
    Manages separate PySerial connections for Motor and Encoder microcontrollers.
    """

    def __init__(self) -> None:
        super().__init__('esp32_bridge')

        # ---------------------------------------------------------------------
        # 1. Declare Parameters
        # ---------------------------------------------------------------------
        # Serial ports
        self.declare_parameter('motor_port', '/dev/ttyUSB0')
        self.declare_parameter('encoder_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('reconnect_interval', 2.0)

        # Kinematics & Physical Parameters (from MEER CAD / URDF)
        self.declare_parameter('wheel_radius', 0.1155)    # meters (115.5 mm)
        self.declare_parameter('track_width', 0.8184)     # meters (818.4 mm)
        self.declare_parameter('ticks_per_rev', 2000.0)   # encoder ticks per full wheel turn

        # Inversion & Calibration Flags
        self.declare_parameter('invert_left_motor', False)
        self.declare_parameter('invert_right_motor', False)
        self.declare_parameter('invert_left_encoder', False)
        self.declare_parameter('invert_right_encoder', False)

        # Protocol & Commands
        self.declare_parameter('command_format', 'vel')   # 'vel' (m/s) or 'pwm' (-255..255)
        self.declare_parameter('max_wheel_speed', 2.0)    # m/s max for PWM mapping
        self.declare_parameter('pwm_max', 255)
        self.declare_parameter('cmd_vel_timeout', 0.5)    # seconds before watchdog stop

        # Frames & TF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        # ---------------------------------------------------------------------
        # 2. Retrieve Parameter Values
        # ---------------------------------------------------------------------
        self.motor_port: str = self.get_parameter('motor_port').value
        self.encoder_port: str = self.get_parameter('encoder_port').value
        self.baud_rate: int = int(self.get_parameter('baud_rate').value)
        self.serial_timeout: float = float(self.get_parameter('serial_timeout').value)
        self.reconnect_interval: float = float(self.get_parameter('reconnect_interval').value)

        self.wheel_radius: float = float(self.get_parameter('wheel_radius').value)
        self.track_width: float = float(self.get_parameter('track_width').value)
        self.ticks_per_rev: float = float(self.get_parameter('ticks_per_rev').value)

        self.inv_left_mot: bool = bool(self.get_parameter('invert_left_motor').value)
        self.inv_right_mot: bool = bool(self.get_parameter('invert_right_motor').value)
        self.inv_left_enc: bool = bool(self.get_parameter('invert_left_encoder').value)
        self.inv_right_enc: bool = bool(self.get_parameter('invert_right_encoder').value)

        self.command_format: str = str(self.get_parameter('command_format').value).lower()
        self.max_wheel_speed: float = float(self.get_parameter('max_wheel_speed').value)
        self.pwm_max: int = int(self.get_parameter('pwm_max').value)
        self.cmd_vel_timeout: float = float(self.get_parameter('cmd_vel_timeout').value)

        self.odom_frame: str = str(self.get_parameter('odom_frame').value)
        self.base_frame: str = str(self.get_parameter('base_frame').value)
        self.publish_tf: bool = bool(self.get_parameter('publish_tf').value)

        self.get_logger().info('====================================================')
        self.get_logger().info('      MEER Rover Dual-ESP32 Serial Bridge Node      ')
        self.get_logger().info('====================================================')
        self.get_logger().info(f'Motor Port   (Actuator): {self.motor_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'Encoder Port (Sensor)  : {self.encoder_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'Kinematics             : Radius={self.wheel_radius}m, Track={self.track_width}m')
        self.get_logger().info(f'Frames                 : odom="{self.odom_frame}" -> base="{self.base_frame}" (TF={self.publish_tf})')

        # ---------------------------------------------------------------------
        # 3. State & Synchronization Primitives
        # ---------------------------------------------------------------------
        self._motor_ser: Optional[serial.Serial] = None
        self._encoder_ser: Optional[serial.Serial] = None
        self._motor_connected = False
        self._encoder_connected = False

        self._motor_lock = threading.Lock()
        self._odom_lock = threading.Lock()
        self._running = True

        # Watchdog & Activity Tracking
        self._last_cmd_vel_time = 0.0
        self._motors_active = False

        # Odometry State Variables (Midpoint Runge-Kutta)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vtheta = 0.0

        self._prev_left_ticks: Optional[int] = None
        self._prev_right_ticks: Optional[int] = None
        self._prev_odom_time: Optional[float] = None

        # ---------------------------------------------------------------------
        # 4. Publishers, Subscribers & TF Broadcaster
        # ---------------------------------------------------------------------
        self._odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        # ---------------------------------------------------------------------
        # 5. Initialize Serial Connections
        # ---------------------------------------------------------------------
        if serial is None:
            self.get_logger().fatal('PySerial is NOT installed! Run: sudo apt install python3-serial')
            return

        self._open_motor_serial(log_failure=True)
        self._open_encoder_serial(log_failure=True)

        # ---------------------------------------------------------------------
        # 6. Start Encoder Reader Thread (Dedicated Sensor Reading)
        # ---------------------------------------------------------------------
        self._encoder_thread = threading.Thread(
            target=self._encoder_read_loop,
            name='EncoderSerialReader',
            daemon=True
        )
        self._encoder_thread.start()

        # ---------------------------------------------------------------------
        # 7. Timers: Watchdog & Reconnection Checker
        # ---------------------------------------------------------------------
        # Watchdog timer (20 Hz): safely stops motors if /cmd_vel ceases
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_callback)

        # Auto-reconnect timer (runs every reconnect_interval seconds)
        self._reconnect_timer = self.create_timer(self.reconnect_interval, self._reconnect_callback)

    # =========================================================================
    # Serial Port Management & Independent Resilience
    # =========================================================================

    def _open_motor_serial(self, log_failure: bool = True) -> bool:
        """Attempt connection to the Motor ESP32 actuator port."""
        with self._motor_lock:
            if self._motor_ser is not None and self._motor_ser.is_open:
                return True
            try:
                self._motor_ser = serial.Serial(
                    port=self.motor_port,
                    baudrate=self.baud_rate,
                    timeout=self.serial_timeout,
                    write_timeout=self.serial_timeout,
                )
                self._motor_connected = True
                self.get_logger().info(f'[MOTOR SERIAL] Connected successfully to {self.motor_port}')
                return True
            except (serial.SerialException, OSError) as exc:
                self._motor_connected = False
                self._motor_ser = None
                if log_failure:
                    self.get_logger().error(
                        f"[MOTOR SERIAL] Failed to connect to '{self.motor_port}': {exc}. "
                        "Motor commands will be buffered/dropped until reconnection."
                    )
                return False

    def _open_encoder_serial(self, log_failure: bool = True) -> bool:
        """Attempt connection to the Encoder ESP32 sensor port."""
        if self._encoder_ser is not None and self._encoder_ser.is_open:
            return True
        try:
            self._encoder_ser = serial.Serial(
                port=self.encoder_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
            )
            self._encoder_connected = True
            self.get_logger().info(f'[ENCODER SERIAL] Connected successfully to {self.encoder_port}')
            return True
        except (serial.SerialException, OSError) as exc:
            self._encoder_connected = False
            self._encoder_ser = None
            if log_failure:
                self.get_logger().error(
                    f"[ENCODER SERIAL] Failed to connect to '{self.encoder_port}': {exc}. "
                    "Odometry will not update until reconnection."
                )
            return False

    def _close_motor_serial(self) -> None:
        """Cleanly close the Motor port on error or shutdown."""
        with self._motor_lock:
            self._motor_connected = False
            if self._motor_ser is not None:
                try:
                    self._motor_ser.close()
                except Exception:
                    pass
                self._motor_ser = None

    def _close_encoder_serial(self) -> None:
        """Cleanly close the Encoder port on error or shutdown."""
        self._encoder_connected = False
        if self._encoder_ser is not None:
            try:
                self._encoder_ser.close()
            except Exception:
                pass
            self._encoder_ser = None

    def _reconnect_callback(self) -> None:
        """Periodically checks and attempts to reconnect disconnected ports."""
        if not self._motor_connected:
            if self._open_motor_serial(log_failure=False):
                self.get_logger().info(f'[MOTOR SERIAL] Reconnected to {self.motor_port}!')
        if not self._encoder_connected:
            if self._open_encoder_serial(log_failure=False):
                self.get_logger().info(f'[ENCODER SERIAL] Reconnected to {self.encoder_port}!')

    # =========================================================================
    # Motor ESP32 (Actuator): Write Callback & Kinematics
    # =========================================================================

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Receives geometry_msgs/Twist from Nav2 or teleoperation.
        Calculates differential velocities and transmits them ONLY to the Motor ESP32.
        """
        self._last_cmd_vel_time = time.time()
        self._motors_active = True

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential / 4WD Skid-Steer kinematics
        # v_left  = v - (omega * track_width / 2)
        # v_right = v + (omega * track_width / 2)
        v_left = linear_x - (angular_z * self.track_width / 2.0)
        v_right = linear_x + (angular_z * self.track_width / 2.0)

        # Apply motor direction inversion if configured
        if self.inv_left_mot:
            v_left = -v_left
        if self.inv_right_mot:
            v_right = -v_right

        self._send_motor_command(v_left, v_right)

    def _send_motor_command(self, v_left: float, v_right: float) -> None:
        """Formats and writes the command string/bytes ONLY to motor_port."""
        if not self._motor_connected or self._motor_ser is None:
            return

        if self.command_format == 'pwm':
            # Sign-magnitude PWM mapping (-pwm_max to +pwm_max)
            scale = self.pwm_max / max(self.max_wheel_speed, 1e-3)
            pwm_l = int(max(-self.pwm_max, min(self.pwm_max, v_left * scale)))
            pwm_r = int(max(-self.pwm_max, min(self.pwm_max, v_right * scale)))
            cmd_line = f"M,{pwm_l},{pwm_r}\n"
        else:
            # Default: Direct velocity commands in m/s (e.g. CMD,0.350,-0.120)
            cmd_line = f"CMD,{v_left:.3f},{v_right:.3f}\n"

        packet = cmd_line.encode('utf-8')

        with self._motor_lock:
            try:
                self._motor_ser.write(packet)
                self._motor_ser.flush()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"[MOTOR SERIAL] Write error on '{self.motor_port}': {exc}")
                self._close_motor_serial()

    def _send_motor_stop(self) -> None:
        """Sends an explicit zero-velocity / STOP packet to the Motor ESP32."""
        if not self._motor_connected or self._motor_ser is None:
            return

        if self.command_format == 'pwm':
            packet = b"M,0,0\n"
        else:
            packet = b"CMD,0.000,0.000\n"

        with self._motor_lock:
            try:
                self._motor_ser.write(packet)
                self._motor_ser.flush()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"[MOTOR SERIAL] Stop command failed on '{self.motor_port}': {exc}")
                self._close_motor_serial()

    def _watchdog_callback(self) -> None:
        """
        Watchdog safety check: if no new /cmd_vel was received within
        cmd_vel_timeout, command the Motor ESP32 to STOP immediately.
        """
        if self._motors_active:
            elapsed = time.time() - self._last_cmd_vel_time
            if elapsed > self.cmd_vel_timeout:
                self.get_logger().warn(
                    f'[MOTOR SERIAL] Watchdog triggered: no /cmd_vel for {elapsed:.2f}s. Halting motors.',
                    throttle_duration_sec=3.0
                )
                self._send_motor_stop()
                self._motors_active = False

    # =========================================================================
    # Encoder ESP32 (Sensor): Background Thread & Odometry Computation
    # =========================================================================

    def _encoder_read_loop(self) -> None:
        """
        Continuously reads incoming serial lines ONLY from encoder_port.
        Parses tick counts and updates odometry & TF.
        """
        self.get_logger().info('[ENCODER SERIAL] Reader thread active.')

        while self._running and rclpy.ok():
            if not self._encoder_connected or self._encoder_ser is None:
                time.sleep(0.2)
                continue

            try:
                # Read a newline-terminated line from the encoder ESP32
                raw_line = self._encoder_ser.readline()
                if not raw_line:
                    continue

                line = raw_line.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                self._process_encoder_line(line)

            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"[ENCODER SERIAL] Read error on '{self.encoder_port}': {exc}")
                self._close_encoder_serial()
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().warn(f"[ENCODER SERIAL] Unexpected error parsing line: {e}")

    def _parse_ticks(self, line: str) -> Optional[Tuple[int, int]]:
        """
        Robustly parses left and right tick values from various telemetry formats:
          - ENC:<left>,<right>
          - ENC <left> <right>
          - E <left> <right>
          - <left>,<right>
          - <left> <right>
        """
        # Strip prefixes if present
        clean = line
        for prefix in ['ENC:', 'enc:', 'ENC ', 'enc ', 'E ', 'e ']:
            if clean.startswith(prefix):
                clean = clean[len(prefix):].strip()
                break

        # Delimited by comma or whitespace
        if ',' in clean:
            tokens = [t.strip() for t in clean.split(',') if t.strip()]
        else:
            tokens = clean.split()

        if len(tokens) >= 2:
            try:
                left = int(tokens[0])
                right = int(tokens[1])
                return left, right
            except ValueError:
                return None
        return None

    def _process_encoder_line(self, line: str) -> None:
        """Decodes raw encoder ticks, updates kinematics, and publishes odometry."""
        parsed = self._parse_ticks(line)
        if parsed is None:
            return

        left_ticks, right_ticks = parsed

        if self.inv_left_enc:
            left_ticks = -left_ticks
        if self.inv_right_enc:
            right_ticks = -right_ticks

        now = time.time()
        ros_time = self.get_clock().now()

        with self._odom_lock:
            if self._prev_left_ticks is None or self._prev_right_ticks is None or self._prev_odom_time is None:
                # First reading: establish baseline tick references
                self._prev_left_ticks = left_ticks
                self._prev_right_ticks = right_ticks
                self._prev_odom_time = now
                return

            dt = now - self._prev_odom_time
            if dt <= 0.0:
                dt = 1e-4

            delta_l = left_ticks - self._prev_left_ticks
            delta_r = right_ticks - self._prev_right_ticks

            self._prev_left_ticks = left_ticks
            self._prev_right_ticks = right_ticks
            self._prev_odom_time = now

            # Distance moved per tick = (2 * pi * wheel_radius) / ticks_per_rev
            dist_per_tick = (2.0 * math.pi * self.wheel_radius) / max(self.ticks_per_rev, 1.0)
            dist_l = delta_l * dist_per_tick
            dist_r = delta_r * dist_per_tick

            # Differential kinematic displacement & yaw change
            delta_s = (dist_r + dist_l) / 2.0
            delta_yaw = (dist_r - dist_l) / max(self.track_width, 1e-3)

            # Midpoint Runge-Kutta integration for position
            mid_yaw = self.yaw + (delta_yaw / 2.0)
            self.x += delta_s * math.cos(mid_yaw)
            self.y += delta_s * math.sin(mid_yaw)
            self.yaw += delta_yaw

            # Normalize yaw to [-pi, pi]
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            # Velocities
            self.vx = delta_s / dt
            self.vtheta = delta_yaw / dt

            x = self.x
            y = self.y
            yaw = self.yaw
            vx = self.vx
            vtheta = self.vtheta

        # Publish Odometry & TF
        self._publish_odometry(ros_time, x, y, yaw, vx, vtheta)

    def _publish_odometry(
        self,
        stamp: rclpy.time.Time,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        vtheta: float
    ) -> None:
        """Publishes the nav_msgs/Odometry message and TF transform."""
        # Convert Euler yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # 1. Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Realistic pose covariance (x, y, z, roll, pitch, yaw)
        odom_msg.pose.covariance[0] = 0.001   # var(x)
        odom_msg.pose.covariance[7] = 0.001   # var(y)
        odom_msg.pose.covariance[14] = 1e6     # var(z) unmeasured
        odom_msg.pose.covariance[21] = 1e6     # var(roll) unmeasured
        odom_msg.pose.covariance[28] = 1e6     # var(pitch) unmeasured
        odom_msg.pose.covariance[35] = 0.01    # var(yaw)

        # Twist (linear & angular velocity)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = vtheta

        odom_msg.twist.covariance[0] = 0.001   # var(vx)
        odom_msg.twist.covariance[7] = 1e6     # var(vy)
        odom_msg.twist.covariance[14] = 1e6    # var(vz)
        odom_msg.twist.covariance[21] = 1e6    # var(wx)
        odom_msg.twist.covariance[28] = 1e6    # var(wy)
        odom_msg.twist.covariance[35] = 0.01   # var(wz)

        self._odom_pub.publish(odom_msg)

        # 2. TF Transform (odom -> base_footprint)
        if self.publish_tf:
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = stamp.to_msg()
            tf_stamped.header.frame_id = self.odom_frame
            tf_stamped.child_frame_id = self.base_frame

            tf_stamped.transform.translation.x = x
            tf_stamped.transform.translation.y = y
            tf_stamped.transform.translation.z = 0.0

            tf_stamped.transform.rotation.x = 0.0
            tf_stamped.transform.rotation.y = 0.0
            tf_stamped.transform.rotation.z = qz
            tf_stamped.transform.rotation.w = qw

            self._tf_broadcaster.sendTransform(tf_stamped)

    # =========================================================================
    # Lifecycle & Cleanup
    # =========================================================================

    def destroy_node(self) -> bool:
        """Safely stops motors and releases both PySerial ports on shutdown."""
        self.get_logger().info('Shutting down Dual-ESP32 Bridge Node...')
        self._running = False

        # Attempt to stop physical motors before exiting
        try:
            self._send_motor_stop()
        except Exception:
            pass

        self._close_motor_serial()
        self._close_encoder_serial()

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualESP32Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

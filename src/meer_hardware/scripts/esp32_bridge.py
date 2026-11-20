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
import socket
import threading
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
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
        self.declare_parameter('baud_rate', 115200)        # motor ESP baud
        self.declare_parameter('encoder_baud', 115200)     # encoder/GPS ESP baud
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('reconnect_interval', 2.0)

        # Kinematics & Physical Parameters (from MEER CAD / URDF)
        self.declare_parameter('wheel_radius', 0.1155)    # meters (115.5 mm)
        self.declare_parameter('track_width', 0.8184)     # meters (818.4 mm)
        self.declare_parameter('ticks_per_rev', 2000.0)   # fallback if left/right not set
        # Tooth-wheel encoders: left disc 14 teeth (1 broken), right disc 15 teeth
        self.declare_parameter('ticks_per_rev_left', 14.0)
        self.declare_parameter('ticks_per_rev_right', 15.0)
        # Single-channel encoders only count up -> direction taken from the motor command
        self.declare_parameter('encoder_signed', False)
        self.declare_parameter('max_ticks_per_sample', 40)  # bigger jump = noise / ESP reset

        # Inversion & Calibration Flags
        self.declare_parameter('invert_left_motor', False)
        self.declare_parameter('invert_right_motor', False)
        self.declare_parameter('invert_left_encoder', False)
        self.declare_parameter('invert_right_encoder', False)

        # Protocol & Commands
        # Motor ESP (NodeMCU) firmware protocol: "x,y" string, x = turn, y = drive,
        # both -100..100. ESP mixes left = y + x, right = y - x.
        #   motor_transport = 'serial' -> send "x,y\n" over the USB cable to motor_port (default,
        #                                works on any WiFi; needs updated rover_esp firmware)
        #   motor_transport = 'udp'    -> send "x,y" to esp_ip:esp_udp_port (Pi must be on the ESP's WiFi)
        self.declare_parameter('motor_transport', 'serial')
        self.declare_parameter('esp_ip', '192.168.1.82')
        self.declare_parameter('esp_udp_port', 4210)
        self.declare_parameter('max_wheel_speed', 1.0)     # m/s of a wheel at 100% command
        self.declare_parameter('min_cmd_percent', 40)      # deadband: smallest % that moves the rover (firmware: 40 -> 20% PWM)
        self.declare_parameter('heartbeat_hz', 10.0)       # resend rate (ESP watchdog = 1.5 s)
        self.declare_parameter('cmd_vel_timeout', 0.5)     # seconds before we command stop

        # Odometry source: 'encoder' | 'cmd_vel' | 'auto'
        #   auto -> use encoder ticks when they arrive, otherwise integrate the
        #           commanded velocity (open-loop) so Nav2 always has odom + TF.
        self.declare_parameter('odom_source', 'auto')

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
        self.encoder_baud: int = int(self.get_parameter('encoder_baud').value)
        self.serial_timeout: float = float(self.get_parameter('serial_timeout').value)
        self.reconnect_interval: float = float(self.get_parameter('reconnect_interval').value)

        self.wheel_radius: float = float(self.get_parameter('wheel_radius').value)
        self.track_width: float = float(self.get_parameter('track_width').value)
        self.ticks_per_rev: float = float(self.get_parameter('ticks_per_rev').value)
        tl = float(self.get_parameter('ticks_per_rev_left').value)
        tr = float(self.get_parameter('ticks_per_rev_right').value)
        self.ticks_left: float = tl if tl > 0 else self.ticks_per_rev
        self.ticks_right: float = tr if tr > 0 else self.ticks_per_rev
        self.encoder_signed: bool = bool(self.get_parameter('encoder_signed').value)
        self.max_ticks_per_sample: int = int(self.get_parameter('max_ticks_per_sample').value)

        self.inv_left_mot: bool = bool(self.get_parameter('invert_left_motor').value)
        self.inv_right_mot: bool = bool(self.get_parameter('invert_right_motor').value)
        self.inv_left_enc: bool = bool(self.get_parameter('invert_left_encoder').value)
        self.inv_right_enc: bool = bool(self.get_parameter('invert_right_encoder').value)

        self.motor_transport: str = str(self.get_parameter('motor_transport').value).lower()
        self.esp_ip: str = str(self.get_parameter('esp_ip').value)
        self.esp_udp_port: int = int(self.get_parameter('esp_udp_port').value)
        self.max_wheel_speed: float = float(self.get_parameter('max_wheel_speed').value)
        self.min_cmd_percent: int = int(self.get_parameter('min_cmd_percent').value)
        self.heartbeat_hz: float = float(self.get_parameter('heartbeat_hz').value)
        self.cmd_vel_timeout: float = float(self.get_parameter('cmd_vel_timeout').value)
        self.odom_source: str = str(self.get_parameter('odom_source').value).lower()

        self.odom_frame: str = str(self.get_parameter('odom_frame').value)
        self.base_frame: str = str(self.get_parameter('base_frame').value)
        self.publish_tf: bool = bool(self.get_parameter('publish_tf').value)

        self.get_logger().info('====================================================')
        self.get_logger().info('      MEER Rover Dual-ESP32 Serial Bridge Node      ')
        self.get_logger().info('====================================================')
        if self.motor_transport == 'udp':
            self.get_logger().info(f'Motor ESP    (Actuator): UDP {self.esp_ip}:{self.esp_udp_port} ("x,y" -100..100)')
        else:
            self.get_logger().info(f'Motor Port   (Actuator): {self.motor_port} @ {self.baud_rate} baud ("x,y")')
        self.get_logger().info(f'Odometry source        : {self.odom_source}')
        self.get_logger().info(f'Encoder Port (Sensor)  : {self.encoder_port} @ {self.encoder_baud} baud')
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
        self._cmd_x = 0          # current turn command  (-100..100)
        self._cmd_y = 0          # current drive command (-100..100)
        self._cmd_lin = 0.0      # last commanded linear.x  (for open-loop odom)
        self._cmd_ang = 0.0      # last commanded angular.z (for open-loop odom)
        self._dir_left = 1.0     # wheel direction for unsigned encoders (+1 fwd / -1 back)
        self._dir_right = 1.0
        self._udp_sock: Optional[socket.socket] = None
        self._last_udp_err_log = 0.0
        self._last_encoder_time = 0.0
        self._last_raw_log = 0.0
        self._last_cmd_odom_time: Optional[float] = None

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
        self._gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._heading_pub = self.create_publisher(Float64, '/compass/heading', 10)
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

        if self.motor_transport == 'udp':
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._motor_connected = True
        else:
            self._open_motor_serial(log_failure=True)
        if self.odom_source != 'cmd_vel':
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
        # Heartbeat + watchdog: resend the current command continuously so the
        # ESP 1.5 s watchdog never trips while driving; send 0,0 when /cmd_vel stops.
        self._watchdog_timer = self.create_timer(
            1.0 / max(self.heartbeat_hz, 1.0), self._watchdog_callback)

        # Open-loop odometry fallback (20 Hz)
        self._cmd_odom_timer = self.create_timer(0.05, self._cmd_odom_callback)

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
                ser = serial.Serial()
                ser.port = self.motor_port
                ser.baudrate = self.baud_rate
                ser.timeout = self.serial_timeout
                ser.write_timeout = self.serial_timeout
                # Do not toggle DTR/RTS: on NodeMCU that resets / holds the ESP in reset
                ser.dtr = False
                ser.rts = False
                ser.open()
                self._motor_ser = ser
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
            ser = serial.Serial()
            ser.port = self.encoder_port
            ser.baudrate = self.encoder_baud
            ser.timeout = self.serial_timeout
            ser.dtr = False
            ser.rts = False
            ser.open()
            self._encoder_ser = ser
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
        if self.motor_transport != 'udp' and not self._motor_connected:
            if self._open_motor_serial(log_failure=False):
                self.get_logger().info(f'[MOTOR SERIAL] Reconnected to {self.motor_port}!')
        if self.odom_source != 'cmd_vel' and not self._encoder_connected:
            if self._open_encoder_serial(log_failure=False):
                self.get_logger().info(f'[ENCODER SERIAL] Reconnected to {self.encoder_port}!')

    # =========================================================================
    # Motor ESP32 (Actuator): Write Callback & Kinematics
    # =========================================================================

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Receives geometry_msgs/Twist from Nav2 / teleop / mission runner and
        converts it to the motor ESP "x,y" protocol (-100..100).
        """
        self._last_cmd_vel_time = time.time()
        self._motors_active = True

        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self._cmd_lin = linear_x
        self._cmd_ang = angular_z

        # Differential / skid-steer kinematics (ROS: +angular.z = turn LEFT)
        v_left = linear_x - (angular_z * self.track_width / 2.0)
        v_right = linear_x + (angular_z * self.track_width / 2.0)

        # Remember physical wheel direction for the single-channel encoders.
        # When the command is ~0 keep the last direction (rover may still coast).
        if abs(v_left) > 1e-3:
            self._dir_left = 1.0 if v_left > 0 else -1.0
        if abs(v_right) > 1e-3:
            self._dir_right = 1.0 if v_right > 0 else -1.0

        if self.inv_left_mot:
            v_left = -v_left
        if self.inv_right_mot:
            v_right = -v_right

        # Wheel speed -> percent
        max_v = max(self.max_wheel_speed, 1e-3)
        left_pct = v_left / max_v * 100.0
        right_pct = v_right / max_v * 100.0

        # Keep both wheels inside -100..100 while preserving the turn ratio
        peak = max(abs(left_pct), abs(right_pct))
        if peak > 100.0:
            left_pct *= 100.0 / peak
            right_pct *= 100.0 / peak

        # Deadband compensation: tiny commands would not move a heavy rover
        left_pct = self._apply_deadband(left_pct)
        right_pct = self._apply_deadband(right_pct)

        # ESP mixes left = y + x, right = y - x  ->  invert that mixing
        y = (left_pct + right_pct) / 2.0
        x = (left_pct - right_pct) / 2.0

        self._cmd_x = int(round(max(-100.0, min(100.0, x))))
        self._cmd_y = int(round(max(-100.0, min(100.0, y))))
        self._send_motor_command(self._cmd_x, self._cmd_y)

    def _apply_deadband(self, pct: float) -> float:
        if abs(pct) < 1.0:
            return 0.0
        floor = float(self.min_cmd_percent)
        if abs(pct) < floor:
            return math.copysign(floor, pct)
        return pct

    def _send_motor_command(self, x: int, y: int) -> None:
        """Sends "x,y" to the Motor ESP over UDP (default) or USB serial."""
        payload = f"{x},{y}"

        if self.motor_transport == 'udp':
            if self._udp_sock is None:
                return
            try:
                self._udp_sock.sendto(payload.encode('ascii'), (self.esp_ip, self.esp_udp_port))
            except OSError as exc:
                now = time.time()
                if now - self._last_udp_err_log > 3.0:
                    self._last_udp_err_log = now
                    self.get_logger().error(
                        f'[MOTOR UDP] send to {self.esp_ip}:{self.esp_udp_port} failed: {exc} '
                        '(is the Pi on the same WiFi as the ESP?)')
            return

        if not self._motor_connected or self._motor_ser is None:
            return
        with self._motor_lock:
            try:
                self._motor_ser.write((payload + "\n").encode('ascii'))
                self._motor_ser.flush()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"[MOTOR SERIAL] Write error on '{self.motor_port}': {exc}")
                self._close_motor_serial()

    def _send_motor_stop(self) -> None:
        """Sends an explicit 0,0 STOP packet to the Motor ESP."""
        self._cmd_x = 0
        self._cmd_y = 0
        self._cmd_lin = 0.0
        self._cmd_ang = 0.0
        self._send_motor_command(0, 0)

    def _watchdog_callback(self) -> None:
        """
        Runs at heartbeat_hz. While /cmd_vel is fresh it re-sends the last
        command (keeps the ESP watchdog fed). If /cmd_vel stops for longer than
        cmd_vel_timeout it sends STOP (0,0) continuously.
        """
        if self._motors_active and (time.time() - self._last_cmd_vel_time) > self.cmd_vel_timeout:
            self.get_logger().warn(
                '[MOTOR] Watchdog: no /cmd_vel received. Halting motors.',
                throttle_duration_sec=3.0
            )
            self._motors_active = False
            self._send_motor_stop()
            return

        if self._motors_active:
            self._send_motor_command(self._cmd_x, self._cmd_y)
        else:
            self._send_motor_command(0, 0)

    def _use_encoder_odom(self) -> bool:
        if self.odom_source == 'encoder':
            return True
        if self.odom_source == 'cmd_vel':
            return False
        return (time.time() - self._last_encoder_time) < 1.0   # auto

    def _cmd_odom_callback(self) -> None:
        """Open-loop odometry from commanded velocity (fallback when no encoder data)."""
        now = time.time()
        if self._use_encoder_odom():
            self._last_cmd_odom_time = None
            return
        if self._last_cmd_odom_time is None:
            self._last_cmd_odom_time = now
            self.get_logger().warn(
                '[ODOM] No encoder data -> using open-loop odometry from /cmd_vel.',
                throttle_duration_sec=10.0)
            return
        dt = now - self._last_cmd_odom_time
        self._last_cmd_odom_time = now

        v = self._cmd_lin if self._motors_active else 0.0
        w = self._cmd_ang if self._motors_active else 0.0
        with self._odom_lock:
            mid_yaw = self.yaw + w * dt / 2.0
            self.x += v * dt * math.cos(mid_yaw)
            self.y += v * dt * math.sin(mid_yaw)
            self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
            self.vx, self.vtheta = v, w
            x, y, yaw = self.x, self.y, self.yaw
        self._publish_odometry(self.get_clock().now(), x, y, yaw, v, w)

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
                if not self._running:
                    break   # port closed during shutdown
                self.get_logger().warn(
                    f"[ENCODER SERIAL] Unexpected error: {e}", throttle_duration_sec=5.0)
                time.sleep(0.1)

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
        if line.startswith('#'):
            return  # info line from the ESP
        if line.startswith('GPS:'):
            self._process_gps_line(line[4:])
            return

        parsed = self._parse_ticks(line)
        if parsed is None:
            now = time.time()
            if now - self._last_raw_log > 5.0:
                self._last_raw_log = now
                self.get_logger().warn(f'[ENCODER SERIAL] Unrecognised line (expected "L,R" ticks): {line!r}')
            return
        self._last_encoder_time = time.time()

        left_ticks, right_ticks = parsed
        now = time.time()
        ros_time = self.get_clock().now()

        with self._odom_lock:
            if (self._prev_left_ticks is None or self._prev_right_ticks is None
                    or self._prev_odom_time is None or (now - self._prev_odom_time) > 1.0):
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

            if not self.encoder_signed:
                # Counts only go up. A negative or huge jump means the ESP
                # restarted or a noise burst -> skip this sample.
                if (delta_l < 0 or delta_r < 0
                        or delta_l > self.max_ticks_per_sample
                        or delta_r > self.max_ticks_per_sample):
                    self.get_logger().warn(
                        f'[ENCODER] Ignoring jump dL={delta_l} dR={delta_r} (ESP reset/noise).',
                        throttle_duration_sec=5.0)
                    return
                delta_l *= self._dir_left
                delta_r *= self._dir_right

            if self.inv_left_enc:
                delta_l = -delta_l
            if self.inv_right_enc:
                delta_r = -delta_r

            # Distance per tick = wheel circumference / ticks per revolution
            circumference = 2.0 * math.pi * self.wheel_radius
            dist_l = delta_l * circumference / max(self.ticks_left, 1.0)
            dist_r = delta_r * circumference / max(self.ticks_right, 1.0)

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

    def _process_gps_line(self, payload: str) -> None:
        """Parses 'yaw,lat,lng,sats,fix' and publishes /gps/fix and /compass/heading."""
        parts = [p.strip() for p in payload.split(',')]
        if len(parts) < 3:
            return
        try:
            yaw_deg = float(parts[0])
            lat = float(parts[1])
            lng = float(parts[2])
            sats = int(parts[3]) if len(parts) > 3 else 0
            fix = int(parts[4]) if len(parts) > 4 else int(lat != 0.0 or lng != 0.0)
        except ValueError:
            return

        stamp = self.get_clock().now().to_msg()

        heading = Float64()
        heading.data = yaw_deg
        self._heading_pub.publish(heading)

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps_link'
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.status.status = NavSatStatus.STATUS_FIX if fix else NavSatStatus.STATUS_NO_FIX
        msg.latitude = lat
        msg.longitude = lng
        msg.altitude = 0.0
        # NEO-6M: roughly 2.5 m horizontal accuracy with a fix
        var = 6.25 if fix else 1e6
        msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var * 4]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._gps_pub.publish(msg)
        if sats or fix:
            self.get_logger().info(
                f'[GPS] fix={fix} sats={sats} lat={lat:.6f} lng={lng:.6f} yaw={yaw_deg:.1f}',
                throttle_duration_sec=10.0)

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
        if self._udp_sock is not None:
            self._udp_sock.close()

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

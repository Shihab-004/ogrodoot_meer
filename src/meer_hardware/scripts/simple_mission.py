#!/usr/bin/env python3
"""
simple_mission.py — EMERGENCY autonomous waypoint mission (no Nav2 / LiDAR / camera).

Needs only:  ros2 launch meer_bringup real.launch.py launch_nav:=false launch_lidar:=false launch_camera:=false
(esp32_bridge publishes /odom and drives the motors over WiFi.)

The rover:  turns toward each waypoint -> drives to it -> waits -> next ... -> returns to base (0,0).
Coordinates are in meters from the start pose:  x = forward,  y = left (+) / right (-).

Usage:
  python3 simple_mission.py 5,0                 # one waypoint, then back to base
  python3 simple_mission.py 5,0 10,2            # two waypoints, then back to base
  python3 simple_mission.py 5,0 --no-return     # stay at the last waypoint
  python3 simple_mission.py 5,0 --speed 0.4 --wait 5
  python3 simple_mission.py 5,0 --avoid          # use LiDAR (/lidar/scan) to go around obstacles

Ctrl+C = immediate stop.
"""

import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SimpleMission(Node):

    def __init__(self, args):
        super().__init__('simple_mission')
        self.args = args
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # LiDAR sectors (meters): closest obstacle front / front-left / front-right
        self.front = self.left = self.right = float('inf')
        self.last_scan = 0.0
        if args.avoid or args.check_lidar:
            self.create_subscription(LaserScan, args.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        log_dir = os.path.expanduser('~/autonomous_meer/log/mission_submissions')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(os.path.join(log_dir, 'onboard_run.log'), 'a')

    # ------------------------------------------------------------------
    def log(self, text):
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        self.get_logger().info(text)
        self.log_file.write(line + '\n')
        self.log_file.flush()

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def _scan_cb(self, msg):
        off = math.radians(self.args.lidar_yaw)
        front = left = right = float('inf')
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max and r > self.args.self_range:
                ang = math.atan2(math.sin(a + off), math.cos(a + off))  # 0 = rover forward, + = left
                deg = math.degrees(ang)
                if abs(deg) <= 30:
                    front = min(front, r)
                elif 30 < deg <= 90:
                    left = min(left, r)
                elif -90 <= deg < -30:
                    right = min(right, r)
            a += msg.angle_increment
        self.front, self.left, self.right = front, left, right
        self.last_scan = time.time()

    def scan_ok(self):
        return self.args.avoid and (time.time() - self.last_scan) < 1.0

    def avoid(self, label):
        """Obstacle ahead: turn toward the freer side until the front is clear, then drive past it."""
        a = self.args
        side = 1.0 if self.left >= self.right else -1.0
        self.log(f'{label}: OBSTACLE {self.front:.2f} m ahead -> going around on the '
                 f'{"LEFT" if side > 0 else "RIGHT"} (L={self.left:.1f} R={self.right:.1f})')
        self.stop(0.3)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 8.0:          # turn until clear
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.front > a.clear_dist:
                break
            self.send(0.0, side * a.turn_speed)
        self.stop(0.2)
        sx, sy = self.x, self.y
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 10.0:         # drive past the obstacle
            rclpy.spin_once(self, timeout_sec=0.05)
            if math.hypot(self.x - sx, self.y - sy) >= a.bypass_dist:
                break
            if self.scan_ok() and self.front < a.stop_dist:
                self.stop(0.2)
                return  # something new ahead -> main loop will avoid again
            self.send(min(a.speed, 0.4), 0.0)
        self.stop(0.2)

    def send(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

    def stop(self, seconds=0.5):
        end = time.time() + seconds
        while time.time() < end:
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------------
    def go_to(self, gx, gy, label):
        a = self.args
        self.log(f'{label}: going to ({gx:.2f}, {gy:.2f}) from ({self.x:.2f}, {self.y:.2f})')
        start = time.time()
        turning = True
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist < a.tolerance:
                self.stop()
                self.log(f'{label}: REACHED ({self.x:.2f}, {self.y:.2f})  error={dist:.2f} m')
                return True
            if time.time() - start > a.timeout:
                self.stop()
                self.log(f'{label}: TIMEOUT at ({self.x:.2f}, {self.y:.2f}), {dist:.2f} m left')
                return False

            if self.scan_ok() and self.front < a.stop_dist and dist > a.stop_dist:
                self.avoid(label)
                turning = True
                continue

            err = math.atan2(dy, dx) - self.yaw
            err = math.atan2(math.sin(err), math.cos(err))

            # 1) turn in place until roughly facing the goal, 2) drive with steering
            if turning and abs(err) < 0.12:
                turning = False
            elif not turning and abs(err) > 0.5:
                turning = True

            if turning:
                w = max(a.turn_speed * 0.5, min(a.turn_speed, abs(err) * 1.2))
                self.send(0.0, math.copysign(w, err))
            else:
                lin = min(a.speed, max(0.2, dist * 0.6))
                if self.scan_ok() and self.front < a.slow_dist:
                    lin = min(lin, 0.3)  # slow down near obstacles
                self.send(lin, max(-0.6, min(0.6, 1.5 * err)))

    def run(self, waypoints):
        self.log('=' * 50)
        self.log(f'SIMPLE MISSION START  waypoints={waypoints}  return={not self.args.no_return}')
        t0 = time.time()
        while not self.have_odom and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have_odom:
            self.log('ERROR: no /odom. Is "ros2 launch meer_bringup real.launch.py" running?')
            return
        if self.args.avoid:
            t1 = time.time()
            while self.last_scan == 0.0 and time.time() - t1 < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_scan == 0.0:
                self.log(f'WARNING: no LiDAR data on {self.args.scan_topic} -> driving WITHOUT obstacle avoidance!')
            else:
                self.log(f'LiDAR OK: front={self.front:.1f} m left={self.left:.1f} m right={self.right:.1f} m')
        # Start pose = base station
        bx, by = self.x, self.y
        for i, (wx, wy) in enumerate(waypoints, 1):
            self.go_to(bx + wx, by + wy, f'WAYPOINT {i}')
            self.log(f'WAYPOINT {i}: holding {self.args.wait:.0f} s')
            self.stop(self.args.wait)
        if not self.args.no_return:
            self.go_to(bx, by, 'RETURN TO BASE')
        self.stop(1.0)
        self.log(f'MISSION COMPLETE in {time.time() - t0:.0f} s')


def parse_waypoints(items):
    wps = []
    for it in items:
        try:
            x, y = it.split(',')
            wps.append((float(x), float(y)))
        except ValueError:
            sys.exit(f'Bad waypoint "{it}" — use x,y  e.g. 5,0 or 10,-2.5')
    return wps


def main():
    p = argparse.ArgumentParser(description='Emergency autonomous waypoint mission')
    p.add_argument('waypoints', nargs='*', help='x,y in meters (x forward, y left)')
    p.add_argument('--speed', type=float, default=0.5, help='max forward speed m/s (default 0.5)')
    p.add_argument('--turn-speed', type=float, default=0.8, help='max turn speed rad/s (default 0.8)')
    p.add_argument('--tolerance', type=float, default=0.5, help='waypoint reached radius m (default 0.5)')
    p.add_argument('--wait', type=float, default=5.0, help='seconds to hold at each waypoint (default 5)')
    p.add_argument('--timeout', type=float, default=180.0, help='seconds per waypoint (default 180)')
    p.add_argument('--no-return', action='store_true', help='do not return to base')
    p.add_argument('--avoid', action='store_true', help='use LiDAR to go around obstacles')
    p.add_argument('--scan-topic', default='/lidar/scan')
    p.add_argument('--stop-dist', type=float, default=1.3, help='obstacle closer than this -> avoid (m)')
    p.add_argument('--slow-dist', type=float, default=2.5, help='slow down when obstacle closer (m)')
    p.add_argument('--clear-dist', type=float, default=2.2, help='front considered clear beyond (m)')
    p.add_argument('--bypass-dist', type=float, default=1.8, help='drive this far past an obstacle (m)')
    p.add_argument('--self-range', type=float, default=0.6, help='ignore hits closer (rover body) (m)')
    p.add_argument('--lidar-yaw', type=float, default=0.0,
                   help='rotate scan so 0 deg = rover forward (try 180 if front/back are swapped)')
    p.add_argument('--check-lidar', action='store_true', help='only print front/left/right distances')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = SimpleMission(args)
    try:
        if args.check_lidar:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                print(f'front={node.front:5.2f}  left={node.left:5.2f}  right={node.right:5.2f}  '
                      f'(scan age {time.time() - node.last_scan:4.1f}s)', flush=True)
                time.sleep(0.3)
        if not args.waypoints:
            sys.exit('Give at least one waypoint, e.g. 5,0')
        node.run(parse_waypoints(args.waypoints))
    except KeyboardInterrupt:
        node.log('STOPPED by operator (Ctrl+C)')
    finally:
        try:
            for _ in range(10):
                node.send(0.0, 0.0)
                time.sleep(0.05)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

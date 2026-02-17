#!/usr/bin/env python3
"""
waypoint_follower_node.py

Navigates through waypoints using GPS position + BNO055 compass heading.
Uses your existing motor command table and publishes to the UART node.

Subscribes:
  nav/waypoints          (rover_project/WaypointList)  — from path planner
  can/gps                (rover_project/GpsFix)        — current position
  can/imu_orientation    (rover_project/ImuOrientation) — compass heading
  can/proximity_sensors  (rover_project/Proximity)      — obstacle safety

Publishes:
  nav/drive_cmd          (std_msgs/Int32)  — motor command for UART node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rover_project.msg import GpsFix, ImuOrientation, WaypointList, Proximity
import math
import time


# Motor command table (matches Arduino / uart_node)
CMD_OFF                 = 0
CMD_AI                  = 1
CMD_STOP                = 2
CMD_FORWARD_STRAIGHT    = 3
CMD_FORWARD_RIGHT       = 4
CMD_FORWARD_LEFT        = 5
CMD_BACKWARD_STRAIGHT   = 6
CMD_BACKWARD_RIGHT      = 7
CMD_BACKWARD_LEFT       = 8
CMD_RIGHT               = 9
CMD_LEFT                = 10


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # Parameters
        self.declare_parameter('waypoint_reached_m', 2.5)
        self.declare_parameter('heading_tolerance_deg', 15.0)
        self.declare_parameter('sharp_turn_deg', 45.0)
        self.declare_parameter('command_rate_hz', 5.0)
        self.declare_parameter('gps_timeout_s', 5.0)
        self.declare_parameter('min_fix_type', 2)
        self.declare_parameter('obstacle_stop_mm', 400)    # front obstacle emergency stop
        self.declare_parameter('heading_offset_deg', 0.0)  # BNO055 mounting offset

        self.wp_threshold = self.get_parameter('waypoint_reached_m').value
        self.heading_tol = self.get_parameter('heading_tolerance_deg').value
        self.sharp_turn = self.get_parameter('sharp_turn_deg').value
        self.gps_timeout = self.get_parameter('gps_timeout_s').value
        self.min_fix_type = self.get_parameter('min_fix_type').value
        self.obstacle_stop_mm = self.get_parameter('obstacle_stop_mm').value
        self.heading_offset = self.get_parameter('heading_offset_deg').value
        cmd_rate = self.get_parameter('command_rate_hz').value

        # State
        self.waypoint_lats = []
        self.waypoint_lons = []
        self.current_wp_index = 0
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None    # degrees, 0=North CW from BNO055
        self.fix_type = 0
        self.last_gps_time = time.time()  # Initialize to current time to allow timeout period
        self.active = False
        self.obstacle_front_mm = 9999
        self.cliff_detected = False
        self.last_cmd = CMD_STOP

        # Publisher
        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)

        # Subscribers
        self.wp_sub = self.create_subscription(
            WaypointList, 'nav/waypoints', self.waypoint_callback, 10)
        self.gps_sub = self.create_subscription(
            GpsFix, 'can/gps', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(
            ImuOrientation, 'can/imu_orientation', self.imu_callback, 10)
        self.prox_sub = self.create_subscription(
            Proximity, 'can/proximity_sensors', self.proximity_callback, 10)

        # Control loop
        self.create_timer(1.0 / cmd_rate, self.control_loop)

        self.get_logger().info("Waypoint follower started")

    #  CALLBACKS

    def waypoint_callback(self, msg: WaypointList):
        self.waypoint_lats = list(msg.latitudes)
        self.waypoint_lons = list(msg.longitudes)
        self.current_wp_index = 1  # skip WP0 (our start position)

        if msg.total_waypoints > 1:
            self.active = True
            self.get_logger().info(
                f"Received {msg.total_waypoints} waypoints — navigating")
        else:
            self.active = False
            self.get_logger().warn("Empty or single-point path received")

    def gps_callback(self, msg: GpsFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.fix_type = msg.fix_type
        self.last_gps_time = time.time()

    def imu_callback(self, msg: ImuOrientation):
        # BNO055 IMU_Heading is already in degrees (0-360, compass convention)
        # Apply mounting offset if the IMU isn't aligned with rover forward
        self.current_heading = (msg.heading + self.heading_offset) % 360.0

    def proximity_callback(self, msg: Proximity):
        if msg.front_valid:
            self.obstacle_front_mm = msg.proximity_front
        self.cliff_detected = msg.cliff_detected

    #  CONTROL LOOP

    def control_loop(self):
        if not self.active:
            return

        # ── Safety: cliff or close obstacle → STOP ──────────────────────
        if self.cliff_detected:
            self.get_logger().warn("CLIFF detected — STOP")
            self.send_cmd(CMD_STOP)
            return

        if self.obstacle_front_mm < self.obstacle_stop_mm:
            self.get_logger().warn(
                f"Obstacle at {self.obstacle_front_mm}mm — STOP")
            self.send_cmd(CMD_STOP)
            return

        # Sensor readiness
        if self.current_lat is None or self.current_heading is None:
            self.get_logger().warn("Waiting for GPS + IMU...", once=True)
            return

        if time.time() - self.last_gps_time > self.gps_timeout:
            self.get_logger().warn("GPS stale — STOP")
            self.send_cmd(CMD_STOP)
            return

        if self.fix_type < self.min_fix_type:
            self.get_logger().warn(
                f"Poor GPS fix (type={self.fix_type}) — STOP", once=True)
            self.send_cmd(CMD_STOP)
            return

        # Check if all waypoints done
        if self.current_wp_index >= len(self.waypoint_lats):
            self.get_logger().info("✓ GOAL REACHED!")
            self.send_cmd(CMD_STOP)
            self.active = False
            return

        # Navigate to current waypoint
        tgt_lat = self.waypoint_lats[self.current_wp_index]
        tgt_lon = self.waypoint_lons[self.current_wp_index]

        dist = self.haversine(
            self.current_lat, self.current_lon, tgt_lat, tgt_lon)
        bearing = self.bearing(
            self.current_lat, self.current_lon, tgt_lat, tgt_lon)

        # Waypoint reached?
        if dist < self.wp_threshold:
            self.get_logger().info(
                f"  ✓ WP{self.current_wp_index} reached ({dist:.1f}m)")
            self.current_wp_index += 1
            return

        # Heading error
        err = self.angle_diff(self.current_heading, bearing)
        abs_err = abs(err)

        if abs_err > self.sharp_turn:
            cmd = CMD_RIGHT if err > 0 else CMD_LEFT
        elif abs_err > self.heading_tol:
            cmd = CMD_FORWARD_RIGHT if err > 0 else CMD_FORWARD_LEFT
        else:
            cmd = CMD_FORWARD_STRAIGHT

        self.send_cmd(cmd)

        self.get_logger().info(
            f"  WP{self.current_wp_index}: {dist:.1f}m | "
            f"bearing={bearing:.0f}° heading={self.current_heading:.0f}° "
            f"err={err:.0f}° → cmd={cmd}")

    #  HELPERS

    def send_cmd(self, cmd: int):
        if cmd != self.last_cmd:
            msg = Int32()
            msg.data = cmd
            self.cmd_pub.publish(msg)
            self.last_cmd = cmd

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2) -> float:
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2) -> float:
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
        return math.degrees(math.atan2(x, y)) % 360.0

    @staticmethod
    def angle_diff(current, target) -> float:
        """Positive = turn right, negative = turn left. Range (-180, 180]"""
        return (target - current + 180) % 360 - 180


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
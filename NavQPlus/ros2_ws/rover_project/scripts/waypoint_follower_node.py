#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from rover_project.msg import GpsFix, ImuOrientation, WaypointList
import math
import time

# Command Table
CMD_STOP              = 2
CMD_FORWARD_STRAIGHT  = 3
CMD_FORWARD_RIGHT     = 4
CMD_FORWARD_LEFT      = 5
CMD_BACKWARD_STRAIGHT = 6
CMD_BACKWARD_RIGHT    = 7
CMD_BACKWARD_LEFT     = 8
CMD_RIGHT             = 9
CMD_LEFT              = 10


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # --- Parameters ---
        self.declare_parameter('waypoint_reached_m', 2.5)
        self.declare_parameter('heading_tolerance_deg', 60.0)
        self.declare_parameter('sharp_turn_deg', 45.0)
        self.declare_parameter('heading_hysteresis_deg', 10.0)
        self.declare_parameter('vision_assist_max_heading_err_deg', 35.0)
        self.declare_parameter('command_rate_hz', 5.0)
        self.declare_parameter('invert_drive', True)

        # Optional GPS parameters (kept for compatibility)
        self.declare_parameter('gps_timeout_s', 5.0)
        self.declare_parameter('min_fix_type', 2)
        self.declare_parameter('obstacle_stop_mm', 400)
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('max_h_acc_m', 10.0)

        # --- Get parameters ---
        self.wp_threshold = self.get_parameter('waypoint_reached_m').value
        self.heading_tol = self.get_parameter('heading_tolerance_deg').value
        self.sharp_turn_deg = self.get_parameter('sharp_turn_deg').value
        self.heading_hysteresis = self.get_parameter('heading_hysteresis_deg').value
        self.vision_assist_max_heading_err = self.get_parameter(
            'vision_assist_max_heading_err_deg').value
        self.invert_drive = self.get_parameter('invert_drive').value
        cmd_rate = self.get_parameter('command_rate_hz').value

        # --- State ---
        self.waypoint_lats = []
        self.waypoint_lons = []
        self.current_wp_index = 0

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.safety_override = False
        self.active = False
        self.last_cmd = CMD_STOP
        self.last_logical_cmd = CMD_STOP  # Added to fix hysteresis bug

        # Vision placeholders (safe defaults)
        self.last_vision_time = 0.0
        self.vision_error = 9999.0

        # --- ROS Interfaces ---
        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)

        self.create_subscription(WaypointList, 'nav/waypoints',
                                 self.waypoint_callback, 10)
        self.create_subscription(GpsFix, 'can/gps',
                                 self.gps_callback, 10)
        self.create_subscription(ImuOrientation, 'can/imu_orientation',
                                 self.imu_callback, 10)
        self.create_subscription(Bool, 'safety/override_active',
                                 self.safety_override_callback, 1)

        self.create_timer(1.0 / cmd_rate, self.control_loop)

        self.get_logger().info("Waypoint Follower Node Started")

    # --------------------------------------------------

    def waypoint_callback(self, msg):
        self.waypoint_lats = list(msg.latitudes)
        self.waypoint_lons = list(msg.longitudes)
        self.current_wp_index = 0  # Fixed: Start at the first waypoint
        self.active = len(self.waypoint_lats) > 0  # Fixed: Check if any waypoints exist

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def imu_callback(self, msg):
        self.current_heading = msg.heading

    def safety_override_callback(self, msg):
        self.safety_override = msg.data

    # --------------------------------------------------

    def control_loop(self):

        # If safety override is active, stay silent
        if self.safety_override:
            return

        if not self.active or self.current_lat is None or self.current_heading is None:
            self.send_cmd(CMD_STOP)
            return

        if self.current_wp_index >= len(self.waypoint_lats):
            self.send_cmd(CMD_STOP)
            self.active = False
            return

        tgt_lat = self.waypoint_lats[self.current_wp_index]
        tgt_lon = self.waypoint_lons[self.current_wp_index]

        dist = self.haversine(self.current_lat, self.current_lon, tgt_lat, tgt_lon)
        bearing = self.bearing(self.current_lat, self.current_lon, tgt_lat, tgt_lon)

        if dist < self.wp_threshold:
            self.current_wp_index += 1
            return

        err = self.angle_diff(self.current_heading, bearing)
        abs_err = abs(err)
        gps_turn_is_sharp = abs_err >= self.sharp_turn_deg

        if abs_err <= self.heading_tol:
            cmd = CMD_FORWARD_STRAIGHT
        elif gps_turn_is_sharp:
            cmd = CMD_FORWARD_RIGHT if err > 0 else CMD_FORWARD_LEFT
        elif self.last_logical_cmd == CMD_FORWARD_LEFT:  # Fixed: Compare against logical command
            cmd = CMD_FORWARD_RIGHT if err > self.heading_tol + self.heading_hysteresis else CMD_FORWARD_LEFT
        elif self.last_logical_cmd == CMD_FORWARD_RIGHT: # Fixed: Compare against logical command
            cmd = CMD_FORWARD_LEFT if err < -(self.heading_tol + self.heading_hysteresis) else CMD_FORWARD_RIGHT
        else:
            cmd = CMD_FORWARD_RIGHT if err > 0 else CMD_FORWARD_LEFT

        self.last_logical_cmd = cmd  # Save the logical state here
        self.send_cmd(cmd)

        self.get_logger().info(
            f"WP{self.current_wp_index}: {dist:.1f}m | "
            f"bearing={bearing:.0f}° heading={self.current_heading:.0f}° "
            f"err={err:.0f}° → cmd={cmd}"
        )

    # --------------------------------------------------

    _INVERT_MAP = {
        CMD_FORWARD_STRAIGHT: CMD_BACKWARD_STRAIGHT,
        CMD_FORWARD_RIGHT: CMD_BACKWARD_RIGHT,
        CMD_FORWARD_LEFT: CMD_BACKWARD_LEFT,
    }

    def send_cmd(self, cmd: int):
        physical = self._INVERT_MAP.get(cmd, cmd) if self.invert_drive else cmd

        # Fixed: Removed the early return to ensure continuous publishing
        self.last_cmd = physical

        msg = Int32()
        msg.data = physical
        self.cmd_pub.publish(msg)

    # --------------------------------------------------

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return math.degrees(math.atan2(x, y)) % 360.0

    @staticmethod
    def angle_diff(a, b):
        return (b - a + 180.0) % 360.0 - 180.0


# --------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
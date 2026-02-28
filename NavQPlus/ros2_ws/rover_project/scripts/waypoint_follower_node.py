#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from rover_project.msg import GpsFix, ImuOrientation, Proximity, WaypointList

# Command Table
CMD_STOP = 2
CMD_FORWARD_STRAIGHT = 3
CMD_FORWARD_RIGHT = 4
CMD_FORWARD_LEFT = 5
CMD_BACKWARD_STRAIGHT = 6
CMD_BACKWARD_RIGHT = 7
CMD_BACKWARD_LEFT = 8


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # Navigation and quality gates
        self.declare_parameter('waypoint_reached_m', 2.5)
        self.declare_parameter('lookahead_distance_m', 6.0)
        self.declare_parameter('heading_tolerance_deg', 22.0)
        self.declare_parameter('sharp_turn_deg', 55.0)
        self.declare_parameter('heading_hysteresis_deg', 8.0)
        self.declare_parameter('heading_filter_alpha', 0.35)
        self.declare_parameter('command_rate_hz', 7.0)
        self.declare_parameter('gps_timeout_s', 5.0)
        self.declare_parameter('min_fix_type', 2)
        self.declare_parameter('max_h_acc_m', 10.0)
        self.declare_parameter('heading_offset_deg', 180.0)

        # Safety / perception assist
        self.declare_parameter('obstacle_stop_mm', 400)
        self.declare_parameter('vision_timeout_s', 0.6)
        self.declare_parameter('vision_error_deadband_px', 40.0)
        self.declare_parameter('vision_error_strong_px', 100.0)
        self.declare_parameter('vision_assist_max_heading_err_deg', 35.0)

        # Drive mapping
        self.declare_parameter('invert_drive', True)

        self.wp_threshold = float(self.get_parameter('waypoint_reached_m').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance_m').value)
        self.heading_tol = float(self.get_parameter('heading_tolerance_deg').value)
        self.sharp_turn_deg = float(self.get_parameter('sharp_turn_deg').value)
        self.heading_hysteresis = float(self.get_parameter('heading_hysteresis_deg').value)
        self.heading_filter_alpha = float(self.get_parameter('heading_filter_alpha').value)
        self.gps_timeout = float(self.get_parameter('gps_timeout_s').value)
        self.min_fix_type = int(self.get_parameter('min_fix_type').value)
        self.max_h_acc_m = float(self.get_parameter('max_h_acc_m').value)
        self.heading_offset = float(self.get_parameter('heading_offset_deg').value)
        self.obstacle_stop_mm = int(self.get_parameter('obstacle_stop_mm').value)
        self.vision_timeout_s = float(self.get_parameter('vision_timeout_s').value)
        self.vision_error_deadband_px = float(self.get_parameter('vision_error_deadband_px').value)
        self.vision_error_strong_px = float(self.get_parameter('vision_error_strong_px').value)
        self.vision_assist_max_heading_err = float(self.get_parameter('vision_assist_max_heading_err_deg').value)
        self.invert_drive = bool(self.get_parameter('invert_drive').value)
        cmd_rate = float(self.get_parameter('command_rate_hz').value)

        self.waypoint_lats = []
        self.waypoint_lons = []
        self.current_wp_index = 0
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.current_fix_type = 0
        self.current_h_acc = math.inf
        self.current_num_sats = 0
        self.current_hdop = math.inf
        self.last_gps_time = 0.0

        self.current_front_mm = None
        self.current_front_valid = False

        self.vision_error = 9999.0
        self.last_vision_time = 0.0

        self.filtered_heading_err = 0.0
        self.last_cmd = CMD_STOP
        self.safety_override = False
        self.active = False

        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)
        self.create_subscription(WaypointList, 'nav/waypoints', self.waypoint_callback, 10)
        self.create_subscription(GpsFix, 'can/gps', self.gps_callback, 10)
        self.create_subscription(ImuOrientation, 'can/imu_orientation', self.imu_callback, 10)
        self.create_subscription(Proximity, 'can/proximity_sensors', self.proximity_callback, 10)
        self.create_subscription(Float32, '/vision/error', self.vision_error_callback, 10)
        self.create_subscription(Bool, 'safety/override_active', self.safety_override_callback, 1)

        self.create_timer(1.0 / max(cmd_rate, 1.0), self.control_loop)

    def waypoint_callback(self, msg):
        self.waypoint_lats = list(msg.latitudes)
        self.waypoint_lons = list(msg.longitudes)
        self.current_wp_index = 1
        self.active = len(self.waypoint_lats) > 1
        self.filtered_heading_err = 0.0
        self.last_cmd = CMD_STOP

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_fix_type = int(msg.fix_type)
        self.current_h_acc = float(msg.h_acc)
        self.current_num_sats = int(msg.num_sats)
        self.current_hdop = float(msg.hdop)
        self.last_gps_time = time.time()

    def imu_callback(self, msg):
        self.current_heading = (float(msg.heading) + self.heading_offset) % 360.0

    def proximity_callback(self, msg):
        self.current_front_mm = int(msg.proximity_front)
        self.current_front_valid = bool(msg.front_valid)

    def vision_error_callback(self, msg):
        self.vision_error = float(msg.data)
        self.last_vision_time = time.time()

    def safety_override_callback(self, msg):
        self.safety_override = msg.data

    def control_loop(self):
        if self.safety_override:
            return

        if not self.active or self.current_lat is None or self.current_heading is None:
            self.send_cmd(CMD_STOP)
            return

        now = time.time()
        gps_stale = (now - self.last_gps_time) > self.gps_timeout
        gps_unreliable = (self.current_fix_type < self.min_fix_type) or (self.current_h_acc > self.max_h_acc_m)
        if gps_stale or gps_unreliable:
            self.send_cmd(CMD_STOP)
            return

        if self.current_front_valid and self.current_front_mm <= self.obstacle_stop_mm:
            self.send_cmd(CMD_STOP)
            return

        if self.current_wp_index >= len(self.waypoint_lats):
            self.send_cmd(CMD_STOP)
            self.active = False
            return

        tgt_lat = self.waypoint_lats[self.current_wp_index]
        tgt_lon = self.waypoint_lons[self.current_wp_index]

        dist = self.haversine(self.current_lat, self.current_lon, tgt_lat, tgt_lon)
        if dist < self.wp_threshold:
            self.current_wp_index += 1
            return

        bearing = self.lookahead_bearing(self.current_lat, self.current_lon)
        err = self.angle_diff(self.current_heading, bearing)
        self.filtered_heading_err = (
            (1.0 - self.heading_filter_alpha) * self.filtered_heading_err
            + self.heading_filter_alpha * err
        )
        err_for_cmd = self.filtered_heading_err
        abs_err = abs(err_for_cmd)
        gps_turn_is_sharp = abs_err >= self.sharp_turn_deg

        if abs_err <= self.heading_tol:
            cmd = CMD_FORWARD_STRAIGHT
            command_source = 'gps-straight'
        elif gps_turn_is_sharp:
            cmd = CMD_FORWARD_RIGHT if err_for_cmd > 0 else CMD_FORWARD_LEFT
            command_source = 'gps-sharp-turn'
        elif self.last_cmd == CMD_FORWARD_LEFT:
            cmd = CMD_FORWARD_RIGHT if err_for_cmd > self.heading_tol + self.heading_hysteresis else CMD_FORWARD_LEFT
            command_source = 'gps-hysteresis'
        elif self.last_cmd == CMD_FORWARD_RIGHT:
            cmd = CMD_FORWARD_LEFT if err_for_cmd < -(self.heading_tol + self.heading_hysteresis) else CMD_FORWARD_RIGHT
            command_source = 'gps-hysteresis'
        else:
            cmd = CMD_FORWARD_RIGHT if err_for_cmd > 0 else CMD_FORWARD_LEFT
            command_source = 'gps-heading'

        vision_recent = (now - self.last_vision_time) < self.vision_timeout_s
        vision_status = 'stale'
        if vision_recent and self.vision_error != 9999.0:
            v_err = self.vision_error
            vision_status = f'{v_err:.0f}px'
            vision_allowed = (not gps_turn_is_sharp) and (abs_err <= self.vision_assist_max_heading_err)
            if vision_allowed:
                if v_err > self.vision_error_strong_px:
                    cmd = CMD_FORWARD_RIGHT
                    command_source = 'vision-strong'
                elif v_err < -self.vision_error_strong_px:
                    cmd = CMD_FORWARD_LEFT
                    command_source = 'vision-strong'
                elif abs(v_err) > self.vision_error_deadband_px and cmd == CMD_FORWARD_STRAIGHT:
                    cmd = CMD_FORWARD_RIGHT if v_err > 0 else CMD_FORWARD_LEFT
                    command_source = 'vision-assist'
            else:
                vision_status = f'{v_err:.0f}px (gps-priority)'

        self.send_cmd(cmd)
        self.last_cmd = cmd

        self.get_logger().info(
            f'WP{self.current_wp_index}: {dist:.1f}m | '
            f'bearing={bearing:.0f}° heading={self.current_heading:.0f}° '
            f'err={err:.0f}° filt={err_for_cmd:.0f}° vision={vision_status} src={command_source} -> cmd={cmd} | '
            f'GPS: {self.current_num_sats}sats HDOP={self.current_hdop:.1f} h_acc={self.current_h_acc:.1f}m'
        )

    _INVERT_MAP = {
        CMD_FORWARD_STRAIGHT: CMD_BACKWARD_STRAIGHT,
        CMD_FORWARD_RIGHT: CMD_BACKWARD_RIGHT,
        CMD_FORWARD_LEFT: CMD_BACKWARD_LEFT,
    }

    def send_cmd(self, cmd: int):
        physical = self._INVERT_MAP.get(cmd, cmd) if self.invert_drive else cmd
        msg = Int32()
        msg.data = physical
        self.cmd_pub.publish(msg)

    def lookahead_bearing(self, lat: float, lon: float) -> float:
        primary_lat = self.waypoint_lats[self.current_wp_index]
        primary_lon = self.waypoint_lons[self.current_wp_index]
        primary_dist = self.haversine(lat, lon, primary_lat, primary_lon)
        primary_bearing = self.bearing(lat, lon, primary_lat, primary_lon)

        if self.current_wp_index + 1 >= len(self.waypoint_lats) or primary_dist > self.lookahead_distance:
            return primary_bearing

        next_lat = self.waypoint_lats[self.current_wp_index + 1]
        next_lon = self.waypoint_lons[self.current_wp_index + 1]
        next_bearing = self.bearing(lat, lon, next_lat, next_lon)

        weight = max(0.0, min(1.0, 1.0 - (primary_dist / max(self.lookahead_distance, 0.1))))
        return self.lerp_angle_deg(primary_bearing, next_bearing, weight)

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        r = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return math.degrees(math.atan2(x, y)) % 360.0

    @staticmethod
    def angle_diff(from_deg, to_deg):
        return (to_deg - from_deg + 540.0) % 360.0 - 180.0

    @staticmethod
    def lerp_angle_deg(a_deg: float, b_deg: float, t: float) -> float:
        delta = (b_deg - a_deg + 540.0) % 360.0 - 180.0
        return (a_deg + t * delta) % 360.0


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

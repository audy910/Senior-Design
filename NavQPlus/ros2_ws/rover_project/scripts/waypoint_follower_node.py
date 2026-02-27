#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
from rover_project.msg import GpsFix, ImuOrientation, WaypointList, Proximity
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

        self.declare_parameter('waypoint_reached_m', 2.5)
        self.declare_parameter('heading_tolerance_deg', 60.0)       # Heading Tolerance
        self.declare_parameter('sharp_turn_deg', 45.0)
        self.declare_parameter('vision_assist_max_heading_err_deg', 35.0)
        self.declare_parameter('command_rate_hz', 5.0)
        self.declare_parameter('invert_drive', True)

        self.wp_threshold = self.get_parameter('waypoint_reached_m').value
        self.heading_tol = self.get_parameter('heading_tolerance_deg').value
        self.sharp_turn_deg = self.get_parameter('sharp_turn_deg').value
        self.heading_hysteresis = self.get_parameter('heading_hysteresis_deg').value
        self.vision_assist_max_heading_err = self.get_parameter('vision_assist_max_heading_err_deg').value
        self.gps_timeout = self.get_parameter('gps_timeout_s').value
        self.min_fix_type = self.get_parameter('min_fix_type').value
        self.obstacle_stop_mm = self.get_parameter('obstacle_stop_mm').value
        self.heading_offset = self.get_parameter('heading_offset_deg').value
        self.max_h_acc_m = self.get_parameter('max_h_acc_m').value
        self.invert_drive = self.get_parameter('invert_drive').value
        cmd_rate = self.get_parameter('command_rate_hz').value

        self.waypoint_lats = []
        self.waypoint_lons = []
        self.current_wp_index = 0
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.safety_override = False
        self.active = False

        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)
        self.create_subscription(WaypointList, 'nav/waypoints', self.waypoint_callback, 10)
        self.create_subscription(GpsFix, 'can/gps', self.gps_callback, 10)
        self.create_subscription(ImuOrientation, 'can/imu_orientation', self.imu_callback, 10)
        self.create_subscription(Bool, 'safety/override_active', self.safety_override_callback, 1)

        self.create_timer(1.0 / cmd_rate, self.control_loop)

    def waypoint_callback(self, msg):
        self.waypoint_lats = list(msg.latitudes)
        self.waypoint_lons = list(msg.longitudes)
        self.current_wp_index = 1
        self.active = len(self.waypoint_lats) > 1

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def imu_callback(self, msg):
        self.current_heading = msg.heading

    def safety_override_callback(self, msg):
        self.safety_override = msg.data

    def control_loop(self):
        # IMPORTANT: If safety is active, stay SILENT.
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

        # Heading error with hysteresis to prevent left/right oscillation.
        # CMD_LEFT/CMD_RIGHT are not used — they turn wheels without moving the rover.
        err = self.angle_diff(self.current_heading, bearing)
        abs_err = abs(err)

        gps_turn_is_sharp = abs_err >= self.sharp_turn_deg

        if gps_turn_is_sharp:
            # Long-range waypoint tracking takes priority on large heading errors.
            cmd = CMD_FORWARD_RIGHT if err > 0 else CMD_FORWARD_LEFT
            command_source = 'gps-sharp-turn'
        elif abs_err <= self.heading_tol:
            cmd = CMD_FORWARD_STRAIGHT
            command_source = 'gps-straight'
        elif self.last_cmd == CMD_FORWARD_LEFT:
            # Already curving left — only switch right if error is well past centre
            cmd = CMD_FORWARD_RIGHT if err > self.heading_tol + self.heading_hysteresis else CMD_FORWARD_LEFT
            command_source = 'gps-hysteresis'
        elif self.last_cmd == CMD_FORWARD_RIGHT:
            # Already curving right — only switch left if error is well past centre
            cmd = CMD_FORWARD_LEFT if err < -(self.heading_tol + self.heading_hysteresis) else CMD_FORWARD_RIGHT
            command_source = 'gps-hysteresis'
        else:
            cmd = CMD_FORWARD_RIGHT if err > 0 else CMD_FORWARD_LEFT
            command_source = 'gps-heading'

        vision_recent = (time.time() - self.last_vision_time) < self.vision_timeout_s
        vision_status = 'stale'
        if vision_recent and self.vision_error != 9999.0:
            v_err = self.vision_error
            vision_status = f"{v_err:.0f}px"
            # Positive vision error means path center is right of image center.
            # Vision is lane-centering assist only; GPS remains primary for sharp turns.
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
                vision_status = f"{v_err:.0f}px (gps-priority)"

        self.send_cmd(cmd)

        self.get_logger().info(
            f"  WP{self.current_wp_index}: {dist:.1f}m | "
            f"bearing={bearing:.0f}° heading={self.current_heading:.0f}° "
            f"err={err:.0f}° vision={vision_status} src={command_source} → cmd={cmd} | "
            f"GPS: {self.current_num_sats}sats HDOP={self.current_hdop:.1f} h_acc={self.current_h_acc:.1f}m")

    #  HELPERS

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

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp, dl = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2-lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
        return math.degrees(math.atan2(x, y)) % 360.0

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

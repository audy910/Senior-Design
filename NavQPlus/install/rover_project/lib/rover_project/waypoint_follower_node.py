#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
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
        
        # GPS Navigation Parameters
        self.declare_parameter('heading_tolerance_deg', 60.0)
        self.declare_parameter('sharp_turn_deg', 45.0)
        self.declare_parameter('heading_hysteresis_deg', 10.0)
        
        # Vision / Sensor Fusion Parameters 
        self.declare_parameter('gps_takeover_deg', 25.0) 
        self.declare_parameter('vision_error_strong_px', 50.0)
        self.declare_parameter('vision_error_deadband_px', 10.0)
        
        self.declare_parameter('command_rate_hz', 5.0)
        self.declare_parameter('invert_drive', False)

        # --- NEW: Progress Tracking Parameters ---
        # How often to check if we are making progress (seconds)
        self.declare_parameter('progress_check_interval_sec', 3.0)
        # How much distance increase is allowed before alarming (meters)
        self.declare_parameter('progress_tolerance_m', 0.5)

        # --- Get parameters ---
        self.wp_threshold = self.get_parameter('waypoint_reached_m').value
        self.heading_tol = self.get_parameter('heading_tolerance_deg').value
        self.sharp_turn_deg = self.get_parameter('sharp_turn_deg').value
        self.heading_hysteresis = self.get_parameter('heading_hysteresis_deg').value
        self.gps_takeover_deg = self.get_parameter('gps_takeover_deg').value
        self.vision_error_strong_px = self.get_parameter('vision_error_strong_px').value
        self.vision_error_deadband_px = self.get_parameter('vision_error_deadband_px').value
        self.invert_drive = self.get_parameter('invert_drive').value
        
        self.progress_interval = self.get_parameter('progress_check_interval_sec').value
        self.progress_tolerance = self.get_parameter('progress_tolerance_m').value
        
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
        self.last_logical_cmd = CMD_STOP  
        self._was_overridden = False      

        # Vision placeholders
        self.last_vision_time = 0.0
        self.vision_error_px = 0.0

        # --- NEW: Progress Tracking State ---
        self.last_dist_check_time = 0.0
        self.previous_distance = None
        self.tracked_wp_index = -1

        # --- ROS Interfaces ---
        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)

        self.create_subscription(WaypointList, 'nav/waypoints', self.waypoint_callback, 10)
        self.create_subscription(GpsFix, 'can/gps', self.gps_callback, 10)
        self.create_subscription(ImuOrientation, 'can/imu_orientation', self.imu_callback, 10)
        self.create_subscription(Bool, 'safety/override_active', self.safety_override_callback, 1)
        self.create_subscription(Float32, 'nav/vision_error', self.vision_callback, 10)

        self.create_timer(1.0 / cmd_rate, self.control_loop)
        self.get_logger().info("Waypoint Follower Node Started (Vision Integration Active)")

    # --------------------------------------------------

    def waypoint_callback(self, msg):
        self.waypoint_lats = list(msg.latitudes)
        self.waypoint_lons = list(msg.longitudes)
        self.current_wp_index = 0  
        self.active = len(self.waypoint_lats) > 0  

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def imu_callback(self, msg):
        self.current_heading = msg.heading

    def safety_override_callback(self, msg):
        self.safety_override = msg.data

    def vision_callback(self, msg):
        self.vision_error_px = msg.data
        self.last_vision_time = self.get_clock().now().nanoseconds / 1e9

    # --------------------------------------------------

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # 1. Safety Check
        if self.safety_override:
            if not self._was_overridden:
                self.get_logger().info("[WAYPOINT] 🛑 YIELDING CONTROL: Safety override is ACTIVE.")
                self._was_overridden = True
            return
        elif self._was_overridden:
            self.get_logger().info("[WAYPOINT] 🟢 TAKING CONTROL: Safety override is OFF.")
            self._was_overridden = False

        if not self.active or self.current_lat is None or self.current_heading is None:
            self.send_cmd(CMD_STOP)
            return

        if self.current_wp_index >= len(self.waypoint_lats):
            self.send_cmd(CMD_STOP)
            self.active = False
            self.get_logger().info("🏁 All waypoints reached.")
            return

        # 2. Macro Navigation Math (GPS)
        tgt_lat = self.waypoint_lats[self.current_wp_index]
        tgt_lon = self.waypoint_lons[self.current_wp_index]

        dist = self.haversine(self.current_lat, self.current_lon, tgt_lat, tgt_lon)
        bearing = self.bearing(self.current_lat, self.current_lon, tgt_lat, tgt_lon)

        if dist < self.wp_threshold:
            self.get_logger().info(f"✅ Reached Waypoint {self.current_wp_index}!")
            self.current_wp_index += 1
            return

        # --- NEW: Progress Check Logic ---
        # Reset tracker if we just switched to a new waypoint
        if self.current_wp_index != self.tracked_wp_index:
            self.tracked_wp_index = self.current_wp_index
            self.previous_distance = dist
            self.last_dist_check_time = now

        # Every interval (e.g., 3 seconds), check our distance progress
        if (now - self.last_dist_check_time) >= self.progress_interval:
            # If our current distance is greater than the previous distance + GPS noise tolerance
            if dist > (self.previous_distance + self.progress_tolerance):
                self.get_logger().warn(
                    f"⚠️ OFF COURSE ALARM! Distance increased from {self.previous_distance:.1f}m to {dist:.1f}m."
                )
                # Uncomment the line below if you want the rover to automatically stop when this happens:
                # self.send_cmd(CMD_STOP); return

            # Update the baseline for the next check
            self.previous_distance = dist
            self.last_dist_check_time = now
        # ----------------------------------

        gps_err = self.angle_diff(self.current_heading, bearing)
        
        # 3. Vision Status Check
        vision_is_fresh = (now - self.last_vision_time) < 1.0  # 1-second timeout

        # 4. SENSOR FUSION DECISION TREE
        if abs(gps_err) > self.gps_takeover_deg or not vision_is_fresh:
            
            nav_source = "GPS 🛰️"
            abs_err = abs(gps_err)
            gps_turn_is_sharp = abs_err >= self.sharp_turn_deg

            if abs_err <= self.heading_tol:
                cmd = CMD_FORWARD_STRAIGHT
            elif gps_turn_is_sharp:
                cmd = CMD_FORWARD_RIGHT if gps_err > 0 else CMD_FORWARD_LEFT
            elif self.last_logical_cmd == CMD_FORWARD_LEFT:  
                cmd = CMD_FORWARD_RIGHT if gps_err > self.heading_tol + self.heading_hysteresis else CMD_FORWARD_LEFT
            elif self.last_logical_cmd == CMD_FORWARD_RIGHT: 
                cmd = CMD_FORWARD_LEFT if gps_err < -(self.heading_tol + self.heading_hysteresis) else CMD_FORWARD_RIGHT
            else:
                cmd = CMD_FORWARD_RIGHT if gps_err > 0 else CMD_FORWARD_LEFT
                
            debug_err = f"{gps_err:.0f}°"

        else:
            nav_source = "VISION 👁️"
            v_err = self.vision_error_px
            cmd = CMD_FORWARD_STRAIGHT  
            
            if v_err > self.vision_error_strong_px:
                cmd = CMD_FORWARD_RIGHT
            elif v_err < -self.vision_error_strong_px:
                cmd = CMD_FORWARD_LEFT
            elif abs(v_err) > self.vision_error_deadband_px:
                cmd = CMD_FORWARD_RIGHT if v_err > 0 else CMD_FORWARD_LEFT
                debug_err = f"{v_err:.0f}px"

        # 5. Execute Command
        self.last_logical_cmd = cmd 
        self.send_cmd(cmd)

        self.get_logger().info(
            f"[{nav_source}] WP{self.current_wp_index}: {dist:.1f}m | "
            f"err={debug_err} → cmd={cmd}"
        )

    # --------------------------------------------------
    _INVERT_MAP = {
        CMD_FORWARD_STRAIGHT: CMD_BACKWARD_STRAIGHT,
        CMD_FORWARD_RIGHT:    CMD_BACKWARD_LEFT,
        CMD_FORWARD_LEFT:     CMD_BACKWARD_RIGHT,
        CMD_BACKWARD_STRAIGHT: CMD_FORWARD_STRAIGHT,
        CMD_BACKWARD_RIGHT:   CMD_FORWARD_LEFT,
        CMD_BACKWARD_LEFT:    CMD_FORWARD_RIGHT,
    }

    def send_cmd(self, cmd: int):
        physical = self._INVERT_MAP.get(cmd, cmd) if self.invert_drive else cmd
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
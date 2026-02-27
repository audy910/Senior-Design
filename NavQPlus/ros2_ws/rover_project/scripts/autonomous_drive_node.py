#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from rover_project.msg import Proximity
from std_msgs.msg import Float32

# Command Table — matches uart_node and waypoint_follower_node
CMD_STOP              = 2
CMD_FORWARD_STRAIGHT  = 3
CMD_FORWARD_RIGHT     = 4
CMD_FORWARD_LEFT      = 5
CMD_BACKWARD_STRAIGHT = 6
CMD_BACKWARD_RIGHT    = 7
CMD_BACKWARD_LEFT     = 8
CMD_RIGHT             = 9   # Pivot right (sensor scan)
CMD_LEFT              = 10  # Pivot left (sensor scan)

# States
STATE_FORWARD    = "FORWARD"
STATE_CLIFF_STOP = "CLIFF_STOP"  # Holding stop while cliff is active; transitions to recovery on clear
STATE_REV_CHECK  = "REV_CHECK"
STATE_REVERSING  = "REVERSING"
STATE_LOOK_L     = "LOOK_L"
STATE_SCAN_L     = "SCAN_L"
STATE_LOOK_R     = "LOOK_R"
STATE_SCAN_R     = "SCAN_R"
STATE_DRIVE_OUT  = "DRIVE_OUT"


class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')

        self.sub = self.create_subscription(
            Proximity,
            'can/proximity_sensors',
            self.proximity_callback,
            10)

        # Publishes drive commands on the shared nav topic (same as waypoint_follower)
        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)

        # Signals waypoint_follower to yield when a safety correction is active
        self.override_pub = self.create_publisher(Bool, 'safety/override_active', 1)

        self.declare_parameter('invert_drive', False)
        self.declare_parameter('wall_threshold_mm', 500.0)
        self.declare_parameter('reverse_time_s', 1.0)
        self.declare_parameter('sensor_time_s', 0.5)
        self.declare_parameter('maneuver_time_s', 2.0)
        self.declare_parameter('required_readings', 4)
        self.declare_parameter('cliff_hold_s', 1.0)

        self.invert_drive    = self.get_parameter('invert_drive').value
        self.wall_threshold  = self.get_parameter('wall_threshold_mm').value
        self.reverse_time    = self.get_parameter('reverse_time_s').value
        self.sensor_time     = self.get_parameter('sensor_time_s').value
        self.maneuver_time   = self.get_parameter('maneuver_time_s').value
        self.required_readings = self.get_parameter('required_readings').value
        self.cliff_hold_seconds = self.get_parameter('cliff_hold_s').value

        # Wall validation
        self.valid_reading_count = 0

        # Cliff latch
        self.cliff_active    = False
        self.last_cliff_time = 0.0

        # State machine
        self.current_state    = STATE_FORWARD
        self.state_start_time = 0.0
        self.chosen_turn_cmd  = CMD_FORWARD_LEFT

    # ── Command helpers ──────────────────────────────────────────────────────

    _INVERT_MAP = {
        CMD_FORWARD_STRAIGHT:  CMD_BACKWARD_STRAIGHT,
        CMD_FORWARD_RIGHT:     CMD_BACKWARD_RIGHT,
        CMD_FORWARD_LEFT:      CMD_BACKWARD_LEFT,
        CMD_BACKWARD_STRAIGHT: CMD_FORWARD_STRAIGHT,
        CMD_BACKWARD_RIGHT:    CMD_FORWARD_RIGHT,
        CMD_BACKWARD_LEFT:     CMD_FORWARD_LEFT,
    }

    def send_cmd(self, cmd: int):
        physical = self._INVERT_MAP.get(cmd, cmd) if self.invert_drive else cmd
        msg = Int32()
        msg.data = physical
        self.cmd_pub.publish(msg)

    def publish_override(self, active: bool):
        msg = Bool()
        msg.data = active
        self.override_pub.publish(msg)

    def set_state(self, new_state):
        self.get_logger().info(f"Transitioning to: {new_state}")
        self.valid_reading_count = 0
        self.current_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

    # ── Main callback ────────────────────────────────────────────────────────

    
    def vision_callback(self, msg):
        self.vision_error = msg.data
        self.vision_last_time = self.get_clock().now().nanoseconds / 1e9
    
    def proximity_callback(self, msg):
        now     = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time

        # ── Cliff latch ──────────────────────────────────────────────────────
        if msg.cliff_detected:
            if not self.cliff_active:
                self.get_logger().warn("Cliff detected — STOP latched")
            self.cliff_active    = True
            self.last_cliff_time = now
        elif self.cliff_active and (now - self.last_cliff_time) > self.cliff_hold_seconds:
            self.get_logger().info("Cliff latch cleared")
            self.cliff_active = False

        # ── Hard safety stop: cliff ───────────────────────────────────────────
        # Latches into CLIFF_STOP so the recovery maneuver runs on clear.
        if self.cliff_active:
            self.publish_override(True)
            self.send_cmd(CMD_STOP)
            if self.current_state != STATE_CLIFF_STOP:
                self.set_state(STATE_CLIFF_STOP)
            return

        # ── Front sensor invalid ─────────────────────────────────────────────
        # HC-SR04 returns invalid when nothing is in range (open space).
        # In STATE_FORWARD: release override and return — waypoint_follower drives.
        # In correction states: fall through so time-based transitions (REVERSING,
        # DRIVE_OUT, etc.) still complete even when the front sensor reads nothing.
        if not msg.front_valid:
            if self.current_state == STATE_FORWARD:
                self.publish_override(False)
                return
            # Correction state: do NOT return — let the state machine run.
            # SCAN_L/R already treat front_valid=False as "clear" (open space).

        # ── Wall validation (STATE_FORWARD only) ─────────────────────────────
        # Count consecutive valid readings below threshold (regardless of direction).
        # This correctly handles the case where waypoint_follower has already stopped
        # the rover — the distance stabilises rather than continuing to decrease.
        if self.current_state == STATE_FORWARD and msg.front_valid:
            if msg.proximity_front < self.wall_threshold:
                self.valid_reading_count += 1
            else:
                self.valid_reading_count = 0  # clear reading — reset hysteresis

        # ── STATE_FORWARD: yield to waypoint_follower when safe ──────────────
        if self.current_state == STATE_FORWARD:
            if msg.proximity_front < self.wall_threshold and self.valid_reading_count >= self.required_readings:
                self.get_logger().warn(f"Valid wall at {msg.proximity_front:.0f}mm — taking override")
                self.set_state(STATE_REV_CHECK)
                # Falls through to correction block below
            else:
                # All clear — release control to waypoint_follower
                self.publish_override(False)
                return

        # ── Cliff cleared: kick off the same recovery maneuver as wall avoidance ─
        # Cliff is NOT active here (returned early above if it were).
        # Back away from the edge and scan left/right before handing back control.
        if self.current_state == STATE_CLIFF_STOP:
            self.get_logger().warn("Cliff cleared — backing away from edge before resuming")
            self.set_state(STATE_REV_CHECK)
            # Falls through to STATE_REV_CHECK below

        # ── Correction states: hold override and drive ───────────────────────
        self.publish_override(True)

        if self.current_state == STATE_REV_CHECK:
            # Only block reversing if the rear sensor is valid AND close.
            # An invalid reading (rear_valid=False, proximity_rear=0) must not
            # trap the rover at a cliff edge — treat it as "clear to reverse".
            rear_blocked = msg.rear_valid and (msg.proximity_rear <= self.wall_threshold)
            if not rear_blocked:
                self.set_state(STATE_REVERSING)
            else:
                self.send_cmd(CMD_STOP)

        elif self.current_state == STATE_REVERSING:
            if elapsed < self.reverse_time:
                self.send_cmd(CMD_BACKWARD_STRAIGHT)
            else:
                self.set_state(STATE_LOOK_L)

        elif self.current_state == STATE_LOOK_L:
            if elapsed < self.sensor_time:
                self.send_cmd(CMD_LEFT)
            else:
                self.set_state(STATE_SCAN_L)

        elif self.current_state == STATE_SCAN_L:
            self.send_cmd(CMD_STOP)
            if elapsed > 0.5:
                # front_valid=False means nothing in range (open space) — treat as clear
                if not msg.front_valid or msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_LEFT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_LOOK_R)

        elif self.current_state == STATE_LOOK_R:
            if elapsed < self.sensor_time:
                self.send_cmd(CMD_RIGHT)
            else:
                self.set_state(STATE_SCAN_R)

        elif self.current_state == STATE_SCAN_R:
            self.send_cmd(CMD_STOP)
            if elapsed > 0.5:
                # front_valid=False means nothing in range (open space) — treat as clear
                if not msg.front_valid or msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_RIGHT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_REV_CHECK)

        elif self.current_state == STATE_DRIVE_OUT:
            if elapsed < self.maneuver_time:
                self.send_cmd(self.chosen_turn_cmd)
            else:
                self.send_cmd(CMD_STOP)
                self.set_state(STATE_FORWARD)
                # Next callback: STATE_FORWARD + safe → override released


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

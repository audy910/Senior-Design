#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String  # Added String import
from rover_project.msg import Proximity

# Command Table
CMD_STOP             = 2
CMD_REVERSE          = 3
CMD_FORWARD_STRAIGHT = 6
CMD_FORWARD_LEFT     = 7
CMD_FORWARD_RIGHT    = 8
CMD_LEFT             = 9   # Turn sensor
CMD_RIGHT            = 10  # Turn sensor

# States
STATE_FORWARD   = "FORWARD"
STATE_REV_CHECK = "REV_CHECK"
STATE_REVERSING = "REVERSING"
STATE_LOOK_L    = "LOOK_L"
STATE_SCAN_L    = "SCAN_L"
STATE_LOOK_R    = "LOOK_R"
STATE_SCAN_R    = "SCAN_R"
STATE_DRIVE_OUT = "DRIVE_OUT"


class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')
        
        # Publishers and Subscriptions
        self.voice_pub = self.create_publisher(String, 'robot_voice_trigger', 10)
        self.cmd_pub = self.create_publisher(Int32, 'bluetooth_commands', 10)
        self.sub = self.create_subscription(
            Proximity,
            'can/proximity_sensors',
            self.proximity_callback,
            10)

        # Configuration
        self.wall_threshold = 500.0  
        self.reverse_time   = 1.0   
        self.sensor_time    = 0.5   
        self.maneuver_time  = 2.0   
        
        # Validation Logic Variables
        self.prev_proximity_front = 9999.0
        self.valid_reading_count = 0
        self.required_readings = 4 

        # State Machine & Safety Variables
        self.current_state = STATE_FORWARD
        self.state_start_time = 0.0
        self.chosen_turn_cmd = CMD_FORWARD_LEFT
        self.cliff_active = False # Added initialization

    def trigger_voice(self, text):
        msg = String()
        msg.data = text
        self.voice_pub.publish(msg)

    def set_state(self, new_state):
        self.get_logger().info(f"Transitioning to: {new_state}")
        self.valid_reading_count = 0
        self.current_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

    def proximity_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time
        drive_msg = Int32()
        
        # --- Voice Triggers ---
        # Trigger on first detection of cliff
        if msg.cliff_detected and not self.cliff_active:
            self.trigger_voice("Stop")
            self.cliff_active = True
        elif not msg.cliff_detected:
            self.cliff_active = False

        # Trigger on valid wall detection
        if self.current_state == STATE_FORWARD:
            if msg.proximity_front < self.wall_threshold and self.valid_reading_count == self.required_readings:
                self.trigger_voice("Stop")

        # --- Hard Safety Stop ---
        if msg.cliff_detected or not msg.front_valid:
            drive_msg.data = CMD_STOP
            self.cmd_pub.publish(drive_msg)
            return

        # --- Data Validation Logic (Forward Only) ---
        is_getting_closer = msg.proximity_front < self.prev_proximity_front
        
        if self.current_state == STATE_FORWARD and msg.proximity_front < self.wall_threshold:
            if is_getting_closer:
                self.valid_reading_count += 1
            else:
                self.valid_reading_count = max(0, self.valid_reading_count - 1)
        
        self.prev_proximity_front = msg.proximity_front

        # --- State Machine ---
        if self.current_state == STATE_FORWARD:
            if msg.proximity_front < self.wall_threshold and self.valid_reading_count >= self.required_readings:
                self.get_logger().warn(f"Valid wall detected at {msg.proximity_front}mm")
                self.set_state(STATE_REV_CHECK)
            else:
                drive_msg.data = CMD_FORWARD_STRAIGHT

        elif self.current_state == STATE_REV_CHECK:
            if msg.proximity_rear > self.wall_threshold:
                self.set_state(STATE_REVERSING)
            else:
                drive_msg.data = CMD_STOP

        elif self.current_state == STATE_REVERSING:
            if elapsed < self.reverse_time:
                drive_msg.data = CMD_REVERSE
            else:
                self.set_state(STATE_LOOK_L)

        elif self.current_state == STATE_LOOK_L:
            if elapsed < self.sensor_time:
                drive_msg.data = CMD_LEFT
            else:
                self.set_state(STATE_SCAN_L)

        elif self.current_state == STATE_SCAN_L:
            drive_msg.data = CMD_STOP
            if elapsed > 0.5:
                if msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_LEFT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_LOOK_R)

        elif self.current_state == STATE_LOOK_R:
            if elapsed < self.sensor_time:
                drive_msg.data = CMD_RIGHT
            else:
                self.set_state(STATE_SCAN_R)

        elif self.current_state == STATE_SCAN_R:
            drive_msg.data = CMD_STOP
            if elapsed > 0.5:
                if msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_RIGHT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_REV_CHECK)

        elif self.current_state == STATE_DRIVE_OUT:
            if elapsed < self.maneuver_time:
                drive_msg.data = self.chosen_turn_cmd
            else:
                self.set_state(STATE_FORWARD)

        self.cmd_pub.publish(drive_msg)

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
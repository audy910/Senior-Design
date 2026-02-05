#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rover_project.msg import Proximity
import time

# Command Table
CMD_STOP             = 2
CMD_FORWARD_STRAIGHT = 3
CMD_FORWARD_LEFT     = 5

# States
STATE_FORWARD = 0
STATE_TURNING = 1

class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')

        self.sub = self.create_subscription(
            Proximity,
            'can/proximity_sensors',
            self.proximity_callback,
            10)

        self.cmd_pub = self.create_publisher(Int32, 'bluetooth_commands', 10)

        # Configuration
        self.wall_threshold = 700.0  # mm (Start reacting at X.XXm)
        self.clearance_time = 1.5     # seconds (Keep turning after wall is gone)
        
        # Internal Logic State
        self.current_state = STATE_FORWARD
        self.last_wall_seen_time = 0.0

        self.get_logger().info("Robust Autonomous Drive Node Online.")

    def proximity_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        drive_msg = Int32()

        # 1. Safety Checks (Cliff or invalid data)
        if msg.cliff_detected or not msg.front_valid:
            drive_msg.data = CMD_STOP
            self.cmd_pub.publish(drive_msg)
            return

        # 2. State Machine Logic
        if self.current_state == STATE_FORWARD:
            if msg.proximity_rear < self.wall_threshold:
                # Transition to Turning
                self.current_state = STATE_TURNING
                self.last_wall_seen_time = now
                drive_msg.data = CMD_FORWARD_LEFT
                self.get_logger().warn("Wall detected! Switching to TURNING state.")
            else:
                drive_msg.data = CMD_FORWARD_STRAIGHT

        elif self.current_state == STATE_TURNING:
            if msg.proximity_rear < self.wall_threshold:
                # Wall is still there, reset the timer
                self.last_wall_seen_time = now
                drive_msg.data = CMD_FORWARD_LEFT
            else:
                # Wall is gone, but are we clear yet?
                if (now - self.last_wall_seen_time) < self.clearance_time:
                    drive_msg.data = CMD_FORWARD_LEFT
                    self.get_logger().info("Wall gone, continuing turn for clearance...", once=True)
                else:
                    # Timer expired, safe to go straight
                    self.current_state = STATE_FORWARD
                    drive_msg.data = CMD_FORWARD_STRAIGHT
                    self.get_logger().info("Clearance reached. Resuming FORWARD.")

        self.cmd_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
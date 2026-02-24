#!/usr/bin/env python3
"""
uart_node.py  (UPDATED)

Changes:
  - Subscribes to 'nav/drive_cmd' for waypoint follower commands (AI mode)
  - Subscribes to 'bluetooth_commands' for manual control (unchanged)
  - Failsafe now runs on a ROS timer instead of never being called
  - CMD_AI from bluetooth toggles into AI mode
  - Any manual command switches back to MANUAL mode
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import serial
import time
import sys


# Command Table
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

MANUAL = 0
AI     = 1


class UartNode(Node):
    def __init__(self):
        super().__init__('uart_node')

        # Subscriptions
        self.manual_sub = self.create_subscription(
            Int32, 'bluetooth_commands', self.manual_command, 10)

        self.ai_sub = self.create_subscription(
            Int32, 'nav/drive_cmd', self.ai_command, 10)

        # UART Setup
        port = '/dev/ttymxc2'
        baudrate = 115200
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            time.sleep(0.01)
            self.ser.flush()
            self.get_logger().info(f"✓ UART connected on {port} @ {baudrate}")
        except Exception as e:
            self.get_logger().error(f"✗ UART ERROR: {e}")
            sys.exit(1)

        # State
        self.state = AI
        self.last_cmd_time = time.time()
        self.timeout_seconds = 3

        # Failsafe timer
        self.create_timer(0.5, self.failsafe_check)

    def failsafe_check(self):
        if time.time() - self.last_cmd_time > self.timeout_seconds:
            self.get_logger().warn("Failsafe STOP")
            self._send_uart(CMD_STOP)
            self.last_cmd_time = time.time()

    def manual_command(self, msg: Int32):
        cmd = msg.data
        self.last_cmd_time = time.time()

        # CMD_AI toggles into AI mode
        if cmd == CMD_AI:
            if self.state != AI:
                self.state = AI
                self.get_logger().info("*** MODE → AI ***")
                self._send_uart(CMD_AI)
            return

        # Any other manual command → switch to MANUAL
        if self.state != MANUAL:
            self.state = MANUAL
            self.get_logger().info("*** MODE → MANUAL ***")

        self._send_uart(cmd)

    def ai_command(self, msg: Int32):
        """Forward waypoint follower commands only when in AI mode."""
        if self.state != AI:
            return

        self.last_cmd_time = time.time()
        self._send_uart(msg.data)

    def _send_uart(self, cmd: int):
        try:
            self.ser.write(bytes([cmd]))
            self.get_logger().info(
                f"UART TX: {cmd} ({'AI' if self.state == AI else 'MANUAL'})")
        except Exception as e:
            self.get_logger().error(f"UART send error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UartNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
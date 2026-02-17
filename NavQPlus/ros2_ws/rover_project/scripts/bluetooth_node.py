#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import time
import serial
import threading


class BluetoothNode(Node):
    def __init__(self):
        super().__init__("bluetooth_node")

        self.bt_serial = None
        self.bt_thread = None
        self.running = True
        self.serial_lock = threading.Lock()
        self.setup_bluetooth()

        self.command_pub = self.create_publisher(Int32, 'bluetooth_commands', 1)

    def setup_bluetooth(self):
        try:
            self.bt_serial = serial.Serial("/dev/rfcomm0", 9600, timeout=0.1)
            time.sleep(2)
            self.get_logger().info('Bluetooth connected successfully')

            # Start Bluetooth listener thread
            self.bt_thread = threading.Thread(target=self.bluetooth_listener, daemon=True)
            self.bt_thread.start()

        except serial.SerialException as e:
            self.get_logger().error(f'Bluetooth connection failed: {e}')
            self.get_logger().warn('Continuing in ML-only mode')

    def bluetooth_listener(self):
        while self.running:
            try:
                with self.serial_lock:
                    if self.bt_serial and self.bt_serial.in_waiting > 0:
                        data = self.bt_serial.readline().decode().strip()
                        if data:
                            self.handle_bluetooth_command(data)
            except Exception as e:
                self.get_logger().error(f'Bluetooth read error: {e}')
            time.sleep(0.001)

    def handle_bluetooth_command(self, command):
        try:
            cmd_value = int(command)
            # Validate command is in valid range (0-10)
            if 0 <= cmd_value <= 10:
                self.get_logger().info(f'BT received: {cmd_value}')
                msg = Int32()
                msg.data = cmd_value
                self.command_pub.publish(msg)
            else:
                self.get_logger().warn(f'Invalid BT command value: {cmd_value} (must be 0-10)')
        except ValueError:
            self.get_logger().warn(f'Invalid BT command format: {command}')

    def shutdown(self):
        """Clean shutdown of bluetooth thread and serial connection"""
        self.running = False
        if self.bt_thread and self.bt_thread.is_alive():
            self.bt_thread.join(timeout=1.0)
        with self.serial_lock:
            if self.bt_serial:
                self.bt_serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
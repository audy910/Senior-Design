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
                if self.bt_serial and self.bt_serial.in_waiting > 0:
                    data = self.bt_serial.readline().decode().strip()
                    self.handle_bluetooth_command(data)
            except Exception as e:
                self.get_logger().error(f'Bluetooth read error: {e}')
            time.sleep(0.001)

    def handle_bluetooth_command(self, command):
        self.get_logger().info(f'BT received: {command}')
        msg = Int32()
        msg.data = int(command)
        self.command_pub.publish(msg)

def main(args=None):
        rclpy.init(args=args)
        node = BluetoothNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
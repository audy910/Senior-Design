import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray

import serial
import time
import sys


# Command Table
CMD_OFF                 = 0
CMD_AI                  = 1     # toggle AI mode
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
            Int32,
            'bluetooth_commands',
            self.manual_command,
            10
        )

        self.bbox_sub = self.create_subscription(
            Int32MultiArray,
            'ai_bboxes',
            self.send_bbox,
            10
        )

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

            print(f"✓ UART connected on {port} @ {baudrate}")
        except Exception as e:
            print(f"✗ UART ERROR: {e}")
            sys.exit(1)

        # State
        self.state = MANUAL          # start in manual mode
        self.last_cmd_time = time.time()
        self.timeout_seconds = 3

    def failsafe(self):
        if time.time() - self.last_cmd_time > self.timeout_seconds:
            print("Failsafe STOP")
            msg = Int32()
            msg.data = CMD_STOP
            self.manual_command(msg)

    def manual_command(self, msg: Int32):
        cmd = msg.data
        self.last_cmd_time = time.time()

        if cmd == CMD_AI:  # 1
            if self.state != AI:
                self.state = AI
                print("*** MODE SET → AI ***")

                # SEND 1 TO K64F TO ENTER AI MODE
                try:
                    self.ser.write(bytes([CMD_AI]))
                    print("Sent CMD_AI (1) to K64F")
                except Exception as e:
                    print("UART send error:", e)

            return

        if self.state != MANUAL:
            self.state = MANUAL
            print("*** MODE SET → MANUAL ***")

        try:
            self.ser.write(bytes([cmd]))
            print(f"Sent MANUAL CMD: {cmd}")
        except Exception as e:
            print(f"UART send error: {e}")

    def send_bbox(self, msg: Int32MultiArray):
        if self.state != AI:
            return  # ignore bboxes in manual mode

        if len(msg.data) != 4:
            return

        x, y, w, h = msg.data

        uart_msg = f"<BBOX,{x},{y},{w},{h}>"

        try:
            self.ser.write(uart_msg.encode("ascii"))
            print("Sent BBOX:", uart_msg)
        except Exception as e:
            print("UART BBOX error:", e)


def main(args=None):
    rclpy.init(args=args)
    node = UartNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

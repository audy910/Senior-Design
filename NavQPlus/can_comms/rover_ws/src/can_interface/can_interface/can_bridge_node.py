import rclpy
from rclpy.node import Node
from std_msgs.msg import String # We'll use String for now, but custom msgs are better!
import can
import cantools
import os
from ament_index_python.packages import get_package_share_directory

class CanBridgeNode(Node):
    def __init__(self):
        super().__init__('can_bridge_node')
        
        package_share_directory = get_package_share_directory('can_interface')
        dbc_path = os.path.join(package_share_directory, 'rover_optimized.dbc')
        
        try:
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f"DBC loaded from: {dbc_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC: {e}")
            raise e

        # 2. Initialize CAN Bus
        try:
            self.bus = can.interface.Bus(channel='can1', bustype='socketcan')
            self.get_logger().info("Connected to can1")
        except Exception as e:
            self.get_logger().error(f"CAN Init Error: {e}")
            raise e

        # 3. Initialize Publishers
        # To expand this, you'd eventually create specific topics for GPS, IMU, etc.
        self.gps_pub = self.create_publisher(String, 'rover/gps', 10)
        self.imu_pub = self.create_publisher(String, 'rover/imu', 10)
        self.prox_pub = self.create_publisher(String, 'rover/proximity', 10)

        # 4. Create a Timer for the high-frequency CAN loop (e.g., 10ms / 100Hz)
        self.timer = self.create_timer(0.01, self.can_receive_callback)

    def can_receive_callback(self):
        # We read non-blocking to keep the ROS node responsive
        message = self.bus.recv(timeout=0.0)
        
        if message is not None:
            try:
                # Decode using your logic
                decoded = self.db.decode_message(message.arbitration_id, message.data)
                msg_def = self.db.get_message_by_frame_id(message.arbitration_id)
                msg_name = msg_def.name
                
                ros_msg = String()

                # Logic Routing
                if msg_name == 'GPS_Position':
                    ros_msg.data = f"Lat: {decoded['GPS_Latitude']}, Lon: {decoded['GPS_Longitude']}"
                    self.gps_pub.publish(ros_msg)
                
                elif msg_name == 'IMU_Data':
                    ros_msg.data = f"Accel: {decoded['IMU_AccelX']}, {decoded['IMU_AccelY']}, {decoded['IMU_AccelZ']}"
                    self.imu_pub.publish(ros_msg)
                
                elif msg_name == 'Proximity_Sensors':
                    ros_msg.data = f"Ultra: {decoded['UltraSonic_Front']}mm, IR: {decoded['IR_Distance']}mm"
                    self.prox_pub.publish(ros_msg)
                    if decoded['UltraSonic_Front'] < 500:
                        self.get_logger().warn("OBSTACLE DETECTED!")

            except KeyError:
                self.get_logger().debug(f"Unknown ID: 0x{message.arbitration_id:X}")
            except Exception as e:
                self.get_logger().error(f"Decode Error: {e}")

    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
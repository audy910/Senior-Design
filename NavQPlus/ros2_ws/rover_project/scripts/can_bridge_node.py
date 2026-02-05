#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float64MultiArray, String
import json
import os
from ament_index_python.packages import get_package_share_directory
import cantools
from rover_project.msg import Proximity

class CanBridgeNode(Node):
    def __init__(self):
        super().__init__('can_dbc_decoder')
        
        package_share_directory = get_package_share_directory('rover_project')
        dbc_file_path = os.path.join(package_share_directory, 'rover.dbc')

        try:
            self.db = cantools.database.load_file(dbc_file_path)
            self.get_logger().info(f"Successfully loaded DBC from: {dbc_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC at {dbc_file_path}: {e}")

        # 2. Setup CAN interface (SocketCAN)
        # Change 'can0' to your actual interface name
        try:
            self.bus = can.interface.Bus(channel='can1', bustype='socketcan')
            self.get_logger().info("Connected to can1.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to CAN bus: {e}")
            return

        try:
            self.proximity_id = self.db.get_message_by_name('Proximity_Sensors').frame_id
            self.get_logger().info(f"Mapping Proximity_Sensors to ID: {self.proximity_id} (0x{self.proximity_id:02x})")
        except KeyError:
            self.get_logger().error("Could not find Proximity_Sensors in DBC!")
            self.proximity_id = None

        # 3. Dynamic Publisher Setup
        # We've created a dictionary to hold publishers for each message in the DBC
        self.publishers_dict = {}
        for message in self.db.messages:
            if message.frame_id == 0x107:
                continue
            topic_name = f"can/{message.name.lower()}"
            # Using Float64MultiArray as a generic way to send all signals in a message
            self.publishers_dict[message.frame_id] = self.create_publisher(
                Float64MultiArray, topic_name, 10)

        self.proximity_publisher = self.create_publisher(Proximity, 'can/proximity_sensors', 10)

        # 4. Create a timer to poll the CAN bus
        self.timer = self.create_timer(0.01, self.can_callback)

    def can_callback(self):
        # Read a message from the bus (non-blocking)
        msg = self.bus.recv(timeout=0)
        if msg is not None:
            # Use the ID we pulled from the DBC
            if msg.arbitration_id == self.proximity_id:
                try:
                    decoded = self.db.decode_message(msg.arbitration_id, msg.data)
                    
                    ros_msg = Proximity()
                    # Mapping based on your DBC field names
                    ros_msg.proximity_front = int(decoded.get('Proximity_Front', 0))
                    ros_msg.proximity_rear = int(decoded.get('Proximity_Rear', 0))
                    ros_msg.proximity_cliff = int(decoded.get('Proximity_Cliff', 0))
                    ros_msg.cliff_detected = bool(decoded.get('Cliff_Detected', False))
                    ros_msg.front_valid = bool(decoded.get('Front_Valid', False))
                    ros_msg.rear_valid = bool(decoded.get('Rear_Valid', False))
                    ros_msg.cliff_valid = bool(decoded.get('Cliff_Valid', False))
                    
                    self.proximity_publisher.publish(ros_msg)
                except Exception as e:
                    self.get_logger().warn(f"Failed to decode Proximity frame: {e}")

                    
            else:
                # Prepare the ROS 2 message
                ros_msg = Float64MultiArray()
                # We convert the dictionary values to a list for the MultiArray
                ros_msg.data = [float(val) for val in decoded_data.values()]
                
                # Publish to the specific topic
                self.publishers_dict[msg.arbitration_id].publish(ros_msg)
                
                # Optional: Log the decode for debugging
                # self.get_logger().info(f"Published {decoded_data} to {msg.arbitration_id}")

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

if __name__ == '__main__':
    main()



















# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String # We'll use String for now, but custom msgs are better!
# import can
# import cantools
# import os
# from ament_index_python.packages import get_package_share_directory

# class CanBridgeNode(Node):
#     def __init__(self):
#         super().__init__('can_bridge_node')
        
#         package_share_directory = get_package_share_directory('rover_project')
#         dbc_path = os.path.join(package_share_directory, 'rover.dbc')
        
#         try:
#             self.db = cantools.database.load_file(dbc_path)
#             self.get_logger().info(f"DBC loaded from: {dbc_path}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to load DBC: {e}")
#             raise e

#         # 2. Initialize CAN Bus
#         try:
#             self.bus = can.interface.Bus(channel='can1', bustype='socketcan')
#             self.get_logger().info("Connected to can1")
#         except Exception as e:
#             self.get_logger().error(f"CAN Init Error: {e}")
#             raise e

#         # 3. Initialize Publishers
#         # To expand this, you'd eventually create specific topics for GPS, IMU, etc.
#         self.gps_pub = self.create_publisher(String, 'rover/gps', 10)
#         self.imu_pub = self.create_publisher(String, 'rover/imu', 10)
#         self.prox_pub = self.create_publisher(String, 'rover/proximity', 10)

#         # 4. Create a Timer for the high-frequency CAN loop (e.g., 10ms / 100Hz)
#         self.timer = self.create_timer(0.01, self.can_receive_callback)

#     def can_receive_callback(self):
#         # We read non-blocking to keep the ROS node responsive
#         message = self.bus.recv(timeout=0.0)
        
#         if message is not None:
#             try:
#                 # Decode using your logic
#                 decoded = self.db.decode_message(message.arbitration_id, message.data)
#                 msg_def = self.db.get_message_by_frame_id(message.arbitration_id)
#                 msg_name = msg_def.name
                
#                 ros_msg = String()

#                 # Logic Routing
#                 if msg_name == 'GPS_Position':
#                     ros_msg.data = f"Lat: {decoded['GPS_Latitude']}, Lon: {decoded['GPS_Longitude']}"
#                     self.gps_pub.publish(ros_msg)
                
#                 elif msg_name == 'IMU_Data':
#                     ros_msg.data = f"Accel: {decoded['IMU_AccelX']}, {decoded['IMU_AccelY']}, {decoded['IMU_AccelZ']}"
#                     self.imu_pub.publish(ros_msg)
                
#                 elif msg_name == 'Proximity_Sensors':
#                     ros_msg.data = f"Ultra: {decoded['UltraSonic_Front']}mm, IR: {decoded['IR_Distance']}mm"
#                     self.prox_pub.publish(ros_msg)
#                     if decoded['UltraSonic_Front'] < 500:
#                         self.get_logger().warn("OBSTACLE DETECTED!")

#             except KeyError:
#                 self.get_logger().debug(f"Unknown ID: 0x{message.arbitration_id:X}")
#             except Exception as e:
#                 self.get_logger().error(f"Decode Error: {e}")

#     def destroy_node(self):
#         self.bus.shutdown()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CanBridgeNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float64MultiArray
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
        # UPDATED: Changed 'bustype' to 'interface' to fix DeprecationWarning
        try:
            self.bus = can.interface.Bus(channel='can1', interface='socketcan')
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
        self.publishers_dict = {}
        for message in self.db.messages:
            # Skip the proximity ID because we have a custom publisher for it
            if self.proximity_id and message.frame_id == self.proximity_id:
                continue
                
            topic_name = f"can/{message.name.lower()}"
            self.publishers_dict[message.frame_id] = self.create_publisher(
                Float64MultiArray, topic_name, 10)

        self.proximity_publisher = self.create_publisher(Proximity, 'can/proximity_sensors', 10)

        # 4. Create a timer to poll the CAN bus
        self.timer = self.create_timer(0.01, self.can_callback)

    def can_callback(self):
        # Read a message from the bus (non-blocking)
        msg = self.bus.recv(timeout=0)
        if msg is not None:
            # CASE 1: Proximity Sensor (Custom ROS Message)
            if self.proximity_id and msg.arbitration_id == self.proximity_id:
                try:
                    decoded = self.db.decode_message(msg.arbitration_id, msg.data)
                    
                    ros_msg = Proximity()
                    ros_msg.proximity_rear = int(decoded.get('Proximity_Front', 0))
                    ros_msg.proximity_front = int(decoded.get('Proximity_Rear', 0))
                    ros_msg.proximity_cliff = int(decoded.get('Proximity_Cliff', 0))
                    ros_msg.cliff_detected = bool(decoded.get('Cliff_Detected', False))
                    ros_msg.front_valid = bool(decoded.get('Front_Valid', False))
                    ros_msg.rear_valid = bool(decoded.get('Rear_Valid', False))
                    ros_msg.cliff_valid = bool(decoded.get('Cliff_Valid', False))
                    
                    self.proximity_publisher.publish(ros_msg)
                except Exception as e:
                    self.get_logger().warn(f"Failed to decode Proximity frame: {e}")

            # CASE 2: All other DBC messages (Generic Float Array)
            elif msg.arbitration_id in self.publishers_dict:
                try:
                    # FIX: We must decode the data here before using it!
                    decoded_data = self.db.decode_message(msg.arbitration_id, msg.data)
                    
                    ros_msg = Float64MultiArray()
                    # Convert dictionary values to list
                    ros_msg.data = [float(val) for val in decoded_data.values()]
                    
                    # Publish to the specific topic
                    self.publishers_dict[msg.arbitration_id].publish(ros_msg)
                except Exception as e:
                    self.get_logger().warn(f"Failed to decode Generic frame {hex(msg.arbitration_id)}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown to prevent errors
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

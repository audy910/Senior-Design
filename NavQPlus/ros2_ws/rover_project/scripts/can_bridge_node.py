#!/usr/bin/env python3
"""
can_bridge_node.py  (UPDATED)

Changes from original:
  - GPS_Position + GPS_Velocity + GPS_Accuracy → published as GpsFix on 'can/gps'
  - IMU_Orientation → published as ImuOrientation on 'can/imu_orientation'
  - Proximity_Sensors → published as Proximity on 'can/proximity_sensors' (unchanged)
  - All other CAN messages still published as Float64MultiArray on can/<name>
"""

import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
import os
from ament_index_python.packages import get_package_share_directory
import cantools
from rover_project.msg import Proximity, GpsFix, ImuOrientation


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

        # CAN interface
        try:
            self.bus = can.interface.Bus(channel='can1', bustype='socketcan')
            self.get_logger().info("Connected to can1.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to CAN bus: {e}")
            return

        # Resolve CAN IDs from DBC
        self.id_map = {}
        for name in ['GPS_Position', 'GPS_Velocity', 'GPS_Accuracy',
                      'IMU_Orientation', 'Proximity_Sensors']:
            try:
                fid = self.db.get_message_by_name(name).frame_id
                self.id_map[name] = fid
                self.get_logger().info(f"  {name} → 0x{fid:03X}")
            except KeyError:
                self.get_logger().warn(f"  {name} not found in DBC")

        # IDs we handle with custom messages (skip generic publisher)
        self.custom_ids = set(self.id_map.values())

        # Custom Publishers
        self.gps_pub = self.create_publisher(GpsFix, 'can/gps', 1)
        self.navsatfix_pub = self.create_publisher(NavSatFix, 'can/gps_fix', 1)
        self.imu_ori_pub = self.create_publisher(ImuOrientation, 'can/imu_orientation', 1)
        self.proximity_pub = self.create_publisher(Proximity, 'can/proximity_sensors', 1)

        # Generic publishers for everything else
        self.publishers_dict = {}
        for message in self.db.messages:
            if message.frame_id in self.custom_ids:
                continue
            topic_name = f"can/{message.name.lower()}"
            self.publishers_dict[message.frame_id] = self.create_publisher(
                Float64MultiArray, topic_name, 10)

        # GPS state (accumulate across multiple CAN frames)
        self.gps_state = {
            'latitude': 0.0, 'longitude': 0.0,
            'speed': 0.0, 'course': 0.0,
            'fix_type': 0, 'num_sats': 0, 'hdop': 0.0,
            'h_acc': 0.0, 'v_acc': 0.0,
        }

        # Poll CAN bus at 100Hz
        self.timer = self.create_timer(0.01, self.can_callback)

    def can_callback(self):
        while True:
            msg = self.bus.recv(timeout=0)
            if msg is None:
                return

            arb_id = msg.arbitration_id

            try:
                decoded = self.db.decode_message(arb_id, msg.data)
            except KeyError:
                # Unknown CAN ID - not in DBC file, silently ignore
                continue
            except Exception as e:
                self.get_logger().warn(f'Failed to decode CAN ID 0x{arb_id:03X}: {e}')
                continue

            # GPS_Position (0x100)
            if arb_id == self.id_map.get('GPS_Position'):
                self.gps_state['latitude'] = decoded.get('GPS_Latitude', 0.0)
                self.gps_state['longitude'] = decoded.get('GPS_Longitude', 0.0)
                self._publish_gps()

            # GPS_Velocity (0x101)
            elif arb_id == self.id_map.get('GPS_Velocity'):
                self.gps_state['speed'] = decoded.get('GPS_Speed', 0.0)
                self.gps_state['course'] = decoded.get('GPS_Course', 0.0)
                self.gps_state['fix_type'] = int(decoded.get('GPS_FixType', 0))
                self.gps_state['num_sats'] = int(decoded.get('GPS_NumSats', 0))
                self.gps_state['hdop'] = decoded.get('GPS_HDOP', 0.0)
                self._publish_gps()

            # GPS_Accuracy (0x102)
            elif arb_id == self.id_map.get('GPS_Accuracy'):
                self.gps_state['h_acc'] = decoded.get('GPS_hAcc', 0.0)
                self.gps_state['v_acc'] = decoded.get('GPS_vAcc', 0.0)
                self._publish_gps()

            # IMU_Orientation (0x105)
            elif arb_id == self.id_map.get('IMU_Orientation'):
                imu_msg = ImuOrientation()
                imu_msg.heading = float(decoded.get('IMU_Heading', 0.0))
                imu_msg.pitch = float(decoded.get('IMU_Pitch', 0.0))
                imu_msg.roll = float(decoded.get('IMU_Roll', 0.0))
                imu_msg.cal_sys = int(decoded.get('IMU_CalSys', 0))
                imu_msg.cal_mag = int(decoded.get('IMU_CalMag', 0))
                self.imu_ori_pub.publish(imu_msg)

            # Proximity_Sensors (0x107)
            elif arb_id == self.id_map.get('Proximity_Sensors'):
                prox_msg = Proximity()
                prox_msg.proximity_front = int(decoded.get('Proximity_Front', 0))
                prox_msg.proximity_rear = int(decoded.get('Proximity_Rear', 0))
                prox_msg.proximity_cliff = int(decoded.get('Proximity_Cliff', 0))
                prox_msg.cliff_detected = bool(decoded.get('Cliff_Detected', False))
                prox_msg.front_valid = bool(decoded.get('Front_Valid', False))
                prox_msg.rear_valid = bool(decoded.get('Rear_Valid', False))
                prox_msg.cliff_valid = bool(decoded.get('Cliff_Valid', False))
                self.proximity_pub.publish(prox_msg)

            # Everything else - generic Float64MultiArray
            elif arb_id in self.publishers_dict:
                ros_msg = Float64MultiArray()
                ros_msg.data = [float(val) for val in decoded.values()]
                self.publishers_dict[arb_id].publish(ros_msg)

    def _publish_gps(self):
        """Publish the accumulated GPS state."""
        gps_msg = GpsFix()
        gps_msg.latitude = self.gps_state['latitude']
        gps_msg.longitude = self.gps_state['longitude']
        gps_msg.speed = float(self.gps_state['speed'])
        gps_msg.course = float(self.gps_state['course'])
        gps_msg.fix_type = int(self.gps_state['fix_type'])
        gps_msg.num_sats = int(self.gps_state['num_sats'])
        gps_msg.hdop = float(self.gps_state['hdop'])
        gps_msg.h_acc = float(self.gps_state['h_acc'])
        gps_msg.v_acc = float(self.gps_state['v_acc'])
        self.gps_pub.publish(gps_msg)

        # NavSatFix for Foxglove Map panel
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.latitude = self.gps_state['latitude']
        fix.longitude = self.gps_state['longitude']
        fix.altitude = float('nan')
        h_acc = float(self.gps_state['h_acc'])
        fix.position_covariance = [h_acc**2, 0.0, 0.0,
                                   0.0, h_acc**2, 0.0,
                                   0.0, 0.0, (h_acc*2)**2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        ft = int(self.gps_state['fix_type'])
        if ft == 0:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif ft >= 3:
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        self.navsatfix_pub.publish(fix)


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
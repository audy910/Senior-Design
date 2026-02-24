#!/usr/bin/env python3
"""
Standalone CAN Bus Test Script
Receives and decodes CAN messages using DBC file
No ROS2 required
"""

import can
import cantools
import sys
from datetime import datetime


class CANTester:
    def __init__(self, dbc_file, can_interface='can0'):
        """
        Initialize CAN tester

        Args:
            dbc_file: Path to DBC file
            can_interface: CAN interface name (default: can0)
        """
        print(f"Loading DBC file: {dbc_file}")
        try:
            self.db = cantools.database.load_file(dbc_file)
            print(f"✓ DBC loaded successfully")
            print(f"  Messages defined: {len(self.db.messages)}")
            for msg in self.db.messages:
                print(f"    - 0x{msg.frame_id:03X} ({msg.frame_id}): {msg.name}")
        except Exception as e:
            print(f"✗ Failed to load DBC: {e}")
            sys.exit(1)

        print(f"\nConnecting to CAN interface: {can_interface}")
        try:
            self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
            print(f"✓ Connected to {can_interface}")
        except Exception as e:
            print(f"✗ Failed to connect to CAN: {e}")
            print(f"\nTroubleshooting:")
            print(f"  1. Check if interface exists: ip link show {can_interface}")
            print(f"  2. Bring up interface: sudo ip link set {can_interface} up type can bitrate 500000")
            print(f"  3. Check for messages: candump {can_interface}")
            sys.exit(1)

        self.message_counts = {}
        self.start_time = datetime.now()

    def format_gps_position(self, data):
        """Format GPS position data"""
        lat = data['GPS_Latitude']
        lon = data['GPS_Longitude']
        return f"Position: {lat:.7f}°, {lon:.7f}° (±11mm precision)"

    def format_gps_velocity(self, data):
        """Format GPS velocity data"""
        speed = data['GPS_Speed']
        course = data['GPS_Course']
        fix = data['GPS_FixType']
        sats = data['GPS_NumSats']
        hdop = data['GPS_HDOP']

        fix_types = {0: "None", 1: "Dead Reckoning", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}
        fix_str = fix_types.get(fix, f"Unknown({fix})")

        return f"Speed: {speed:.3f} m/s, Course: {course:.2f}°, Fix: {fix_str}, Sats: {sats}, HDOP: {hdop:.1f}"

    def format_gps_accuracy(self, data):
        """Format GPS accuracy data"""
        h_acc = data['GPS_hAcc']
        v_acc = data['GPS_vAcc']
        return f"Accuracy: H={h_acc:.3f}m, V={v_acc:.3f}m"

    def format_imu_accel(self, data):
        """Format IMU acceleration data"""
        x = data['IMU_AccelX']
        y = data['IMU_AccelY']
        z = data['IMU_AccelZ']
        cal = data['IMU_CalAccel']
        cal_str = ['Uncalibrated', 'Partially', 'Mostly', 'Fully Calibrated'][cal]
        return f"Accel: X={x:+.3f}, Y={y:+.3f}, Z={z:+.3f} m/s² | Cal: {cal_str}"

    def format_imu_gyro(self, data):
        """Format IMU gyroscope data"""
        x = data['IMU_GyroX']
        y = data['IMU_GyroY']
        z = data['IMU_GyroZ']
        cal = data['IMU_CalGyro']
        cal_str = ['Uncalibrated', 'Partially', 'Mostly', 'Fully Calibrated'][cal]
        return f"Gyro: X={x:+.2f}, Y={y:+.2f}, Z={z:+.2f} °/s | Cal: {cal_str}"

    def format_imu_orientation(self, data):
        """Format IMU orientation data"""
        heading = data['IMU_Heading']
        pitch = data['IMU_Pitch']
        roll = data['IMU_Roll']
        cal_sys = data['IMU_CalSys']
        cal_mag = data['IMU_CalMag']

        sys_str = ['Uncal', 'Partial', 'Mostly', 'Fully'][cal_sys]
        mag_str = ['Uncal', 'Partial', 'Mostly', 'Fully'][cal_mag]

        return f"Orientation: H={heading:.2f}°, P={pitch:+.2f}°, R={roll:+.2f}° | Sys:{sys_str}, Mag:{mag_str}"

    def format_imu_quaternion(self, data):
        """Format IMU quaternion data"""
        w = data['IMU_QuatW']
        x = data['IMU_QuatX']
        y = data['IMU_QuatY']
        z = data['IMU_QuatZ']
        return f"Quaternion: W={w:.4f}, X={x:.4f}, Y={y:.4f}, Z={z:.4f}"

    def format_proximity_sensors(self, data):
        """Format proximity sensors data"""
        front = data['Proximity_Front']
        rear = data['Proximity_Rear']
        cliff = data['Proximity_Cliff']
        cliff_detected = data['Cliff_Detected']
        front_valid = data['Front_Valid']
        rear_valid = data['Rear_Valid']
        cliff_valid = data['Cliff_Valid']
        
        # Convert mm to cm for display
        front_cm = front / 10.0
        rear_cm = rear / 10.0
        cliff_cm = cliff / 10.0
        
        # Status indicators
        front_status = "✓" if front_valid else "✗"
        rear_status = "✓" if rear_valid else "✗"
        cliff_status = "✓" if cliff_valid else "✗"
        
        # Emergency indicator
        if cliff_detected:
            alert = "🚨 CLIFF DETECTED - EMERGENCY STOP! 🚨"
        else:
            alert = "✓ Safe"
        
        return (f"Front: {front_cm:.1f}cm {front_status} | "
                f"Rear: {rear_cm:.1f}cm {rear_status} | "
                f"Cliff: {cliff_cm:.1f}cm {cliff_status} | "
                f"{alert}")

    def print_message(self, msg_name, data, formatted):
        """Print a formatted message"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Update counter
        if msg_name not in self.message_counts:
            self.message_counts[msg_name] = 0
        self.message_counts[msg_name] += 1

        # Print header with color for cliff warnings
        if msg_name == "Proximity_Sensors" and data.get('Cliff_Detected', 0) == 1:
            print(f"\n\033[91m[{timestamp}] {msg_name} (#{self.message_counts[msg_name]})\033[0m")  # Red
        else:
            print(f"\n[{timestamp}] {msg_name} (#{self.message_counts[msg_name]})")
        
        print(f"  {formatted}")

        # Print raw values if verbose
        # print(f"  Raw: {data}")

    def decode_message(self, msg):
        """Decode a CAN message using DBC"""
        try:
            message = self.db.get_message_by_frame_id(msg.arbitration_id)
            decoded = message.decode(msg.data)
            return message.name, decoded
        except KeyError:
            return None, None
        except Exception as e:
            print(f"Error decoding 0x{msg.arbitration_id:03X}: {e}")
            return None, None

    def run(self):
        """Main loop to receive and display CAN messages"""
        print("\n" + "="*70)
        print("Waiting for CAN messages... (Ctrl+C to stop)")
        print("="*70)

        try:
            while True:
                # Receive message (blocking with 1 second timeout)
                msg = self.bus.recv(timeout=1.0)

                if msg is None:
                    continue

                # Decode message
                msg_name, decoded = self.decode_message(msg)

                if msg_name is None:
                    print(f"\n[Unknown] ID: 0x{msg.arbitration_id:03X} ({msg.arbitration_id}), Data: {msg.data.hex()}")
                    continue

                # Format based on message type
                if msg_name == "GPS_Position":
                    formatted = self.format_gps_position(decoded)
                elif msg_name == "GPS_Velocity":
                    formatted = self.format_gps_velocity(decoded)
                elif msg_name == "GPS_Accuracy":
                    formatted = self.format_gps_accuracy(decoded)
                elif msg_name == "IMU_Accel":
                    formatted = self.format_imu_accel(decoded)
                elif msg_name == "IMU_Gyro":
                    formatted = self.format_imu_gyro(decoded)
                elif msg_name == "IMU_Orientation":
                    formatted = self.format_imu_orientation(decoded)
                elif msg_name == "IMU_Quaternion":
                    formatted = self.format_imu_quaternion(decoded)
                elif msg_name == "Proximity_Sensors":
                    formatted = self.format_proximity_sensors(decoded)
                else:
                    formatted = str(decoded)

                # Print the message
                self.print_message(msg_name, decoded, formatted)

        except KeyboardInterrupt:
            print("\n\n" + "="*70)
            print("Statistics:")
            print("="*70)
            elapsed = (datetime.now() - self.start_time).total_seconds()
            print(f"Runtime: {elapsed:.1f} seconds\n")

            if self.message_counts:
                for msg_name, count in sorted(self.message_counts.items()):
                    rate = count / elapsed if elapsed > 0 else 0
                    print(f"  {msg_name:20s}: {count:5d} messages ({rate:.1f} Hz)")
            else:
                print("  No messages received")

            print("\n" + "="*70)

        finally:
            self.bus.shutdown()
            print("CAN bus closed")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description='Test CAN bus receiver with DBC decoding')
    parser.add_argument('dbc_file', help='Path to DBC file')
    parser.add_argument('--interface', '-i', default='can1', help='CAN interface (default: can1)')

    args = parser.parse_args()

    tester = CANTester(args.dbc_file, args.interface)
    tester.run()


if __name__ == '__main__':
    main()


# ## Key Changes:

# 1. **Added `format_proximity_sensors()` method** - formats all proximity data nicely
# 2. **Added color coding** - proximity messages with cliff detection show in RED
# 3. **Shows decimal IDs** - prints both hex and decimal in DBC loading and unknown messages
# 4. **Handles Proximity_Sensors** in the main run loop
# 5. **Better status indicators** - uses ✓/✗ for valid/invalid sensors

# ## Expected Output:

# ### Normal Operation:
# ```
# [14:23:45.123] Proximity_Sensors (#1)
#   Front: 125.0cm ✓ | Rear: 89.3cm ✓ | Cliff: 5.0cm ✓ | ✓ Safe
# ```

# ### Cliff Detected (RED text):
# ```
# [14:23:45.345] Proximity_Sensors (#2)
#   Front: 125.0cm ✓ | Rear: 89.3cm ✓ | Cliff: 100.0cm ✓ | 🚨 CLIFF DETECTED - EMERGENCY STOP! 🚨
# ```

# ### Invalid Readings:
# ```
# [14:23:45.567] Proximity_Sensors (#3)
#   Front: 0.0cm ✗ | Rear: 89.3cm ✓ | Cliff: 5.0cm ✓ | ✓ Safe
# ```

# ## Statistics Output Example:
# ```
# ======================================================================
# Statistics:
# ======================================================================
# Runtime: 30.5 seconds

#   GPS_Accuracy        :   305 messages (10.0 Hz)
#   GPS_Position        :   305 messages (10.0 Hz)
#   GPS_Velocity        :   305 messages (10.0 Hz)
#   IMU_Accel           :  1525 messages (50.0 Hz)
#   IMU_Gyro            :  1525 messages (50.0 Hz)
#   IMU_Orientation     :   610 messages (20.0 Hz)
#   IMU_Quaternion      :   610 messages (20.0 Hz)
#   Proximity_Sensors   :   610 messages (20.0 Hz)

# ======================================================================
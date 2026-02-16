#!/usr/bin/env python3

import can
import cantools
import sys
import time

# Load the DBC file
try:
    db = cantools.database.load_file('rover_optimized.dbc')
    print("DBC file loaded successfully")
    print(f"Messages defined: {len(db.messages)}")
    for msg in db.messages:
        print(f"  - {msg.name} (ID: {msg.frame_id}, 0x{msg.frame_id:X})")
except Exception as e:
    print(f"Error loading DBC file: {e}")
    sys.exit(1)

# Set up CAN interface
try:
    bus = can.interface.Bus(channel='can1', bustype='socketcan')
    print("CAN bus initialized on can0")
except Exception as e:
    print(f"Error initializing CAN bus: {e}")
    print("Make sure CAN interface is up: sudo ip link set can0 up type can bitrate 500000")
    sys.exit(1)

print("\nListening for CAN messages... (Press Ctrl+C to stop)\n")

# Statistics
msg_count = {}
last_print_time = time.time()

try:
    for message in bus:
        try:
            # Decode the message
            decoded = db.decode_message(message.arbitration_id, message.data)
            
            # Get message name
            msg_def = db.get_message_by_frame_id(message.arbitration_id)
            msg_name = msg_def.name
            
            # Count messages
            if msg_name not in msg_count:
                msg_count[msg_name] = 0
            msg_count[msg_name] += 1
            
            # Print decoded data based on message type
            if msg_name == 'GPS_Position':
                print(f"[GPS Position] Time: {decoded['GPS_Time']} ms")
                print(f"  Latitude:  {decoded['GPS_Latitude']:.6f}°")
                print(f"  Longitude: {decoded['GPS_Longitude']:.6f}°")
                print()
                
            elif msg_name == 'GPS_Velocity':
                print(f"[GPS Velocity] Time: {decoded['GPS_Speed_Time']} ms")
                print(f"  Speed:  {decoded['GPS_Speed']:.2f} m/s")
                print(f"  Course: {decoded['GPS_Course']:.1f}°")
                print()
                
            elif msg_name == 'IMU_Data':
                print(f"[IMU Accel] Time: {decoded['IMU_Time']} ms")
                print(f"  Accel X: {decoded['IMU_AccelX']:.3f} m/s²")
                print(f"  Accel Y: {decoded['IMU_AccelY']:.3f} m/s²")
                print(f"  Accel Z: {decoded['IMU_AccelZ']:.3f} m/s²")
                print()
                
            elif msg_name == 'IMU_Orientation':
                print(f"[IMU Orientation] Time: {decoded['IMU_Orient_Time']} ms")
                print(f"  Heading: {decoded['IMU_Heading']:.2f}°")
                print(f"  Pitch:   {decoded['IMU_Pitch']:.2f}°")
                print(f"  Roll:    {decoded['IMU_Roll']:.2f}°")
                print()
                
            elif msg_name == 'Proximity_Sensors':
                print(f"[Proximity] Time: {decoded['Proximity_Time']} ms")
                print(f"  Ultrasonic: {decoded['UltraSonic_Front']} mm")
                print(f"  IR:         {decoded['IR_Distance']} mm")
                if decoded['UltraSonic_Front'] < 500:
                    print("  ⚠️  OBSTACLE DETECTED!")
                print()
            
            # Print statistics every 5 seconds
            if time.time() - last_print_time > 5.0:
                print("=" * 50)
                print("Message Statistics:")
                for name, count in sorted(msg_count.items()):
                    print(f"  {name}: {count} messages")
                print("=" * 50)
                print()
                last_print_time = time.time()
                
        except KeyError as e:
            # Unknown message ID
            print(f"Unknown message ID: 0x{message.arbitration_id:X} ({message.arbitration_id})")
            print(f"  Data: {' '.join(f'{b:02X}' for b in message.data)}")
            print()
            
        except Exception as e:
            print(f"Error decoding message: {e}")
            print(f"  ID: 0x{message.arbitration_id:X}")
            print(f"  Data: {' '.join(f'{b:02X}' for b in message.data)}")
            print()

except KeyboardInterrupt:
    print("\n\nStopping...")
    print("\nFinal Statistics:")
    for name, count in sorted(msg_count.items()):
        print(f"  {name}: {count} messages")
    
finally:
    bus.shutdown()
    print("CAN bus closed")
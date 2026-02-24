#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rover_project.msg import Proximity
from std_msgs.msg import Float32

# Command Table
CMD_STOP             = 2
CMD_REVERSE          = 3
CMD_FORWARD_STRAIGHT = 6
CMD_FORWARD_LEFT     = 7
CMD_FORWARD_RIGHT    = 8
CMD_LEFT             = 9   # Turn sensor
CMD_RIGHT            = 10  # Turn sensor

# States
STATE_FORWARD   = "FORWARD"
STATE_REV_CHECK = "REV_CHECK"
STATE_REVERSING = "REVERSING"
STATE_LOOK_L    = "LOOK_L"
STATE_SCAN_L    = "SCAN_L"
STATE_LOOK_R    = "LOOK_R"
STATE_SCAN_R    = "SCAN_R"
STATE_DRIVE_OUT = "DRIVE_OUT"

class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')

        self.sub = self.create_subscription(
            Proximity,
            'can/proximity_sensors',
            self.proximity_callback,
            10)

        self.cmd_pub = self.create_publisher(Int32, 'bluetooth_commands', 10)
        
        self.vision_sub = self.create_subscription(
            Float32,
            '/vision/error',
            self.vision_callback,
            10)

        self.vision_error = 0.0
        self.vision_last_time = 0.0
        # Configuration
        self.wall_threshold = 500.0  
        self.reverse_time   = 1.0   
        self.sensor_time    = 0.5   
        self.maneuver_time  = 2.0   
        
        # Validation Logic Variables
        self.prev_proximity_front = 9999.0
        self.valid_reading_count = 0
        self.required_readings = 4  # Number of consecutive "getting closer" readings required

        self.current_state = STATE_FORWARD
        self.state_start_time = 0.0
        self.chosen_turn_cmd = CMD_FORWARD_LEFT

    def set_state(self, new_state):
        self.get_logger().info(f"Transitioning to: {new_state}")
        # Reset validation counter when changing behavior
        self.valid_reading_count = 0
        self.current_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
    
    def vision_callback(self, msg):
        self.vision_error = msg.data
        self.vision_last_time = self.get_clock().now().nanoseconds / 1e9
    
    def proximity_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time
        drive_msg = Int32()

        # Safety Check
        if msg.cliff_detected or not msg.front_valid:
            drive_msg.data = CMD_STOP
            self.cmd_pub.publish(drive_msg)
            return

        # --- Data Validation Logic (Forward Only) ---
        is_getting_closer = msg.proximity_front < self.prev_proximity_front
        
        # If we are moving forward and the sensor sees something within threshold
        if self.current_state == STATE_FORWARD and msg.proximity_front < self.wall_threshold:
            if is_getting_closer:
                self.valid_reading_count += 1
            else:
                # If distance increased while moving forward, it might be a sensor glitch
                self.valid_reading_count = max(0, self.valid_reading_count - 1)
        
        # Store for next comparison
        self.prev_proximity_front = msg.proximity_front

        # --- State Machine ---
        if self.current_state == STATE_FORWARD:

            # Only trigger wall reaction if it's close AND has been getting closer
            if msg.proximity_front < self.wall_threshold and self.valid_reading_count >= self.required_readings:
                self.get_logger().warn(f"Valid wall detected at {msg.proximity_front}mm")
                self.set_state(STATE_REV_CHECK)
            else:
                now = self.get_clock().now().nanoseconds / 1e9
                vision_recent = (now - self.vision_last_time) < 0.5

                if vision_recent and self.vision_error != 9999.0:

                    if self.vision_error > 60:
                        drive_msg.data = CMD_FORWARD_RIGHT
                    elif self.vision_error < -60:
                        drive_msg.data = CMD_FORWARD_LEFT
                    else:
                        drive_msg.data = CMD_FORWARD_STRAIGHT
                else:
                    drive_msg.data = CMD_STOP

        elif self.current_state == STATE_REV_CHECK:
            if msg.proximity_rear > self.wall_threshold:
                self.set_state(STATE_REVERSING)
            else:
                drive_msg.data = CMD_STOP

        elif self.current_state == STATE_REVERSING:
            if elapsed < self.reverse_time:
                drive_msg.data = CMD_REVERSE
            else:
                self.set_state(STATE_LOOK_L)

        elif self.current_state == STATE_LOOK_L:
            if elapsed < self.sensor_time:
                drive_msg.data = CMD_LEFT
                
            else:
                self.set_state(STATE_SCAN_L)

        elif self.current_state == STATE_SCAN_L:
            drive_msg.data = CMD_STOP
            if elapsed > 0.5:
                if msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_LEFT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_LOOK_R)

        elif self.current_state == STATE_LOOK_R:
            if elapsed < self.sensor_time:
                drive_msg.data = CMD_RIGHT
            else:
                self.set_state(STATE_SCAN_R)

        elif self.current_state == STATE_SCAN_R:
            drive_msg.data = CMD_STOP
            if elapsed > 0.5:
                if msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_RIGHT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_REV_CHECK)

        elif self.current_state == STATE_DRIVE_OUT:
            if elapsed < self.maneuver_time:
                drive_msg.data = self.chosen_turn_cmd
            else:
                self.set_state(STATE_FORWARD)

        self.cmd_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
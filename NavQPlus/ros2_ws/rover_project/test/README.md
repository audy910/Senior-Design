# Rover Project — Test Reference

## Running Tests with Colcon

All commands should be run from the workspace root (`ros2_ws/`).

```bash
# Build the package before testing
colcon build --packages-select rover_project

# Run all tests
colcon test --packages-select rover_project

# Run tests and stream output to terminal in real time
colcon test --packages-select rover_project --event-handlers console_direct+

# Show a summary of pass/fail after the run
colcon test-result

# Show full verbose results including individual test names
colcon test-result --verbose

# Show only failures
colcon test-result --verbose | grep FAILED

# Re-run only the tests that failed last time
colcon test --packages-select rover_project --retest-until-pass 1
```

Run a single test file directly (faster, no colcon overhead):

```bash
python3 -m pytest rover_project/test/test_path_planning.py -v
python3 -m pytest rover_project/test/test_node_integration.py -v
python3 -m pytest rover_project/test/test_device_integration.py -v

# Run a single test by name
python3 -m pytest rover_project/test/test_node_integration.py::TestAutonomousDriveStateMachine::test_cliff_detected_publishes_override_active -v
```

---

## Live Node Communication (ros2 topic)

Source your workspace first:

```bash
source install/setup.bash
```

### Monitoring Topics

```bash
# List all active topics
ros2 topic list

# Show publish rate of a topic
ros2 topic hz /can/gps

# Show full message content on a topic
ros2 topic echo /can/gps
ros2 topic echo /nav/goal
ros2 topic echo /nav/waypoints
ros2 topic echo /nav/drive_cmd
ros2 topic echo /safety/override_active
ros2 topic echo /robot_commands
ros2 topic echo /can/proximity_sensors
ros2 topic echo /can/imu_orientation

# Show message type for a topic
ros2 topic type /nav/goal
ros2 topic info /nav/goal
```

---

## Pipeline Testing Commands

### Navigation Pipeline
`audio_input → robot_commands → send_goal → /nav/goal → path_planner → /nav/waypoints → waypoint_follower → /nav/drive_cmd`

**Inject a destination command (bypasses audio input, triggers send_goal → path_planner):**

```bash
# Named destinations — send_goal maps these to lat/lon automatically
ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'HUB'"
ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'ORBACH'"
ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'RIVERA'"
ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'WCH'"
ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'BOURNS'"
ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'BELLTOWER'"
```

**Inject a lat/lon directly to /nav/goal (bypasses audio + send_goal, kicks path_planner directly):**

```bash
# Format: latitude and longitude as float64 fields
ros2 topic pub --once /nav/goal rover_project/msg/NavGoal \
  "{latitude: 33.9743, longitude: -117.3280}"

# Orbach Science Library
ros2 topic pub --once /nav/goal rover_project/msg/NavGoal \
  "{latitude: 33.9742, longitude: -117.3245}"

# Tomás Rivera Library
ros2 topic pub --once /nav/goal rover_project/msg/NavGoal \
  "{latitude: 33.9726, longitude: -117.3280}"

# Winston Chung Hall
ros2 topic pub --once /nav/goal rover_project/msg/NavGoal \
  "{latitude: 33.9750, longitude: -117.3257}"

# Bourns Hall
ros2 topic pub --once /nav/goal rover_project/msg/NavGoal \
  "{latitude: 33.9739, longitude: -117.3263}"

# Belltower
ros2 topic pub --once /nav/goal rover_project/msg/NavGoal \
  "{latitude: 33.9733, longitude: -117.3280}"
```

**Check that path_planner responded:**

```bash
ros2 topic echo /nav/waypoints    # should show list of lat/lon waypoints
ros2 topic echo /nav/path_geojson # should show GeoJSON for Foxglove visualization
```

> **Note:** path_planner requires GPS fix_type >= 2 and h_acc <= 10 m before it
> will plan a route. If waypoints are not appearing, check GPS quality first:
> `ros2 topic echo /can/gps`

---

### GPS / CAN Sensor Data

```bash
# Full GPS fix (custom message from CAN bridge)
ros2 topic echo /can/gps

# Standard NavSatFix (for Foxglove map view)
ros2 topic echo /can/gps_fix

# IMU heading/pitch/roll
ros2 topic echo /can/imu_orientation

# Proximity sensors (front, rear, cliff distances + validity flags)
ros2 topic echo /can/proximity_sensors
```

**Inject a fake GPS fix to unblock path_planner during testing:**

```bash
ros2 topic pub --once /can/gps rover_project/msg/GpsFix \
  "{latitude: 33.9737, longitude: -117.3281, fix_type: 3, num_sats: 10, h_acc: 1.5, v_acc: 2.0, speed: 0.0}"
```

**Inject fake IMU data (heading in degrees from North):**

```bash
ros2 topic pub --once /can/imu_orientation rover_project/msg/ImuOrientation \
  "{heading: 90.0, pitch: 0.0, roll: 0.0, cal_sys: 3, cal_mag: 3}"
```

---

### Obstacle Avoidance / Safety

**Trigger a cliff detection:**

```bash
ros2 topic pub --once /can/proximity_sensors rover_project/msg/Proximity \
  "{proximity_front: 800, proximity_rear: 800, proximity_cliff: 50, \
    cliff_detected: true, front_valid: true, rear_valid: true, cliff_valid: true}"
```

**Clear the cliff (open space ahead):**

```bash
ros2 topic pub --once /can/proximity_sensors rover_project/msg/Proximity \
  "{proximity_front: 0, proximity_rear: 0, proximity_cliff: 900, \
    cliff_detected: false, front_valid: false, rear_valid: true, cliff_valid: true}"
```

**Simulate a wall in front (triggers obstacle avoidance state machine):**

```bash
# front_valid=true + proximity_front < wall_threshold (400 mm) = wall detected
ros2 topic pub --once /can/proximity_sensors rover_project/msg/Proximity \
  "{proximity_front: 300, proximity_rear: 900, proximity_cliff: 900, \
    cliff_detected: false, front_valid: true, rear_valid: true, cliff_valid: true}"
```

**Check safety override state:**

```bash
ros2 topic echo /safety/override_active   # true = autonomy blocked
```

**Force-clear the safety override:**

```bash
ros2 topic pub --once /safety/override_active std_msgs/msg/Bool "data: false"
```

---

### Drive Commands / UART

**Inject a drive command directly (bypasses waypoint_follower, goes straight to uart_node):**

```bash
# CMD values: 0=OFF, 1=AI, 2=STOP, 3=BACK, 4=BACK_L, 5=BACK_R, 6=FWD, 7=FWD_L, 8=FWD_R, 9=LEFT, 10=RIGHT
ros2 topic pub --once /nav/drive_cmd std_msgs/msg/Int32 "data: 2"   # STOP
ros2 topic pub --once /nav/drive_cmd std_msgs/msg/Int32 "data: 6"   # FORWARD
ros2 topic pub --once /nav/drive_cmd std_msgs/msg/Int32 "data: 7"   # FORWARD_LEFT
ros2 topic pub --once /nav/drive_cmd std_msgs/msg/Int32 "data: 8"   # FORWARD_RIGHT
```

**Switch UART node to manual/AI mode via Bluetooth channel:**

```bash
ros2 topic pub --once /bluetooth_commands std_msgs/msg/Int32 "data: 1"   # CMD_AI → AI mode
ros2 topic pub --once /bluetooth_commands std_msgs/msg/Int32 "data: 6"   # any cmd → MANUAL mode
```

---

### Vision

**Inject a vision path error (lateral offset from camera lane center, −1.0 to 1.0):**

```bash
ros2 topic pub --once /nav/vision_error std_msgs/msg/Float32 "data: 0.0"    # centered
ros2 topic pub --once /nav/vision_error std_msgs/msg/Float32 "data: 0.5"    # drifting right
ros2 topic pub --once /nav/vision_error std_msgs/msg/Float32 "data: -0.5"   # drifting left
ros2 topic pub --once /nav/vision_error std_msgs/msg/Float32 "data: 9999.0" # blind signal
```

---

## Useful ROS2 Debugging Commands

```bash
# List all running nodes
ros2 node list

# Show a node's published and subscribed topics
ros2 node info /path_planner_node
ros2 node info /waypoint_follower_node
ros2 node info /autonomous_drive_node

# Show full message definition for a custom type
ros2 interface show rover_project/msg/NavGoal
ros2 interface show rover_project/msg/GpsFix
ros2 interface show rover_project/msg/Proximity

# Record all topics to a bag file for replay
ros2 bag record -a -o my_test_run

# Replay a recorded bag
ros2 bag record -a -o my_test_run
ros2 bag play my_test_run

# Record only specific topics
ros2 bag record /can/gps /nav/goal /nav/waypoints /nav/drive_cmd -o gps_nav_run

# Print bag info (topics, duration, message counts)
ros2 bag info my_test_run

# Check publish rate of a topic (useful for verifying CAN data is flowing)
ros2 topic hz /can/gps
ros2 topic hz /can/proximity_sensors

# Measure end-to-end latency between two topics
ros2 topic delay /nav/drive_cmd

# Show ROS2 log output for a specific node
ros2 run rover_project path_planner_node --ros-args --log-level debug
```

---

## Common Failure Diagnoses

| Symptom | What to check |
|---|---|
| `/nav/waypoints` empty after sending goal | `ros2 topic echo /can/gps` — fix_type must be ≥ 2 and h_acc ≤ 10 m |
| `/nav/path_geojson` missing in Foxglove | Same as above; also confirm `/nav/goal` was received: `ros2 topic echo /nav/goal` |
| UART node not forwarding drive commands | Check mode: publish `data: 1` to `/bluetooth_commands` to enter AI mode |
| Obstacle avoidance stuck | Check `/safety/override_active`; publish `data: false` to reset |
| No output from `send_goal` | Destination key must match exactly (case-insensitive): hub, orbach, rivera, wch, bourns, belltower |
| CAN bridge not publishing | Verify CAN interface: `ip -details link show can1` and `candump can1` |

cmake_minimum_required(VERSION 3.8)
project(rover_project)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generate Messages (existing + new)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Proximity.msg"
  "msg/GpsFix.msg"
  "msg/ImuOrientation.msg"
  "msg/NavGoal.msg"
  "msg/WaypointList.msg"
)

# 2. Install node scripts
install(PROGRAMS
  scripts/can_bridge_node.py
  scripts/uart_node.py
  scripts/autonomous_drive_node.py
  scripts/bluetooth_node.py
  scripts/path_planner_node.py
  scripts/waypoint_follower_node.py
  scripts/send_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install DBC file
install(FILES
  rover.dbc
  DESTINATION share/${PROJECT_NAME}
)

# 5. Install map data directory (shapefiles)
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  OPTIONAL
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
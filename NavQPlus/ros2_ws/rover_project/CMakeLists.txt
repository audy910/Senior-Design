cmake_minimum_required(VERSION 3.8)
project(rover_project)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generate Messages
# This automatically handles registration in the ament index
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Proximity.msg"
)

# 2. Install the nodes
# Make sure your folder name matches (we used 'scripts' in the last step)
install(PROGRAMS
  scripts/can_bridge_node.py
  scripts/uart_node.py
  scripts/autonomous_drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  rover.dbc
  DESTINATION share/${PROJECT_NAME}
)

# 3. Just install the package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
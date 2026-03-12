cmake_minimum_required(VERSION 3.8)
project(rover_project)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install pip-only Python dependencies at build time
execute_process(
  COMMAND pip3 install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
  RESULT_VARIABLE PIP_RESULT
  OUTPUT_QUIET
)
if(NOT PIP_RESULT EQUAL 0)
  message(WARNING "pip3 install of requirements.txt failed (exit code ${PIP_RESULT}). "
                  "Run: pip3 install -r requirements.txt manually.")
endif()

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
  scripts/audio_input_node.py
  scripts/audio_output_node.py
  scripts/fast_scnn_node.py
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

# 5. Install map data directory 
install(FILES
  UCR_Centerlines.json
  DESTINATION share/${PROJECT_NAME}
)

# 6. Install TFLite model + Vosk model
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  OPTIONAL
)

# 7. Install audio clips
install(DIRECTORY audio_clips/
  DESTINATION share/${PROJECT_NAME}/audio_clips
  OPTIONAL
)

install(FILES package.xml requirements.txt DESTINATION share/${PROJECT_NAME})

ament_package()
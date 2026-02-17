from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Launch Arguments
        DeclareLaunchArgument(
            'shapefile_path',
            default_value='/home/user/ros2_ws/install/rover_project/share/rover_project/maps/UCR_Centerlines.shp',
            description='Path to UCR_Centerlines.shp'
        ),

        # CAN Bridge (ESP32 sensor data)
        Node(
            package='rover_project',
            executable='can_bridge_node.py',
            name='can_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        ),

        # UART Node (Arduino motor commands)
        Node(
            package='rover_project',
            executable='uart_node.py',
            name='uart_processor',
            output='screen',
        ),

        # Path Planner (centerlines → waypoints)
        Node(
            package='rover_project',
            executable='path_planner_node.py',
            name='path_planner',
            output='screen',
            parameters=[{
                'shapefile_path': LaunchConfiguration('shapefile_path'),
                'walk_paths_only': False,      # True = only Walk_Path=Yes (187 of 766)
                'snap_tolerance_ft': 2.0,      # merge nearby nodes at intersections
                'max_snap_distance_ft': 500.0, # max dist to snap GPS to network
                'replan_deviation_m': 5.0,
            }]
        ),

        # Waypoint Follower (GPS+IMU → motor commands)
        Node(
            package='rover_project',
            executable='waypoint_follower_node.py',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'waypoint_reached_m': 2.5,
                'heading_tolerance_deg': 15.0,
                'sharp_turn_deg': 45.0,
                'command_rate_hz': 5.0,
                'gps_timeout_s': 5.0,
                'min_fix_type': 2,
                'obstacle_stop_mm': 400,
                'heading_offset_deg': 0.0,
            }]
        ),

        # Foxglove Bridge (visualization)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8770,
                'address': '0.0.0.0',
                'send_buffer_limit': 10000000
            }]
        ),
    ])
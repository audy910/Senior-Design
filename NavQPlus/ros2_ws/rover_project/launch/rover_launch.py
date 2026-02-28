import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_project')
    default_shapefile = os.path.join(pkg_share, 'maps', 'UCR_Centerlines.shp')

    return LaunchDescription([

        # Launch Arguments
        DeclareLaunchArgument(
            'shapefile_path',
            default_value=default_shapefile,
            description='Path to UCR_Centerlines.shp'
        ),

        DeclareLaunchArgument(
            'enable_reactive_drive',
            default_value='false',
            description='Enable legacy autonomous_drive_node (publishes manual bluetooth commands)'
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

        Node(
            package='rover_project',
            executable='fast_scnn_node.py',
            name='fast_scnn_node',
            output='screen',
            emulate_tty=True,
            prefix=['nice -n 10']   # lower CPU priority
        ),


        # Path Planner (centerlines → waypoints)
        Node(
            package='rover_project',
            executable='path_planner_node.py',
            name='path_planner',
            output='screen',
            parameters=[{
                'shapefile_path': LaunchConfiguration('shapefile_path'),
                'walk_paths_only': True,       # True = only Walk_Path=Yes (187 of 766, avoids roads)
                'snap_tolerance_ft': 2.0,      # merge nearby nodes at intersections
                'max_snap_distance_ft': 500.0, # max dist to snap GPS to network
                'replan_deviation_m': 5.0,
                'replan_cooldown_s': 20.0,
                'max_h_acc_m': 10.0,
            }]
        ),

        # Autonomous Drive (safety override: cliff + wall avoidance)
        # Publishes safety/override_active to silence waypoint_follower during corrections.
        Node(
            package='rover_project',
            executable='autonomous_drive_node.py',
            name='autonomous_drive',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'invert_drive': True,          # must match waypoint_follower invert_drive
                'wall_threshold_mm': 500.0,
                'reverse_time_s': 1.0,
                'sensor_time_s': 0.5,
                'maneuver_time_s': 2.0,
                'required_readings': 4,
                'cliff_hold_s': 1.0,
            }]
        ),

        # Waypoint Follower (GPS+IMU → motor commands)
        Node(
            package='rover_project',
            executable='waypoint_follower_node.py',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'waypoint_reached_m': 3.0,
                'heading_tolerance_deg': 60.0,
                'sharp_turn_deg': 40.0,
                'heading_hysteresis_deg': 10.0,
                'vision_assist_max_heading_err_deg': 35.0,
                'command_rate_hz': 5.0,
                'gps_timeout_s': 5.0,
                'min_fix_type': 2,
                'obstacle_stop_mm': 400,
                'heading_offset_deg': 180.0,
                'max_h_acc_m': 10.0,
                'invert_drive': True,          # set False if FORWARD/BACKWARD are correct
                'vision_timeout_s': 0.6,
                'vision_error_deadband_px': 40.0,
                'vision_error_strong_px': 100.0,
            }]
        ),

        # Auto-send default goal after 5s (gives path planner time to load shapefile)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rover_project',
                    executable='send_goal.py',
                    name='goal_sender',
                    output='screen',
                    arguments=['33.97400', '-117.32800'],
                )
            ]
        ),
        
        # Foxglove Bridge (visualization)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8770,
                'address': '0.0.0.0',
                'send_buffer_limit': 100000

            }]
        ),
    ])
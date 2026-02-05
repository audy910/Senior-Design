from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CAN Node (ESP32 Comms)
        Node(
            package='rover_project',
            executable='can_bridge_node.py',
            name='can_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # UART Node (Arduino Comms)
        Node(
            package='rover_project',
            executable='uart_node.py',
            name='uart_processor'
        ),

        # Auto Drive Feature
        Node(
            package='rover_project',
            executable='autonomous_drive_node.py',
            name='auto_drive'
        ),
        # 2. Foxglove Bridge Node
        # We use port 8765 to avoid the "Bind Error" you saw earlier
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8770,
                'address': '0.0.0.0',
                'send_buffer_limit': 10000000
            }]
        )
    ])
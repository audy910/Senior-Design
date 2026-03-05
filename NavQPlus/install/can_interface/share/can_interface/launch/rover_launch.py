from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Your CAN Bridge Node
        Node(
            package='can_interface',
            executable='can_bridge_node',
            name='can_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False
            }]
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
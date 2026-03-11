#!/usr/bin/env python3
"""
send_goal.py — Listen for destination name commands on 'nav/destination'
and publish the corresponding GPS NavGoal on 'nav/goal'.

Supported destinations (case-insensitive):
  hub           -> The HUB (Student Services Building)
  orbach        -> Orbach Science Library
  rivera        -> Tomás Rivera Library
  winston_chung -> Winston Chung Hall

Publish a command:
  ros2 topic pub --once /nav/destination std_msgs/msg/String "data: 'hub'"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rover_project.msg import NavGoal


# UCR campus destination coordinates (WGS84)
DESTINATIONS = {
    "hub": {
        "name": "The HUB (Student Services Building)",
        "latitude":  33.9743,
        "longitude": -117.328,
    },
    "orbach": {
        "name": "Orbach Science Library",
        "latitude":  33.9742,
        "longitude": -117.3245,
    },
    "rivera": {
        "name": "Tomás Rivera Library",
        "latitude":  33.9726,
        "longitude": -117.328,
    },
    "winston_chung": {
        "name": "Winston Chung Hall",
        "latitude":  33.975,
        "longitude": -117.3257,
    },
    "bell_tower": {
        "name": "Bell Tower",
        "latitude": 33.9733,
        "longitude": -117.328,
    }
}


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self.pub = self.create_publisher(NavGoal, 'nav/goal', 10)
        self.sub = self.create_subscription(
            String,
            'nav/destination',
            self.destination_callback,
            10,
        )

        keys = ', '.join(DESTINATIONS.keys())
        self.get_logger().info(
            f"GoalSender ready. Listening on /nav/destination. "
            f"Valid destinations: {keys}"
        )

    def destination_callback(self, msg: String):
        key = msg.data.strip().lower().replace(' ', '_')

        if key not in DESTINATIONS:
            valid = ', '.join(DESTINATIONS.keys())
            self.get_logger().warn(
                f"Unknown destination '{msg.data}'. Valid options: {valid}"
            )
            return

        dest = DESTINATIONS[key]
        nav_msg = NavGoal()
        nav_msg.latitude = dest["latitude"]
        nav_msg.longitude = dest["longitude"]

        self.pub.publish(nav_msg)
        self.get_logger().info(
            f"Goal set -> {dest['name']} "
            f"({dest['latitude']:.6f}, {dest['longitude']:.6f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

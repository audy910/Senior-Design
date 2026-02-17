#!/usr/bin/env python3
"""
send_goal.py — Publish a navigation goal from the command line.

Usage:
  ros2 run rover_project send_goal.py 33.4484 -112.0740
"""

import rclpy
from rclpy.node import Node
from rover_project.msg import NavGoal
import sys
import time


def main():
    if len(sys.argv) != 3:
        print("Usage: ros2 run rover_project send_goal.py <lat> <lon>")
        print("  e.g: ros2 run rover_project send_goal.py 33.4484 -112.0740")
        sys.exit(1)

    lat = float(sys.argv[1])
    lon = float(sys.argv[2])

    rclpy.init()
    node = rclpy.create_node('goal_sender')
    pub = node.create_publisher(NavGoal, 'nav/goal', 10)

    time.sleep(1.0)  # wait for subscribers

    msg = NavGoal()
    msg.latitude = lat
    msg.longitude = lon

    for _ in range(5):
        pub.publish(msg)
        node.get_logger().info(f"Published goal: ({lat}, {lon})")
        time.sleep(0.3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
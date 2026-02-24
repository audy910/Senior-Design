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
    from rclpy.utilities import remove_ros_args
    user_args = remove_ros_args(sys.argv)[1:]

    if len(user_args) != 2:
        print("Usage: ros2 run rover_project send_goal.py <lat> <lon>")
        print("  e.g: ros2 run rover_project send_goal.py 33.4484 -112.0740")
        sys.exit(1)

    lat = float(user_args[0])
    lon = float(user_args[1])

    rclpy.init()
    node = rclpy.create_node('goal_sender')
    pub = node.create_publisher(NavGoal, 'nav/goal', 10)

    time.sleep(1.0)  # wait for subscribers

    msg = NavGoal()
    msg.latitude = lat
    msg.longitude = lon

    pub.publish(msg)
    node.get_logger().info(f"Published goal: ({lat}, {lon})")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
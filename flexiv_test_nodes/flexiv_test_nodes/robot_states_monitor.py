#!/usr/bin/env python3
"""Example subscriber node demonstrating how to receive and process robot
states published by the robot_states_publisher node.

Author: Flexiv Robotics
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from flexiv_msgs.msg import RobotStates
import math


class StateMonitor(Node):
    """Example ROS2 node that subscribes to robot states and displays key
    information."""

    def __init__(self):
        super().__init__("state_monitor")

        # Declare a parameter for the robot serial number
        self.declare_parameter("robot_sn", "Rizon4-000000")
        robot_sn = self.get_parameter("robot_sn").get_parameter_value().string_value

        # Construct topic name dynamically based on robot SN
        # ROS topic names can't contain dashes, so replace with underscores
        topic_robot_sn = robot_sn.replace("-", "_")
        topic_name = f"/{topic_robot_sn}/flexiv_robot_states"

        self.get_logger().info(f"State Monitor started")
        self.get_logger().info(f"Subscribing to topic: {topic_name}")

        # Create subscription
        self.subscription = self.create_subscription(
            RobotStates, topic_name, self.callback, 10
        )

    def callback(self, msg):
        """Callback function that processes received robot states.

        Args:
            msg: RobotStates message containing complete robot state information
        """
        # Access joint positions (7 joints)
        self.get_logger().info(f'Joint positions: {[f"{j:.3f}" for j in msg.q]}')

        # Access TCP position
        tcp = msg.tcp_pose.pose.position
        self.get_logger().info(
            f"TCP Position: x={tcp.x:.3f}, y={tcp.y:.3f}, z={tcp.z:.3f}"
        )

        # Access TCP orientation (quaternion)
        tcp_ori = msg.tcp_pose.pose.orientation
        self.get_logger().info(
            f"TCP Orientation: w={tcp_ori.w:.3f}, x={tcp_ori.x:.3f}, "
            f"y={tcp_ori.y:.3f}, z={tcp_ori.z:.3f}"
        )

        # Access external force magnitude
        force = msg.ext_wrench_in_tcp.wrench.force
        force_mag = math.sqrt(force.x**2 + force.y**2 + force.z**2)
        self.get_logger().info(f"External Force Magnitude: {force_mag:.3f} N")

        self.get_logger().info("---")


def main(args=None):
    """Main entry point for the State Monitor example node."""
    rclpy.init(args=args)

    try:
        node = StateMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt, shutting down...")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

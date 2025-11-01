#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from flexiv_msgs.msg import RobotStates

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')

        # Declare a parameter for the robot serial number
        self.declare_parameter('robot_sn', 'Rizon4-000000')
        robot_sn = self.get_parameter('robot_sn').get_parameter_value().string_value

        # Construct topic name dynamically based on robot SN
        # ROS topic names can't contain dashes
        topic_robot_sn = robot_sn.replace('-', '_')
        topic_name = f'/{topic_robot_sn}/flexiv_robot_states'
        self.get_logger().info(f'Subscribing to topic: {topic_name}')

        # Create subscription
        self.subscription = self.create_subscription(
            RobotStates,
            topic_name,
            self.callback,
            10
        )

    def callback(self, msg):
        # Access joint positions
        self.get_logger().info(f'Joint 1: {msg.q[0]:.3f} rad')

        # Access TCP position
        tcp = msg.tcp_pose.pose.position
        self.get_logger().info(f'TCP: [{tcp.x:.3f}, {tcp.y:.3f}, {tcp.z:.3f}]')


def main():
    rclpy.init()
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
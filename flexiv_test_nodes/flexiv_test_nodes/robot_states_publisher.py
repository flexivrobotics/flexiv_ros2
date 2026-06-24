#!/usr/bin/env python3
"""This node reads robot states from Flexiv RDK and publishes them as ROS2
messages (flexiv_msgs/msg/RobotStates).

Author: Flexiv Robotics
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import flexivrdk
import sys
import argparse

# Import ROS2 message types
from flexiv_msgs.msg import RobotStates
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped


class RobotStatesPublisher(Node):
    """ROS2 node that publishes Flexiv robot states from RDK to ROS2 topics."""

    def __init__(self, robot_sn: str):
        super().__init__("rdk_state_publisher")

        # Declare parameters
        self.declare_parameter("robot_sn", robot_sn)
        self.declare_parameter("publish_rate", 100)  # Default 100 Hz
        # RDK v2.1 robot.states() returns a dict keyed by JointGroup. Select which group's states
        # to publish on the single topic. "ALL" = the whole robot (7 joints single-arm, 14 dual);
        # use "ARM_1"/"ARM_2" for an individual arm of a dual-arm robot.
        self.declare_parameter("joint_group", "ALL")

        # Get parameters
        self.robot_sn = self.get_parameter("robot_sn").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.joint_group_name = self.get_parameter("joint_group").value
        try:
            self.joint_group = getattr(flexivrdk.JointGroup, self.joint_group_name)
        except AttributeError:
            self.get_logger().error(
                f"Invalid joint_group '{self.joint_group_name}', falling back to 'ALL'"
            )
            self.joint_group_name = "ALL"
            self.joint_group = flexivrdk.JointGroup.ALL

        self.get_logger().info(
            f"Initializing Robot States Publisher for robot {self.robot_sn}"
        )
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Publishing joint group: {self.joint_group_name}")

        # Initialize RDK Robot instance. RDK v2.1's Robot constructor is
        # Robot(robot_sn, verbose=True, lite=False) — network-interface whitelisting was removed.
        try:
            self.robot = flexivrdk.Robot(self.robot_sn)
            self.get_logger().info("Successfully connected to robot via RDK")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {str(e)}")
            raise

        # Setup QoS profile for real-time communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Create publisher for robot states
        # Sanitize robot_sn for ROS2 topic name (replace dashes with underscores)
        topic_name = f'/{self.robot_sn.replace("-", "_")}/flexiv_robot_states'
        self.state_publisher = self.create_publisher(
            RobotStates, topic_name, qos_profile
        )

        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_robot_states)

        self.get_logger().info("Robot States Publisher initialized successfully")
        self.get_logger().info(f"Publishing to: {topic_name}")

        # Log robot operational status
        self.log_robot_status()

    def log_robot_status(self):
        """Log the current robot status from RDK."""
        try:
            is_busy = self.robot.busy()
            is_operational = self.robot.operational()
            is_fault = self.robot.fault()
            is_reduced = self.robot.reduced()

            self.get_logger().info("=== Robot Status ===")
            self.get_logger().info(f"Busy: {is_busy}")
            self.get_logger().info(f"Operational: {is_operational}")
            self.get_logger().info(f"Fault: {is_fault}")
            self.get_logger().info(f"Reduced Mode: {is_reduced}")
            self.get_logger().info("===================")

            if is_fault:
                self.get_logger().warn("Robot is in FAULT state!")
            elif not is_operational:
                self.get_logger().warn("Robot is NOT operational!")

        except Exception as e:
            self.get_logger().error(f"Error checking robot status: {str(e)}")

    def create_pose_stamped(self, pose_data, frame_id="world"):
        """Convert RDK pose data [x, y, z, q_w, q_x, q_y, q_z] to
        PoseStamped."""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame_id

        pose_stamped.pose.position.x = pose_data[0]
        pose_stamped.pose.position.y = pose_data[1]
        pose_stamped.pose.position.z = pose_data[2]

        pose_stamped.pose.orientation.w = pose_data[3]
        pose_stamped.pose.orientation.x = pose_data[4]
        pose_stamped.pose.orientation.y = pose_data[5]
        pose_stamped.pose.orientation.z = pose_data[6]

        return pose_stamped

    def create_twist_stamped(self, vel_data, frame_id="world"):
        """Convert RDK velocity data [v_x, v_y, v_z, w_x, w_y, w_z] to
        TwistStamped."""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = frame_id

        # Linear velocity
        twist_stamped.twist.linear.x = vel_data[0]
        twist_stamped.twist.linear.y = vel_data[1]
        twist_stamped.twist.linear.z = vel_data[2]

        # Angular velocity
        twist_stamped.twist.angular.x = vel_data[3]
        twist_stamped.twist.angular.y = vel_data[4]
        twist_stamped.twist.angular.z = vel_data[5]

        return twist_stamped

    def create_wrench_stamped(self, wrench_data, frame_id):
        """Convert RDK wrench data [f_x, f_y, f_z, m_x, m_y, m_z] to
        WrenchStamped."""
        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = self.get_clock().now().to_msg()
        wrench_stamped.header.frame_id = frame_id

        # Force
        wrench_stamped.wrench.force.x = wrench_data[0]
        wrench_stamped.wrench.force.y = wrench_data[1]
        wrench_stamped.wrench.force.z = wrench_data[2]

        # Torque/Moment
        wrench_stamped.wrench.torque.x = wrench_data[3]
        wrench_stamped.wrench.torque.y = wrench_data[4]
        wrench_stamped.wrench.torque.z = wrench_data[5]

        return wrench_stamped

    def publish_robot_states(self):
        """Main callback function to read robot states and publish as ROS2
        message."""
        try:
            # Get robot states from RDK. v2.1 returns a dict keyed by JointGroup; select the
            # configured group (default ALL = whole robot).
            all_states = self.robot.states()
            if self.joint_group not in all_states:
                self.get_logger().error(
                    f"Joint group '{self.joint_group_name}' not reported by the robot. "
                    f"Available: {[str(g) for g in all_states.keys()]}",
                    throttle_duration_sec=1.0,
                )
                return
            robot_states = all_states[self.joint_group]

            # Create ROS2 RobotStates message
            msg = RobotStates()

            # Populate header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"

            # Robot timestamp
            msg.robot_timestamp.sec = robot_states.timestamp[0]
            msg.robot_timestamp.nanosec = robot_states.timestamp[1]

            # Joint-space states (array size = DoF of the selected joint group)
            msg.q = list(robot_states.q)  # Joint positions (link-side)
            msg.theta = list(robot_states.theta)  # Joint positions (motor-side)
            msg.dq = list(robot_states.dq)  # Joint velocities (link-side)
            msg.dtheta = list(robot_states.dtheta)  # Joint velocities (motor-side)
            msg.tau = list(robot_states.tau)  # Joint torques
            msg.tau_dot = list(robot_states.tau_dot)  # Joint torque derivatives
            msg.tau_ext = list(robot_states.tau_ext)  # External joint torques
            msg.tau_interact = list(
                robot_states.tau_interact
            )  # Interaction joint torques
            msg.temperature = list(robot_states.temperature)  # Joint temperatures

            # Cartesian-space states using geometry_msgs
            # TCP pose: [x, y, z, q_w, q_x, q_y, q_z]
            msg.tcp_pose = self.create_pose_stamped(robot_states.tcp_pose, "world")

            # TCP twist: [v_x, v_y, v_z, w_x, w_y, w_z]
            msg.tcp_twist = self.create_twist_stamped(robot_states.tcp_twist, "world")

            # Flange pose: [x, y, z, q_w, q_x, q_y, q_z]
            msg.flange_pose = self.create_pose_stamped(
                robot_states.flange_pose, "world"
            )

            # Force-torque sensor reading: [f_x, f_y, f_z, m_x, m_y, m_z]
            msg.raw_ft_sensor = self.create_wrench_stamped(
                robot_states.raw_ft_sensor, "flange"
            )

            # External wrench in TCP frame: [f_x, f_y, f_z, m_x, m_y, m_z]
            msg.tcp_wrench_local = self.create_wrench_stamped(
                robot_states.tcp_wrench_local, "flange"
            )

            # External wrench in world frame: [f_x, f_y, f_z, m_x, m_y, m_z]
            msg.tcp_wrench = self.create_wrench_stamped(
                robot_states.tcp_wrench, "world"
            )

            # External wrench in TCP frame (raw): [f_x, f_y, f_z, m_x, m_y, m_z]
            msg.raw_tcp_wrench_local = self.create_wrench_stamped(
                robot_states.raw_tcp_wrench_local, "flange"
            )

            # External wrench in world frame (raw): [f_x, f_y, f_z, m_x, m_y, m_z]
            msg.raw_tcp_wrench = self.create_wrench_stamped(
                robot_states.raw_tcp_wrench, "world"
            )

            # Publish the message
            self.state_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(
                f"Error publishing robot states: {str(e)}", throttle_duration_sec=1.0
            )

    def destroy_node(self):
        """Clean shutdown of the node."""
        self.get_logger().info("Shutting down Robot States Publisher...")
        try:
            # Stop the robot safely if needed
            if hasattr(self, "robot"):
                self.get_logger().info("Disconnecting from robot...")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")
        finally:
            super().destroy_node()


def main(args=None):
    """Main entry point for the Robot States Publisher node."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Flexiv Robot States Publisher Node")
    parser.add_argument(
        "--robot-sn",
        type=str,
        required=True,
        help="Robot serial number (e.g., Enlight-L-123456)",
    )

    # Parse known args (ROS2 args are handled separately)
    parsed_args, unknown = parser.parse_known_args()

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        # Create and spin the node
        node = RobotStatesPublisher(robot_sn=parsed_args.robot_sn)

        # Spin the node
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1
    finally:
        # Cleanup
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())

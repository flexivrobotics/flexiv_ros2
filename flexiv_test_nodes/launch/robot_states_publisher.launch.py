"""Launch file for Robot States Publisher Node.

Example usage:
    ros2 launch flexiv_test_nodes robot_states_publisher.launch.py \
        robot_sn:=Enlight-L-123456

    # Custom publish rate:
    ros2 launch flexiv_test_nodes robot_states_publisher.launch.py \
        robot_sn:=Enlight-L-123456 \
        publish_rate:=200

    # Publish a single arm of a dual-arm robot:
    ros2 launch flexiv_test_nodes robot_states_publisher.launch.py \
        robot_sn:=Enlight-LL-123456 \
        joint_group:=ARM_1

Author: Flexiv Robotics
License: Apache-2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Robot States Publisher."""

    # Declare launch arguments
    robot_sn_arg = DeclareLaunchArgument(
        "robot_sn", description="Robot serial number (e.g., Enlight-L-123456)"
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="100",
        description="Publishing rate in Hz (default: 100)",
    )

    joint_group_arg = DeclareLaunchArgument(
        "joint_group",
        default_value="ALL",
        description="RDK joint group whose states to publish: ALL (whole robot), ARM_1 (left), "
        "or ARM_2 (right).",
    )

    # Create the Robot States Publisher node
    robot_states_publisher_node = Node(
        package="flexiv_test_nodes",
        executable="robot_states_publisher",
        name="robot_states_publisher",
        output="screen",
        parameters=[
            {
                "robot_sn": LaunchConfiguration("robot_sn"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "joint_group": LaunchConfiguration("joint_group"),
            }
        ],
        arguments=[
            "--robot-sn",
            LaunchConfiguration("robot_sn"),
        ],
    )

    return LaunchDescription(
        [
            robot_sn_arg,
            publish_rate_arg,
            joint_group_arg,
            robot_states_publisher_node,
        ]
    )

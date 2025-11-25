"""Launch file for Robot States Publisher Node.

Example usage:
    # Auto-detect network interface:
    ros2 launch flexiv_test_nodes robot_states_publisher.launch.py \
        robot_sn:=Rizon4s-123456

    # With specific network interface:
    ros2 launch flexiv_test_nodes robot_states_publisher.launch.py \
        robot_sn:=Rizon4s-123456 \
        network_interface:=eth0

    # Custom publish rate:
    ros2 launch flexiv_test_nodes robot_states_publisher.launch.py \
        robot_sn:=Rizon4s-123456 \
        publish_rate:=200

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
        "robot_sn", description="Robot serial number (e.g., Rizon4s-123456)"
    )

    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="",
        description="Network interface name (e.g., eth0, enp0s31f6). Leave empty to auto-detect.",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="100",
        description="Publishing rate in Hz (default: 100)",
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
                "network_interface": LaunchConfiguration("network_interface"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }
        ],
        arguments=[
            "--robot-sn",
            LaunchConfiguration("robot_sn"),
            "--network-interface",
            LaunchConfiguration("network_interface"),
        ],
    )

    return LaunchDescription(
        [
            robot_sn_arg,
            network_interface_arg,
            publish_rate_arg,
            robot_states_publisher_node,
        ]
    )

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("flexiv_bringup"),
            "config",
            "joint_position_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="flexiv_test_nodes",
                executable="publisher_joint_position",
                name="publisher_joint_position_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )

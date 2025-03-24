from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_sn = LaunchConfiguration("robot_sn")

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("flexiv_bringup"),
            "config",
            "joint_trajectory_position_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_sn",
                description="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
            ),
            Node(
                package="flexiv_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_joint_trajectory_controller",
                parameters=[ParameterFile(position_goals, allow_substs=True)],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
        ]
    )

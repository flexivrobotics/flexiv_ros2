from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_sn",
                description="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
            ),
            DeclareLaunchArgument(
                name="mode",
                default_value="pure_motion",
                choices=["pure_motion", "motion_force"],
                description="pure_motion: TCP sine-sweep with online impedance, null-space and "
                "contact wrench changes. motion_force: search for contact, then press along Z.",
            ),
            DeclareLaunchArgument(
                name="hold",
                default_value="false",
                description="pure_motion only: hold the TCP instead of sweeping it",
            ),
            DeclareLaunchArgument(
                name="polish",
                default_value="false",
                description="motion_force only: sweep along world Y while pressing",
            ),
            DeclareLaunchArgument(
                name="force_frame",
                default_value="world",
                choices=["world", "tcp"],
                description="motion_force only: reference frame of force control",
            ),
            Node(
                package="flexiv_test_nodes",
                executable="cartesian_motion_force_example",
                name="cartesian_motion_force_example",
                parameters=[
                    {
                        "robot_sn": LaunchConfiguration("robot_sn"),
                        "mode": LaunchConfiguration("mode"),
                        "hold": ParameterValue(
                            LaunchConfiguration("hold"), value_type=bool
                        ),
                        "polish": ParameterValue(
                            LaunchConfiguration("polish"), value_type=bool
                        ),
                        "force_frame": LaunchConfiguration("force_frame"),
                    }
                ],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
        ]
    )

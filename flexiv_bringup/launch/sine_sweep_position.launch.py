from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sine_sweep_config = PathJoinSubstitution(
        [
            FindPackageShare("flexiv_bringup"),
            "config",
            "sine_sweep_position_config.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="flexiv_test_nodes",
                executable="sine_sweep_position_controller",
                name="sine_sweep_position_controller",
                parameters=[sine_sweep_config],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )

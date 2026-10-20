from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def publisher_node(node_name, config_file, condition):
    """Joint trajectory publisher whose node name must match the config file's
    root key."""
    return Node(
        package="flexiv_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name=node_name,
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [FindPackageShare("flexiv_bringup"), "config", config_file]
                ),
                allow_substs=True,
            )
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=condition,
    )


def generate_launch_description():
    arm = LaunchConfiguration("arm")
    right_start_delay = LaunchConfiguration("right_start_delay")

    single = publisher_node(
        "publisher_joint_trajectory_controller",
        "joint_trajectory_position_publisher.yaml",
        IfCondition(PythonExpression(["'", arm, "' == 'single'"])),
    )
    left = publisher_node(
        "left_publisher_joint_trajectory_controller",
        "joint_trajectory_position_publisher_left.yaml",
        IfCondition(PythonExpression(["'", arm, "' in ['left', 'both']"])),
    )
    right = publisher_node(
        "right_publisher_joint_trajectory_controller",
        "joint_trajectory_position_publisher_right.yaml",
        IfCondition(PythonExpression(["'", arm, "' in ['right', 'both']"])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_sn",
                description="Serial number of the robot to connect to. Remove any space, for example: Enlight-L-123456",
            ),
            DeclareLaunchArgument(
                name="arm",
                default_value="single",
                choices=["single", "left", "right", "both"],
                description="Which publisher(s) to run. 'single': one publisher on "
                "flexiv_arm_controller, for a single-arm robot. 'left'/'right': one publisher on "
                "left_/right_flexiv_arm_controller, for a dual-arm robot. 'both': the independence "
                "demo - the LEFT publisher starts immediately and the RIGHT publisher "
                "'right_start_delay' seconds later, so the right-arm goal always arrives while the "
                "left arm is still executing its trajectory.",
            ),
            DeclareLaunchArgument(
                name="right_start_delay",
                default_value="4.0",
                description="Seconds to delay the RIGHT-arm publisher by when arm:=both. The "
                "shipped left-arm config runs a 10 s trajectory, so the default puts the "
                "right-arm goal 4 s into it.",
            ),
            single,
            left,
            # Delayed so the right-arm goal lands mid-motion on the left arm.
            TimerAction(period=right_start_delay, actions=[right]),
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_sn_param_name = "robot_sn"
    gripper_name_param_name = "gripper_name"
    use_fake_hardware_param_name = "use_fake_hardware"
    gripper_joint_names_param_name = "gripper_joint_names"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_sn_param_name,
            description="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_name_param_name,
            description="Full name of the gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
            default_value="Flexiv-GN01",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            use_fake_hardware_param_name,
            default_value="false",
            description="Start gripper with fake gripper joint states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_joint_names_param_name,
            description="Control joint names of the mounted gripper.",
            default_value="[finger_width_joint]",
        )
    )

    # Initialize arguments
    robot_sn = LaunchConfiguration(robot_sn_param_name)
    gripper_name = LaunchConfiguration(gripper_name_param_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_param_name)
    gripper_joint_names = LaunchConfiguration(gripper_joint_names_param_name)

    gripper_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_gripper"), "config", "flexiv_gripper_node.yaml"]
    )

    # Flexiv gripper node
    flexiv_gripper_node = Node(
        package="flexiv_gripper",
        executable="flexiv_gripper_node",
        name="flexiv_gripper_node",
        parameters=[
            {
                "robot_sn": robot_sn,
                "gripper_name": gripper_name,
                "gripper_joint_names": gripper_joint_names,
            },
            gripper_config_file,
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    nodes = [flexiv_gripper_node]

    return LaunchDescription(declared_arguments + nodes)

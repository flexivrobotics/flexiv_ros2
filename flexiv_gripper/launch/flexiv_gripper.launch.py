import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper_node_param_name = "gripper_node_name"
    robot_sn_param_name = "robot_sn"
    gripper_name_param_name = "gripper_name"
    use_fake_hardware_param_name = "use_fake_hardware"
    use_lite_rdk_param_name = "use_lite_rdk"
    gripper_joint_names_param_name = "gripper_joint_names"
    joint_group_param_name = "joint_group"
    rdk_install_prefix_param_name = "rdk_install_prefix"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_node_param_name,
            default_value="flexiv_gripper_node",
            description="Name of the flexiv gripper node.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_sn_param_name,
            description="Serial number of the robot to connect to. Remove any space, for example: Enlight-L-123456",
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
            use_lite_rdk_param_name,
            default_value="false",
            description="Use a lite RDK instance. Requires another normal RDK instance, such as the robot driver, to already be connected.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_joint_names_param_name,
            description="Control joint names of the mounted gripper.",
            default_value="[finger_width_joint]",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            joint_group_param_name,
            default_value="",
            description="Joint group whose gripper to control (e.g. ARM_1 or ARM_2). Leave empty to "
            "auto-detect on single-arm robots; must be set for dual-arm robots.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            rdk_install_prefix_param_name,
            default_value=os.path.expanduser("~/rdk_install"),
            description="Prefix where flexiv_rdk and its shared-library dependencies are installed.",
        )
    )

    # Initialize arguments
    gripper_node = LaunchConfiguration(gripper_node_param_name)
    robot_sn = LaunchConfiguration(robot_sn_param_name)
    gripper_name = LaunchConfiguration(gripper_name_param_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_param_name)
    use_lite_rdk = LaunchConfiguration(use_lite_rdk_param_name)
    gripper_joint_names = LaunchConfiguration(gripper_joint_names_param_name)
    joint_group = LaunchConfiguration(joint_group_param_name)
    rdk_install_prefix = LaunchConfiguration(rdk_install_prefix_param_name)

    set_rdk_ld_library_path = SetEnvironmentVariable(
        name="LD_LIBRARY_PATH",
        value=[
            PathJoinSubstitution([rdk_install_prefix, "lib"]),
            ":",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    )

    gripper_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_gripper"), "config", "flexiv_gripper_node.yaml"]
    )

    # Flexiv gripper node
    flexiv_gripper_node = Node(
        package="flexiv_gripper",
        executable="flexiv_gripper_node",
        name=gripper_node,
        parameters=[
            {
                "robot_sn": robot_sn,
                "gripper_name": gripper_name,
                "gripper_joint_names": gripper_joint_names,
                "use_lite_rdk": use_lite_rdk,
                "joint_group": joint_group,
            },
            gripper_config_file,
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    nodes = [flexiv_gripper_node]

    return LaunchDescription(declared_arguments + [set_rdk_ld_library_path] + nodes)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_sn_param_name = "robot_sn"
    robot_type_param_name = "robot_type"
    target_filename_param_name = "target_filename"
    template_filename_param_name = "template_filename"

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
            robot_type_param_name,
            default_value="",
            description="Type of the Flexiv robot, which selects the kinematics file to update. Defaults to the model name reported by the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            target_filename_param_name,
            default_value="",
            description="Write the synced parameters here instead of updating flexiv_description's default_kinematics.yaml. Use this to keep several robots of the same type side by side.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            template_filename_param_name,
            default_value="",
            description="Template kinematics YAML file to sync. Defaults to flexiv_description/config/[robot_type]/default_kinematics.yaml",
        )
    )

    # Initialize arguments
    robot_sn = LaunchConfiguration(robot_sn_param_name)
    robot_type = LaunchConfiguration(robot_type_param_name)
    target_filename = LaunchConfiguration(target_filename_param_name)
    template_filename = LaunchConfiguration(template_filename_param_name)

    calibration_correction_node = Node(
        package="flexiv_calibration",
        executable="calibration_correction",
        name="calibration_correction",
        parameters=[
            {
                "robot_sn": robot_sn,
                "robot_type": robot_type,
                "target_filename": target_filename,
                "template_filename": template_filename,
            }
        ],
        output="screen",
    )

    # This is a one-shot tool, so stop the launch service once it is done.
    shutdown_after_node_exits = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=calibration_correction_node,
            on_exit=[EmitEvent(event=Shutdown(reason="calibration finished"))],
        )
    )

    nodes = [calibration_correction_node, shutdown_after_node_exits]

    return LaunchDescription(declared_arguments + nodes)

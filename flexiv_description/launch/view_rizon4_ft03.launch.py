from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "rizon4_ft03.urdf"]
    )

    def launch_setup(context, *args, **kwargs):
        import os

        urdf_path = urdf_file.perform(context)
        with open(urdf_path, "r") as inf:
            robot_description_content = inf.read()

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        )

        joint_state_publisher_node = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        )

        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("flexiv_description"), "rviz", "view_rizon.rviz"]
                ),
            ],
        )

        return [
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="False",
                description="Flag to enable joint_state_publisher_gui",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

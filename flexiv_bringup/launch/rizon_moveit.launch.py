import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare command-line arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rizon_type",
            description="Type of the Flexiv Rizon robot.",
            default_value="rizon4",
            choices=["rizon4", "rizon4s", "rizon10"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of the robot server (remote).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "local_ip",
            description="IP address of the workstation PC (local).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="start RViz automatically with the launch file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path to the sqlite database used by the warehouse_ros package.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_servo",
            default_value="false",
            description="Start the MoveIt servo node?",
        )
    )

    rizon_type = LaunchConfiguration("rizon_type")
    robot_ip = LaunchConfiguration("robot_ip")
    local_ip = LaunchConfiguration("local_ip")
    start_rviz = LaunchConfiguration("start_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    start_servo = LaunchConfiguration("start_servo")

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "rizon.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            flexiv_urdf_xacro,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "local_ip:=",
            local_ip,
            " ",
            "name:=",
            "rizon",
            " ",
            "rizon_type:=",
            rizon_type,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt configuration
    flexiv_srdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_moveit_config"), "srdf", "rizon.srdf.xacro"]
    )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            flexiv_srdf_xacro,
            " ",
            "name:=",
            "rizon",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("flexiv_moveit_config"), "config", "kinematics.yaml"]
    )

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("flexiv_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_simple_controllers_yaml = load_yaml(
        "flexiv_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "flexiv_moveit_config", "config/joint_limits.yaml"
        )
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            warehouse_ros_config,
        ],
    )

    # RViz with MoveIt configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_moveit_config"), "rviz", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
            warehouse_ros_config,
        ],
        condition=IfCondition(start_rviz),
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Robot controllers
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("flexiv_bringup"), "config", "rizon_controllers.yaml"]
    )

    # Run controller manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Run robot controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rizon_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml(
        "flexiv_moveit_config", "config/rizon_moveit_servo_config.yaml"
    )
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(start_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )

    nodes = [
        move_group_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        servo_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

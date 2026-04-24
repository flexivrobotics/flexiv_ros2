import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def load_yaml(package_name, file_path, replacements=None):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            yaml_content = file.read()

        if replacements:
            for placeholder, replacement in replacements.items():
                yaml_content = yaml_content.replace(placeholder, replacement)

        return yaml.safe_load(yaml_content)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context):
    # Initialize Arguments
    rizon_type = LaunchConfiguration("rizon_type")
    robot_sn = LaunchConfiguration("robot_sn")
    robot_sn_str = robot_sn.perform(context)
    rdk_control_mode = LaunchConfiguration("rdk_control_mode")
    start_rviz = LaunchConfiguration("start_rviz")
    load_gripper = LaunchConfiguration("load_gripper")
    gripper_name = LaunchConfiguration("gripper_name")
    load_mounted_ft_sensor = LaunchConfiguration("load_mounted_ft_sensor")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    start_servo = LaunchConfiguration("start_servo")
    gripper_ready_gate_condition = PythonExpression(
        [
            "'",
            load_gripper,
            "'.lower() in ['true', '1'] and '",
            use_fake_hardware,
            "'.lower() not in ['true', '1']",
        ]
    )

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "rizon.urdf.xacro"]
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_urdf_xacro,
                " ",
                "robot_sn:=",
                robot_sn,
                " ",
                "rizon_type:=",
                rizon_type,
                " ",
                "ros2_control:=true ",
                "rdk_control_mode:=",
                rdk_control_mode,
                " ",
                "load_gripper:=",
                load_gripper,
                " ",
                "gripper_name:=",
                gripper_name,
                " ",
                "load_mounted_ft_sensor:=",
                load_mounted_ft_sensor,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
                " ",
                "fake_sensor_commands:=",
                fake_sensor_commands,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt configuration
    flexiv_srdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_moveit_config"), "srdf", "rizon.srdf.xacro"]
    )

    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_srdf_xacro,
                " ",
                "robot_sn:=",
                robot_sn,
                " ",
                "rizon_type:=",
                rizon_type,
                " ",
                "load_gripper:=",
                load_gripper,
                " ",
                "load_mounted_ft_sensor:=",
                load_mounted_ft_sensor,
            ]
        ),
        value_type=str,
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    replacements = {"$(var robot_sn)": robot_sn_str}

    robot_description_kinematics_yaml = load_yaml(
        "flexiv_moveit_config", "config/kinematics.yaml", replacements
    )
    robot_description_kinematics = {
        "robot_description_kinematics": robot_description_kinematics_yaml
    }

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
    ompl_planning_yaml = load_yaml(
        "flexiv_moveit_config", "config/ompl_planning.yaml", replacements
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_simple_controllers_yaml = load_yaml(
        "flexiv_moveit_config", "config/moveit_controllers.yaml", replacements
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
            "flexiv_moveit_config", "config/joint_limits.yaml", replacements
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
            publish_robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
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
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
            {"robot_sn": robot_sn},
            {"rdk_control_mode": rdk_control_mode},
        ],
        remappings=[("joint_states", "flexiv_rizon_arm/joint_states")],
        output="both",
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "flexiv_rizon_arm/joint_states",
                    "flexiv_gripper_node/gripper_joint_states",
                ],
                "rate": 30,
            }
        ],
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

    # Run Flexiv robot states broadcaster
    flexiv_robot_states_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster"],
        parameters=[{"robot_sn": robot_sn}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Include gripper launch file
    load_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_gripper"),
                    "launch",
                    "flexiv_gripper.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_sn": robot_sn,
            "gripper_name": gripper_name,
            "use_fake_hardware": use_fake_hardware,
            "use_lite_rdk": "true",
        }.items(),
        condition=IfCondition(load_gripper),
    )

    gripper_ready_waiter = Node(
        package="flexiv_gripper",
        executable="wait_for_gripper_ready",
        name="wait_for_gripper_ready",
        parameters=[{"ready_topic": "/flexiv_gripper_node/ready"}],
        output="screen",
        condition=IfCondition(gripper_ready_gate_condition),
    )

    def launch_robot_controller_after_gripper_ready(event, context):
        if event.returncode == 0:
            return [robot_controller_spawner]
        return [
            EmitEvent(event=Shutdown(reason="flexiv_gripper_node did not report ready"))
        ]

    # Servo node for realtime control
    servo_yaml = load_yaml(
        "flexiv_moveit_config",
        "config/rizon_moveit_servo_config.yaml",
        replacements,
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

    # Run gpio controller
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"robot_sn": robot_sn}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Delay start of robot_controller after `joint_state_broadcaster` when gripper is not loaded
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            ),
            condition=UnlessCondition(gripper_ready_gate_condition),
        )
    )

    delay_gripper_launch_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[load_gripper_launch],
        ),
        condition=IfCondition(gripper_ready_gate_condition),
    )

    delay_robot_controller_spawner_after_gripper_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_ready_waiter,
            on_exit=launch_robot_controller_after_gripper_ready,
        ),
        condition=IfCondition(gripper_ready_gate_condition),
    )

    # Delay move_group start after `robot_controller_spawner`
    delay_move_group_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # Delay rviz start after `robot_controller_spawner`
    delay_rviz_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        ros2_control_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        gripper_ready_waiter,
        joint_state_broadcaster_spawner,
        flexiv_robot_states_broadcaster_spawner,
        gpio_controller_spawner,
        servo_node,
        delay_gripper_launch_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_gripper_ready,
        delay_move_group_after_robot_controller_spawner,
        delay_rviz_after_robot_controller_spawner,
    ]

    return nodes


def generate_launch_description():
    # Declare command-line arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rizon_type",
            description="Type of the Flexiv Rizon robot.",
            default_value="Rizon4",
            choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_sn",
            description="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rdk_control_mode",
            default_value="joint_position",
            description="RDK control mode for the ROS 2 control joint position and velocity interfaces. Options: joint_position, joint_impedance",
            choices=["joint_position", "joint_impedance"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz automatically with the launch file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_gripper",
            default_value="false",
            description="Flag to load the Flexiv Grav gripper as the end-effector of the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_name",
            default_value="Flexiv-GN01",
            description="Full name of the gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_mounted_ft_sensor",
            default_value="false",
            description="Flag to load the mounted force torque sensor. Only available for Rizon4, Rizon4R and Rizon10.",
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

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

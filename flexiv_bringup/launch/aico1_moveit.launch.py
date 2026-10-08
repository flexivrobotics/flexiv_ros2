from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
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
    path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(path) as file:
        content = file.read()
    for placeholder, value in (replacements or {}).items():
        content = content.replace(placeholder, value)
    return yaml.safe_load(content)


def launch_setup(context):
    arm_type = LaunchConfiguration("arm_type")
    robot_sn = LaunchConfiguration("robot_sn")
    rdk_control_mode = LaunchConfiguration("rdk_control_mode")
    rdk_realtime_mode = LaunchConfiguration("rdk_realtime_mode")
    start_rviz = LaunchConfiguration("start_rviz")
    load_gripper = LaunchConfiguration("load_gripper")
    gripper_name = LaunchConfiguration("gripper_name")
    load_mounted_ft_sensor = LaunchConfiguration("load_mounted_ft_sensor")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_type = LaunchConfiguration("robot_type")
    external_axis_prefix = LaunchConfiguration("external_axis_prefix")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file").perform(
        context
    )
    # Passing an empty path through would reach xacro.load_yaml('') and abort.
    kinematics_xacro_arg = (
        f" kinematics_parameters_file:={kinematics_params_file}"
        if kinematics_params_file
        else ""
    )
    gripper_ready_gate_condition = PythonExpression(
        [
            "'",
            load_gripper,
            "'.lower() in ['true', '1'] and '",
            use_fake_hardware,
            "'.lower() not in ['true', '1']",
        ]
    )

    robot_sn_str = robot_sn.perform(context)
    external_axis_prefix_str = external_axis_prefix.perform(context)
    robot_type_str = robot_type.perform(context)

    # Construct prefix
    prefix_str = robot_sn_str + "_" if robot_sn_str else ""
    set_prefix = SetLaunchConfiguration(name="prefix", value=prefix_str)

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_hardware"), "urdf", "flexiv.urdf.xacro"]
    )

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
                "arm_type:=",
                arm_type,
                " ",
                "ros2_control:=true ",
                "rdk_control_mode:=",
                rdk_control_mode,
                " ",
                "rdk_realtime_mode:=",
                rdk_realtime_mode,
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
                " ",
                "robot_type:=",
                robot_type,
                " ",
                "external_axis_prefix:=",
                external_axis_prefix,
                kinematics_xacro_arg,
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # MoveIt configuration
    flexiv_srdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_moveit_config"), "srdf", "aico1.srdf.xacro"]
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
                "load_gripper:=",
                load_gripper,
                " ",
                "load_mounted_ft_sensor:=",
                load_mounted_ft_sensor,
                " ",
                "robot_type:=",
                robot_type,
                " ",
                "external_axis_prefix:=",
                external_axis_prefix,
            ]
        ),
        value_type=str,
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    # The external axis joint names carry the robot type lowercased with dashes
    # as underscores, e.g. AICO1-4-V2 -> aico1_4_v2.
    replacements = {
        "$(var prefix)": prefix_str,
        "$(var external_axis_prefix)": external_axis_prefix_str,
        "$(var external_axis_type)": robot_type_str.lower().replace("-", "_"),
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "flexiv_moveit_config", "config/kinematics.yaml", replacements
        )
    }

    # Without planner_configs in ompl_planning.yaml, add MoveIt's default
    # planners the way MoveItConfigsBuilder does.
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            **load_yaml("flexiv_moveit_config", "config/ompl_planning.yaml"),
            **load_yaml("moveit_configs_utils", "default_configs/ompl_defaults.yaml"),
        },
    }

    controllers_file = "config/aico/aico1_4_v1_moveit_controllers.yaml"
    if robot_type_str == "AICO1-4-V2":
        controllers_file = "config/aico/aico1_4_v2_moveit_controllers.yaml"

    moveit_controllers = load_yaml(
        "flexiv_moveit_config", controllers_file, replacements
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # The arm file is the base so its default scaling factors are kept.
    joint_limits = load_yaml(
        "flexiv_moveit_config", "config/joint_limits.yaml", replacements
    )
    joint_limits["joint_limits"].update(
        load_yaml(
            "flexiv_moveit_config", "config/aico/aico_joint_limits.yaml", replacements
        )["joint_limits"]
    )
    joint_limits_yaml = {"robot_description_planning": joint_limits}

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": LaunchConfiguration("warehouse_sqlite_path"),
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
            planning_pipelines,
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
            planning_pipelines,
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
    ros2_controllers_file = "aico1_4_v1_controllers.yaml"
    if robot_type_str == "AICO1-4-V2":
        ros2_controllers_file = "aico1_4_v2_controllers.yaml"
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("flexiv_bringup"), "config", ros2_controllers_file]
    )

    # Run controller manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
        ],
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
    rizon_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rizon_arm_controller"],
    )

    # Run joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-ros-args",
            "-r joint_states:=flexiv_rizon_arm/joint_states",
        ],
    )

    # Run Flexiv robot states broadcaster
    flexiv_robot_states_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster"],
        condition=UnlessCondition(use_fake_hardware),
    )

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
            "gripper_joint_names": [
                "[",
                LaunchConfiguration("prefix"),
                "finger_width_joint]",
            ],
            "robot_sn": robot_sn,
            "gripper_name": gripper_name,
            "use_lite_rdk": "true",
        }.items(),
    )

    gripper_ready_waiter = Node(
        package="flexiv_gripper",
        executable="wait_for_gripper_ready",
        name="wait_for_gripper_ready",
        parameters=[{"ready_topic": "/flexiv_gripper_node/ready"}],
        output="screen",
        condition=IfCondition(gripper_ready_gate_condition),
    )

    def launch_controller_after_gripper_ready(event, context):
        if event.returncode == 0:
            return [rizon_arm_controller_spawner]
        return [
            EmitEvent(event=Shutdown(reason="flexiv_gripper_node did not report ready"))
        ]

    # Run gpio controller
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gpio_controller",
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Delay start of controllers after `joint_state_broadcaster`
    delay_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rizon_arm_controller_spawner],
        ),
        condition=UnlessCondition(gripper_ready_gate_condition),
    )

    # Start gripper only after ros2_control has activated and the joint state broadcaster is up.
    delay_gripper_launch_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[load_gripper_launch, gripper_ready_waiter],
        ),
        condition=IfCondition(gripper_ready_gate_condition),
    )

    delay_controller_after_gripper_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_ready_waiter,
            on_exit=launch_controller_after_gripper_ready,
        ),
        condition=IfCondition(gripper_ready_gate_condition),
    )

    # Start move_group and RViz once the arm controller is up
    delay_moveit_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rizon_arm_controller_spawner,
            on_exit=[move_group_node, rviz_node],
        )
    )

    nodes = [
        set_prefix,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        flexiv_robot_states_broadcaster_spawner,
        gpio_controller_spawner,
        delay_gripper_launch_after_joint_state_broadcaster_spawner,
        delay_controller_after_jsb,
        delay_controller_after_gripper_ready,
        delay_moveit_after_controller,
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_type",
            description="Type of the arm carried by the external axis.",
            default_value="Rizon4",
            choices=["Rizon4", "Rizon4s"],
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
            "rdk_realtime_mode",
            default_value="false",
            description="Use the real-time RDK modes for the joint position, velocity and Cartesian interfaces, streamed at the controller manager rate. Requires a low-latency host. Options: true, false",
            choices=["true", "false"],
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
            "robot_type",
            default_value="AICO1-4-V1",
            description="Type of the AICO1 platform.",
            choices=["AICO1-4-V1", "AICO1-4-V2"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "external_axis_prefix",
            default_value="",
            description="Prefix for the external axis links and joints.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path to the warehouse database",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value="",
            description="Kinematics YAML file holding this robot's measured parameters, as generated by flexiv_calibration. Defaults to the nominal values shipped in flexiv_description.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

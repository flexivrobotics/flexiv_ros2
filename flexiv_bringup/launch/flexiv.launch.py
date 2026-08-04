import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
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
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def launch_setup(context):
    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    robot_sn = LaunchConfiguration("robot_sn")
    robot_type_str = robot_type.perform(context)
    dual_arm_robot_types = ["Enlight-LL", "MICO-Core", "MICO-Plus", "MICO-Ultra"]
    pan_tilt_robot_types = ["MICO-Plus", "MICO-Ultra"]
    is_dual = robot_type_str in dual_arm_robot_types
    rdk_control_mode = LaunchConfiguration("rdk_control_mode")
    start_rviz = LaunchConfiguration("start_rviz")
    load_gripper = LaunchConfiguration("load_gripper")
    gripper_name = LaunchConfiguration("gripper_name")
    gripper_name_left = LaunchConfiguration("gripper_name_left")
    gripper_name_right = LaunchConfiguration("gripper_name_right")
    robot_controller = LaunchConfiguration("robot_controller")
    rdk_install_prefix = LaunchConfiguration("rdk_install_prefix")
    load_mounted_ft_sensor = LaunchConfiguration("load_mounted_ft_sensor")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
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
                "robot_type:=",
                robot_type,
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

    # RViZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "rviz", "view_flexiv.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    # Robot controllers — select by topology (single 7-joint / dual 14-joint / MICO 16-joint).
    if robot_type_str in pan_tilt_robot_types:
        controllers_file = "flexiv_mico_controllers.yaml"
    elif is_dual:
        controllers_file = "flexiv_dual_controllers.yaml"
    else:
        controllers_file = "flexiv_controllers.yaml"
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("flexiv_bringup"), "config", controllers_file]
    )

    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
            {"robot_sn": robot_sn},
            {"rdk_control_mode": rdk_control_mode},
        ],
        remappings=[("joint_states", "flexiv_arm/joint_states")],
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
                    "flexiv_arm/joint_states",
                    "flexiv_gripper_node/gripper_joint_states",
                    "left_flexiv_gripper_node/gripper_joint_states",
                    "right_flexiv_gripper_node/gripper_joint_states",
                ],
                "rate": 30,
            }
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Run robot controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
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

    # Run Flexiv robot states broadcaster(s). Single-arm robots publish one; dual-arm robots
    # publish one per arm (left_<sn> / right_<sn>), matching the selected controllers yaml.
    robot_states_broadcaster_spawners = []
    if is_dual:
        robot_states_broadcaster_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_flexiv_robot_states_broadcaster"],
                condition=UnlessCondition(use_fake_hardware),
            )
        )
        robot_states_broadcaster_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_flexiv_robot_states_broadcaster"],
                condition=UnlessCondition(use_fake_hardware),
            )
        )
    else:
        robot_states_broadcaster_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["flexiv_robot_states_broadcaster"],
                parameters=[{"robot_sn": robot_sn}],
                condition=UnlessCondition(use_fake_hardware),
            )
        )

    # ---- Gripper launch + readiness gating ----
    # Grippers run only on real hardware (gripper_ready_gate_condition). Single-arm robots run one
    # gripper node; dual-arm robots run one per arm group (ARM_1 = left, ARM_2 = right), each a lite
    # RDK instance sharing the driver's connection.
    def gripper_launch(node_name, name, joint_group=None, gripper_joint_names=None):
        args = {
            "gripper_node_name": node_name,
            "robot_sn": robot_sn,
            "gripper_name": name,
            "use_fake_hardware": use_fake_hardware,
            "use_lite_rdk": "true",
            "rdk_install_prefix": rdk_install_prefix,
        }
        if joint_group is not None:
            args["joint_group"] = joint_group
        if gripper_joint_names is not None:
            args["gripper_joint_names"] = gripper_joint_names
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("flexiv_gripper"),
                        "launch",
                        "flexiv_gripper.launch.py",
                    ]
                )
            ),
            launch_arguments=args.items(),
            condition=IfCondition(gripper_ready_gate_condition),
        )

    def gripper_ready_waiter_node(node_name, topic):
        return Node(
            package="flexiv_gripper",
            executable="wait_for_gripper_ready",
            name=node_name,
            parameters=[{"ready_topic": topic}],
            output="screen",
            condition=IfCondition(gripper_ready_gate_condition),
        )

    def launch_robot_controller_after_gripper_ready(event, context):
        if event.returncode == 0:
            return [robot_controller_spawner]
        return [EmitEvent(event=Shutdown(reason="flexiv gripper did not report ready"))]

    # Nodes added directly, and the event handlers that sequence the gripper(s) -> controller.
    gripper_nodes = []
    gripper_event_handlers = []
    if is_dual:
        load_gripper_launch_left = gripper_launch(
            "left_flexiv_gripper_node",
            gripper_name_left,
            "ARM_1",
            gripper_joint_names=["[left_", robot_sn, "_finger_width_joint]"],
        )
        load_gripper_launch_right = gripper_launch(
            "right_flexiv_gripper_node",
            gripper_name_right,
            "ARM_2",
            gripper_joint_names=["[right_", robot_sn, "_finger_width_joint]"],
        )
        left_gripper_ready_waiter = gripper_ready_waiter_node(
            "wait_for_left_gripper_ready", "/left_flexiv_gripper_node/ready"
        )
        right_gripper_ready_waiter = gripper_ready_waiter_node(
            "wait_for_right_gripper_ready", "/right_flexiv_gripper_node/ready"
        )

        def launch_right_waiter_after_left_ready(event, context):
            if event.returncode == 0:
                return [right_gripper_ready_waiter]
            return [
                EmitEvent(
                    event=Shutdown(
                        reason="left_flexiv_gripper_node did not report ready"
                    )
                )
            ]

        gripper_nodes = [left_gripper_ready_waiter]
        gripper_event_handlers = [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[load_gripper_launch_left, load_gripper_launch_right],
                ),
                condition=IfCondition(gripper_ready_gate_condition),
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=left_gripper_ready_waiter,
                    on_exit=launch_right_waiter_after_left_ready,
                ),
                condition=IfCondition(gripper_ready_gate_condition),
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=right_gripper_ready_waiter,
                    on_exit=launch_robot_controller_after_gripper_ready,
                ),
                condition=IfCondition(gripper_ready_gate_condition),
            ),
        ]
    else:
        load_gripper_launch = gripper_launch(
            "flexiv_gripper_node",
            gripper_name,
            gripper_joint_names=["[", robot_sn, "_finger_width_joint]"],
        )
        gripper_ready_waiter = gripper_ready_waiter_node(
            "wait_for_gripper_ready", "/flexiv_gripper_node/ready"
        )
        gripper_nodes = [gripper_ready_waiter]
        gripper_event_handlers = [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[load_gripper_launch],
                ),
                condition=IfCondition(gripper_ready_gate_condition),
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gripper_ready_waiter,
                    on_exit=launch_robot_controller_after_gripper_ready,
                ),
                condition=IfCondition(gripper_ready_gate_condition),
            ),
        ]

    # Run gpio controller
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"robot_sn": robot_sn}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # When no gripper is loaded, start the robot controller right after joint_state_broadcaster.
    # (With a gripper, the gripper_event_handlers chain starts the controller once grippers ready.)
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            ),
            condition=UnlessCondition(gripper_ready_gate_condition),
        )
    )

    # Delay rviz start after `robot_controller_spawner`
    delay_rviz_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # MICO-Ultra mobile base alert
    mico_ultra_alert = LogInfo(
        msg=(
            "MICO-Ultra mobile base is NOT supported in ros2_control (v2.1). "
            "The arms and pan-tilt torso are controllable; the mobile base is not "
            "driven by this stack."
        ),
        condition=IfCondition(PythonExpression(["'", robot_type, "' == 'MICO-Ultra'"])),
    )

    nodes = [
        mico_ultra_alert,
        ros2_control_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        *robot_states_broadcaster_spawners,
        gpio_controller_spawner,
        *gripper_nodes,
        *gripper_event_handlers,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_robot_controller_spawner,
    ]

    return nodes


def generate_launch_description():
    robot_types = [
        "Enlight-L",
        "Enlight-LL",
        "MICO-Core",
        "MICO-Plus",
        "MICO-Ultra",
    ]

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type of the Flexiv robot. Single-arm: Enlight-L. Dual-arm: Enlight-LL, MICO-Core, MICO-Plus, MICO-Ultra.",
            default_value="Enlight-L",
            choices=robot_types,
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_sn",
            description="Serial number of the robot to connect to. Remove any space, for example: Enlight-L-123456",
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
            description="Full name of the gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device. For single-arm robots and as the default for both arms of a dual-arm robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_name_left",
            default_value=LaunchConfiguration("gripper_name"),
            description="Gripper device name for the LEFT arm (ARM_1) of a dual-arm robot. Defaults to 'gripper_name'.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_name_right",
            default_value=LaunchConfiguration("gripper_name"),
            description="Gripper device name for the RIGHT arm (ARM_2) of a dual-arm robot. Defaults to 'gripper_name'.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_mounted_ft_sensor",
            default_value="false",
            description="Flag to load the mounted force torque sensor.",
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
            "robot_controller",
            default_value="flexiv_arm_controller",
            description="Robot controller to start. Available: flexiv_arm_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rdk_install_prefix",
            default_value=os.path.expanduser("~/rdk_install"),
            description="Prefix where flexiv_rdk and its shared-library dependencies are installed.",
        )
    )

    set_rdk_ld_library_path = SetEnvironmentVariable(
        name="LD_LIBRARY_PATH",
        value=[
            PathJoinSubstitution([LaunchConfiguration("rdk_install_prefix"), "lib"]),
            PythonExpression(
                [
                    "':' if '",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                    "' else ''",
                ]
            ),
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [set_rdk_ld_library_path, OpaqueFunction(function=launch_setup)]
    )

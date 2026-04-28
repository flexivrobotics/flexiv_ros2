from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
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


def generate_launch_description():
    rizon_type_param_name = "rizon_type"
    robot_sn_left_param_name = "robot_sn_left"
    robot_sn_right_param_name = "robot_sn_right"
    rdk_control_mode_param_name = "rdk_control_mode"
    start_rviz_param_name = "start_rviz"
    use_fake_hardware_param_name = "use_fake_hardware"
    fake_sensor_commands_param_name = "fake_sensor_commands"
    load_gripper_left_param_name = "load_gripper_left"
    gripper_name_left_param_name = "gripper_name_left"
    load_gripper_right_param_name = "load_gripper_right"
    gripper_name_right_param_name = "gripper_name_right"
    load_mounted_ft_sensor_left_param_name = "load_mounted_ft_sensor_left"
    load_mounted_ft_sensor_right_param_name = "load_mounted_ft_sensor_right"
    external_axis_type_param_name = "external_axis_type"
    external_axis_prefix_param_name = "external_axis_prefix"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            rizon_type_param_name,
            description="Type of the Flexiv Rizon robot.",
            default_value="Rizon4",
            choices=["Rizon4", "Rizon10"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_sn_left_param_name,
            description="Serial number of the left robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_sn_right_param_name,
            description="Serial number of the right robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            rdk_control_mode_param_name,
            default_value="joint_position",
            description="RDK control mode for the ROS 2 control joint position and velocity interfaces.",
            choices=["joint_position", "joint_impedance"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            start_rviz_param_name,
            default_value="true",
            description="Start RViz automatically with the launch file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            use_fake_hardware_param_name,
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            fake_sensor_commands_param_name,
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            load_gripper_left_param_name,
            default_value="false",
            description="Flag to load the Flexiv Grav gripper as the end-effector of the left robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_name_left_param_name,
            default_value="Flexiv-GN01",
            description="Full name of the left gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            load_gripper_right_param_name,
            default_value="false",
            description="Flag to load the Flexiv Grav gripper as the end-effector of the right robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_name_right_param_name,
            default_value="Flexiv-GN01",
            description="Full name of the right gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            load_mounted_ft_sensor_left_param_name,
            default_value="false",
            description="Flag to load the mounted force torque sensor for the left robot. Only available for Rizon4, Rizon4R and Rizon10.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            load_mounted_ft_sensor_right_param_name,
            default_value="false",
            description="Flag to load the mounted force torque sensor for the right robot. Only available for Rizon4, Rizon4R and Rizon10.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            external_axis_type_param_name,
            default_value="AICO2-4-V1",
            description="Type of the AICO2 platform.",
            choices=["AICO2-4-V1", "AICO2-4-V2", "AICO2-10-V1"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            external_axis_prefix_param_name,
            default_value="",
            description="Prefix for the external axis links and joints.",
        )
    )

    # Initialize Arguments
    rizon_type = LaunchConfiguration(rizon_type_param_name)
    robot_sn_left = LaunchConfiguration(robot_sn_left_param_name)
    robot_sn_right = LaunchConfiguration(robot_sn_right_param_name)
    rdk_control_mode = LaunchConfiguration(rdk_control_mode_param_name)
    start_rviz = LaunchConfiguration(start_rviz_param_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_param_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_param_name)
    load_gripper_left = LaunchConfiguration(load_gripper_left_param_name)
    gripper_name_left = LaunchConfiguration(gripper_name_left_param_name)
    load_gripper_right = LaunchConfiguration(load_gripper_right_param_name)
    gripper_name_right = LaunchConfiguration(gripper_name_right_param_name)
    load_mounted_ft_sensor_left = LaunchConfiguration(
        load_mounted_ft_sensor_left_param_name
    )
    load_mounted_ft_sensor_right = LaunchConfiguration(
        load_mounted_ft_sensor_right_param_name
    )
    external_axis_type = LaunchConfiguration(external_axis_type_param_name)
    external_axis_prefix = LaunchConfiguration(external_axis_prefix_param_name)
    left_gripper_ready_gate_condition = PythonExpression(
        [
            "'",
            load_gripper_left,
            "'.lower() in ['true', '1'] and '",
            use_fake_hardware,
            "'.lower() not in ['true', '1']",
        ]
    )
    right_gripper_ready_gate_condition = PythonExpression(
        [
            "'",
            load_gripper_right,
            "'.lower() in ['true', '1'] and '",
            use_fake_hardware,
            "'.lower() not in ['true', '1']",
        ]
    )

    # Construct prefixes
    from launch.actions import SetLaunchConfiguration

    set_prefix_left = SetLaunchConfiguration(
        name="prefix_left",
        value=PythonExpression(["'left_' + '", robot_sn_left, "' + '_'"]),
    )
    set_prefix_right = SetLaunchConfiguration(
        name="prefix_right",
        value=PythonExpression(["'right_' + '", robot_sn_right, "' + '_'"]),
    )

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "aico2.urdf.xacro"]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_urdf_xacro,
                " ",
                "rizon_type:=",
                rizon_type,
                " ",
                "robot_sn_left:=",
                robot_sn_left,
                " ",
                "robot_sn_right:=",
                robot_sn_right,
                " ",
                "load_gripper_left:=",
                load_gripper_left,
                " ",
                "gripper_name_left:=",
                gripper_name_left,
                " ",
                "load_gripper_right:=",
                load_gripper_right,
                " ",
                "gripper_name_right:=",
                gripper_name_right,
                " ",
                "load_mounted_ft_sensor_left:=",
                load_mounted_ft_sensor_left,
                " ",
                "load_mounted_ft_sensor_right:=",
                load_mounted_ft_sensor_right,
                " ",
                "ros2_control:=true ",
                "rdk_control_mode:=",
                rdk_control_mode,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
                " ",
                "fake_sensor_commands:=",
                fake_sensor_commands,
                " ",
                "external_axis_type:=",
                PythonExpression(
                    ["'", external_axis_type, "'.lower().replace('-', '_')"]
                ),
                " ",
                "external_axis_prefix:=",
                external_axis_prefix,
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # RViZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "rviz", "view_rizon.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    # Robot controllers
    controller_file_name = PythonExpression(
        [
            "'aico2_10_v1_controllers.yaml' if '",
            external_axis_type,
            "' == 'AICO2-10-V1' else ('aico2_4_v2_controllers.yaml' if '",
            external_axis_type,
            "' == 'AICO2-4-V2' else 'aico2_4_v1_controllers.yaml')",
        ]
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("flexiv_bringup"),
            "config",
            controller_file_name,
        ]
    )

    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
            {"robot_sn_left": robot_sn_left},
            {"robot_sn_right": robot_sn_right},
            {"prefix_left": LaunchConfiguration("prefix_left")},
            {"prefix_right": LaunchConfiguration("prefix_right")},
            {"rdk_control_mode": rdk_control_mode},
            {"external_axis_prefix": external_axis_prefix},
        ],
        remappings=[("joint_states", "flexiv_dual_arm/joint_states")],
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
                    "flexiv_dual_arm/joint_states",
                    "left_gripper_node/gripper_joint_states",
                    "right_gripper_node/gripper_joint_states",
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

    # Run left arm controller
    left_rizon_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_rizon_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run right arm controller
    right_rizon_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_rizon_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run Flexiv robot states broadcaster left
    flexiv_robot_states_broadcaster_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster_left"],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Run Flexiv robot states broadcaster right
    flexiv_robot_states_broadcaster_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster_right"],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Include gripper launch files
    load_gripper_left_launch = IncludeLaunchDescription(
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
            "gripper_node_name": "left_gripper_node",
            "robot_sn": robot_sn_left,
            "gripper_name": gripper_name_left,
            "use_fake_hardware": use_fake_hardware,
            "use_lite_rdk": "true",
        }.items(),
        condition=IfCondition(load_gripper_left),
    )
    load_gripper_right_launch = IncludeLaunchDescription(
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
            "gripper_node_name": "right_gripper_node",
            "robot_sn": robot_sn_right,
            "gripper_name": gripper_name_right,
            "use_fake_hardware": use_fake_hardware,
            "use_lite_rdk": "true",
        }.items(),
        condition=IfCondition(load_gripper_right),
    )

    left_gripper_ready_waiter = Node(
        package="flexiv_gripper",
        executable="wait_for_gripper_ready",
        name="wait_for_left_gripper_ready",
        parameters=[{"ready_topic": "/left_gripper_node/ready"}],
        output="screen",
        condition=IfCondition(left_gripper_ready_gate_condition),
    )

    right_gripper_ready_waiter = Node(
        package="flexiv_gripper",
        executable="wait_for_gripper_ready",
        name="wait_for_right_gripper_ready",
        parameters=[{"ready_topic": "/right_gripper_node/ready"}],
        output="screen",
        condition=IfCondition(right_gripper_ready_gate_condition),
    )

    def launch_controller_after_gripper_ready(controller_spawner, gripper_node_name):
        def handle_gripper_ready(event, context):
            if event.returncode == 0:
                return [controller_spawner]
            return [
                EmitEvent(
                    event=Shutdown(reason=f"{gripper_node_name} did not report ready")
                )
            ]

        return handle_gripper_ready

    # Run gpio controllers
    gpio_controller_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gpio_controller_left",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_fake_hardware),
    )
    gpio_controller_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gpio_controller_right",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Delay start of controllers after `joint_state_broadcaster`
    delay_left_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_rizon_arm_controller_spawner],
        ),
        condition=UnlessCondition(left_gripper_ready_gate_condition),
    )

    delay_left_gripper_launch_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[load_gripper_left_launch, left_gripper_ready_waiter],
            ),
            condition=IfCondition(left_gripper_ready_gate_condition),
        )
    )

    delay_right_gripper_launch_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[load_gripper_right_launch],
            ),
            condition=IfCondition(right_gripper_ready_gate_condition),
        )
    )

    delay_left_controller_after_gripper_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_gripper_ready_waiter,
            on_exit=launch_controller_after_gripper_ready(
                left_rizon_arm_controller_spawner, "left_gripper_node"
            ),
        ),
        condition=IfCondition(left_gripper_ready_gate_condition),
    )

    # Delay right controller start after left controller
    delay_right_controller_after_left_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_rizon_arm_controller_spawner,
            on_exit=[right_rizon_arm_controller_spawner],
        ),
        condition=UnlessCondition(right_gripper_ready_gate_condition),
    )

    delay_right_gripper_ready_waiter_after_left_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_rizon_arm_controller_spawner,
            on_exit=[right_gripper_ready_waiter],
        ),
        condition=IfCondition(right_gripper_ready_gate_condition),
    )

    delay_right_controller_after_gripper_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_gripper_ready_waiter,
            on_exit=launch_controller_after_gripper_ready(
                right_rizon_arm_controller_spawner, "right_gripper_node"
            ),
        ),
        condition=IfCondition(right_gripper_ready_gate_condition),
    )

    # Delay rviz start after right controller
    delay_rviz_after_right_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_rizon_arm_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        set_prefix_left,
        set_prefix_right,
        ros2_control_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        flexiv_robot_states_broadcaster_left_spawner,
        flexiv_robot_states_broadcaster_right_spawner,
        gpio_controller_left_spawner,
        gpio_controller_right_spawner,
        delay_left_gripper_launch_after_joint_state_broadcaster_spawner,
        delay_right_gripper_launch_after_joint_state_broadcaster_spawner,
        delay_left_controller_after_jsb,
        delay_left_controller_after_gripper_ready,
        delay_right_controller_after_left_controller,
        delay_right_gripper_ready_waiter_after_left_controller,
        delay_right_controller_after_gripper_ready,
        delay_rviz_after_right_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)

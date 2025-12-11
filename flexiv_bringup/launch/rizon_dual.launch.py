from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
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
    rizon_type_left_param_name = "rizon_type_left"
    rizon_type_right_param_name = "rizon_type_right"
    robot_sn_left_param_name = "robot_sn_left"
    robot_sn_right_param_name = "robot_sn_right"
    rdk_control_mode_param_name = "rdk_control_mode"
    start_rviz_param_name = "start_rviz"
    use_fake_hardware_param_name = "use_fake_hardware"
    fake_sensor_commands_param_name = "fake_sensor_commands"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            rizon_type_left_param_name,
            description="Type of the left Flexiv Rizon robot.",
            default_value="Rizon4",
            choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            rizon_type_right_param_name,
            description="Type of the right Flexiv Rizon robot.",
            default_value="Rizon4R",
            choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
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

    # Initialize Arguments
    rizon_type_left = LaunchConfiguration(rizon_type_left_param_name)
    rizon_type_right = LaunchConfiguration(rizon_type_right_param_name)
    robot_sn_left = LaunchConfiguration(robot_sn_left_param_name)
    robot_sn_right = LaunchConfiguration(robot_sn_right_param_name)
    rdk_control_mode = LaunchConfiguration(rdk_control_mode_param_name)
    start_rviz = LaunchConfiguration(start_rviz_param_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_param_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_param_name)

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

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "rizon_dual.urdf.xacro"]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_urdf_xacro,
                " ",
                "robot_sn_left:=",
                robot_sn_left,
                " ",
                "robot_sn_right:=",
                robot_sn_right,
                " ",
                "rizon_type_left:=",
                rizon_type_left,
                " ",
                "rizon_type_right:=",
                rizon_type_right,
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
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("flexiv_bringup"), "config", "rizon_dual_controllers.yaml"]
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
                    "left_arm_controller/joint_states",
                    "right_arm_controller/joint_states",
                    "joint_states",
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
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run right arm controller
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run Flexiv robot states broadcaster left
    flexiv_robot_states_broadcaster_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster_left"],
        parameters=[{"robot_sn": robot_sn_left}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Run Flexiv robot states broadcaster right
    flexiv_robot_states_broadcaster_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster_right"],
        parameters=[{"robot_sn": robot_sn_right}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Run gpio controller
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Delay start of controllers after `joint_state_broadcaster`
    delay_left_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    delay_right_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    # Delay rviz start after `joint_state_broadcaster` (just to be safe)
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
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
        gpio_controller_spawner,
        delay_left_controller_after_jsb,
        delay_right_controller_after_jsb,
        delay_rviz_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes)

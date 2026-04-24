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
)


def load_yaml(package_name, file_path, replacements=None):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            yaml_content = file.read()

        if replacements:
            for key, value in replacements.items():
                yaml_content = yaml_content.replace(key, value)

        return yaml.safe_load(yaml_content)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context):
    # Initialize Arguments
    rizon_type_left = LaunchConfiguration("rizon_type_left")
    rizon_type_right = LaunchConfiguration("rizon_type_right")
    robot_sn_left = LaunchConfiguration("robot_sn_left")
    robot_sn_right = LaunchConfiguration("robot_sn_right")

    robot_sn_left_str = robot_sn_left.perform(context)
    robot_sn_right_str = robot_sn_right.perform(context)

    rdk_control_mode = LaunchConfiguration("rdk_control_mode")
    start_rviz = LaunchConfiguration("start_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    load_gripper_left = LaunchConfiguration("load_gripper_left")
    gripper_name_left = LaunchConfiguration("gripper_name_left")
    load_gripper_right = LaunchConfiguration("load_gripper_right")
    gripper_name_right = LaunchConfiguration("gripper_name_right")

    load_mounted_ft_sensor_left = LaunchConfiguration("load_mounted_ft_sensor_left")
    load_mounted_ft_sensor_right = LaunchConfiguration("load_mounted_ft_sensor_right")
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

    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # Construct prefixes
    prefix_left_str = "left_" + robot_sn_left_str + "_"
    prefix_right_str = "right_" + robot_sn_right_str + "_"

    set_prefix_left = SetLaunchConfiguration(name="prefix_left", value=prefix_left_str)
    set_prefix_right = SetLaunchConfiguration(
        name="prefix_right", value=prefix_right_str
    )

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
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # MoveIt configuration
    flexiv_srdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_moveit_config"), "srdf", "rizon_dual.srdf.xacro"]
    )

    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_srdf_xacro,
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
                "load_gripper_right:=",
                load_gripper_right,
                " ",
                "load_mounted_ft_sensor_left:=",
                load_mounted_ft_sensor_left,
                " ",
                "load_mounted_ft_sensor_right:=",
                load_mounted_ft_sensor_right,
                " ",
                "prefix_left:=",
                "left_",
                " ",
                "prefix_right:=",
                "right_",
            ]
        ),
        value_type=str,
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    # Trajectory Execution Configuration
    replacements = {
        "$(var prefix_left)": prefix_left_str,
        "$(var prefix_right)": prefix_right_str,
    }

    robot_description_kinematics_yaml = load_yaml(
        "flexiv_moveit_config", "config/dual_arm/kinematics_dual.yaml", replacements
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
        "flexiv_moveit_config", "config/dual_arm/ompl_planning_dual.yaml", replacements
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "flexiv_moveit_config",
        "config/dual_arm/moveit_controllers_dual.yaml",
        replacements,
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

    # Load for left arm
    joint_limits_left = load_yaml(
        "flexiv_moveit_config",
        "config/joint_limits.yaml",
        {"$(var robot_sn)": prefix_left_str.rstrip("_")},
    )
    joint_limits_right = load_yaml(
        "flexiv_moveit_config",
        "config/joint_limits.yaml",
        {"$(var robot_sn)": prefix_right_str.rstrip("_")},
    )

    joint_limits_yaml = {"robot_description_planning": {"joint_limits": {}}}

    if joint_limits_left and "joint_limits" in joint_limits_left:
        joint_limits_yaml["robot_description_planning"]["joint_limits"].update(
            joint_limits_left["joint_limits"]
        )

    if joint_limits_right and "joint_limits" in joint_limits_right:
        joint_limits_yaml["robot_description_planning"]["joint_limits"].update(
            joint_limits_right["joint_limits"]
        )

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
        [FindPackageShare("flexiv_bringup"), "config", "rizon_dual_controllers.yaml"]
    )

    # Run controller manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
            {"robot_sn_left": robot_sn_left},
            {"robot_sn_right": robot_sn_right},
            {"prefix_left": prefix_left_str},
            {"prefix_right": prefix_right_str},
            {"rdk_control_mode": rdk_control_mode},
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

    # Run robot controller
    left_rizon_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_rizon_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    right_rizon_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_rizon_arm_controller",
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
    flexiv_robot_states_broadcaster_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster_left"],
        condition=UnlessCondition(use_fake_hardware),
    )

    flexiv_robot_states_broadcaster_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster_right"],
        condition=UnlessCondition(use_fake_hardware),
    )

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
            "robot_sn": robot_sn_left,
            "gripper_name": gripper_name_left,
            "use_fake_hardware": use_fake_hardware,
            "gripper_node_name": "left_gripper_node",
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
            "robot_sn": robot_sn_right,
            "gripper_name": gripper_name_right,
            "use_fake_hardware": use_fake_hardware,
            "gripper_node_name": "right_gripper_node",
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
        move_group_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_publisher_node,
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

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rizon_type_left",
            description="Type of the left Flexiv Rizon robot.",
            default_value="Rizon4",
            choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rizon_type_right",
            description="Type of the right Flexiv Rizon robot.",
            default_value="Rizon4R",
            choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_sn_left",
            description="Serial number of the left robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_sn_right",
            description="Serial number of the right robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rdk_control_mode",
            default_value="joint_position",
            description="RDK control mode for the ROS 2 control joint position and velocity interfaces.",
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
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_gripper_left",
            default_value="false",
            description="Flag to load the Flexiv Grav gripper as the end-effector of the left robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_name_left",
            default_value="Flexiv-GN01",
            description="Full name of the left gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_gripper_right",
            default_value="false",
            description="Flag to load the Flexiv Grav gripper as the end-effector of the right robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_name_right",
            default_value="Flexiv-GN01",
            description="Full name of the right gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_mounted_ft_sensor_left",
            default_value="false",
            description="Flag to load the mounted force torque sensor for the left robot. Only available for Rizon4, Rizon4R and Rizon10.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_mounted_ft_sensor_right",
            default_value="false",
            description="Flag to load the mounted force torque sensor for the right robot. Only available for Rizon4, Rizon4R and Rizon10.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path to the warehouse database",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

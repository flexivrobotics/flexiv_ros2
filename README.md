# Flexiv ROS 2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![docs](https://img.shields.io/badge/docs-sphinx-yellow)](https://www.flexiv.com/software/rdk/manual/ros2_bridge.html)

For ROS 2 users to easily work with [RDK](https://github.com/flexivrobotics/flexiv_rdk), the APIs of RDK are wrapped into ROS packages in `flexiv_ros2`. Key functionalities like realtime and non-realtime joint torque and position control are supported, and the integration with `ros2_control` framework and MoveIt! 2 is also implemented.

## References

[Flexiv RDK main webpage](https://www.flexiv.com/software/rdk) contains important information like RDK user manual and network setup.

## Compatibility

| **Supported OS** | **Supported ROS 2 distribution**                              |
| ---------------- | ------------------------------------------------------------- |
| Ubuntu 22.04     | [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) |
| Ubuntu 24.04     | [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)     |

### Release Status

| **ROS 2 Distro**   | Humble               | Jazzy                |
| ------------------ | -------------------- | -------------------- |
| **Branch**         | [humble-v1](https://github.com/flexivrobotics/flexiv_ros2/tree/humble-v1) | [jazzy-v1](https://github.com/flexivrobotics/flexiv_ros2/tree/jazzy-v1) |
| **Release Status** | [![Humble Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml) | [![Jazzy Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=jazzy)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/jazzy-binary-build.yml) |

## Getting Started

This branch targets ROS 2 Humble (Ubuntu 22.04); for ROS 2 Jazzy use the [jazzy-v1](https://github.com/flexivrobotics/flexiv_ros2/tree/jazzy-v1) branch. Other versions of Ubuntu and ROS 2 may work, but are not officially supported.

1. Install [ROS 2 Humble via Debian Packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. Install the build tools. The ROS package dependencies are installed by `rosdep` in step 4:

   ```bash
   sudo apt install -y \
   python3-colcon-common-extensions \
   python3-rosdep \
   python3-vcstool \
   wget
   ```

3. Setup workspace:

   ```bash
   mkdir -p ~/flexiv_ros2_ws/src
   cd ~/flexiv_ros2_ws/src
   git clone https://github.com/flexivrobotics/flexiv_ros2.git -b humble-v1
   ```

4. Install dependencies:

   ```bash
   cd ~/flexiv_ros2_ws
   vcs import src < src/flexiv_ros2/flexiv.humble.repos --recursive --skip-existing
   touch src/flexiv_rdk/COLCON_IGNORE
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
   ```

5. Choose a directory for installing `flexiv_rdk` library and all its dependencies. For example, a new folder named `flexiv_install` under the home directory: `~/flexiv_install`. Compile and install to the installation directory:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_rdk/thirdparty
   bash build_and_install_dependencies.sh ~/flexiv_install
   ```

6. Configure and install `flexiv_rdk`:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_rdk
   rm -rf build && mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=~/flexiv_install
   cmake --build . --target install --config Release
   ```

7. Build and source the workspace:

   ```bash
   cd ~/flexiv_ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=~/flexiv_install
   source install/setup.bash
   ```

### Flexiv DRDK Installation (Optional)

If you are using a Flexiv dual robot setup, you can install `flexiv_drdk` as well.

1. Clone `flexiv_drdk` into the workspace source directory and ignore it from colcon build:

   ```bash
   cd ~/flexiv_ros2_ws/src
   git clone --branch v1.2.4 --depth 1 https://github.com/flexivrobotics/flexiv_drdk.git
   touch flexiv_drdk/COLCON_IGNORE
   ```

2. Install dependencies and build `flexiv_drdk` by choosing an installation directory, e.g., `~/flexiv_install`:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_drdk/thirdparty
   bash build_and_install_dependencies.sh ~/flexiv_install --skip-rdk
   ```

3. Configure and install `flexiv_drdk`:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_drdk
   rm -rf build && mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=~/flexiv_install -DCMAKE_PREFIX_PATH=~/flexiv_install
   cmake --build . --target install --config Release
   ```

4. Rebuild the workspace with both RDK and DRDK installation paths:

   ```bash
   cd ~/flexiv_ros2_ws
   colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=~/flexiv_install
   ```

> [!IMPORTANT]
> Remember to source the setup file and the workspace whenever a new terminal is opened:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/flexiv_ros2_ws/install/setup.bash
> ```

## Usage

> [!NOTE]
> The instruction below is only a quick reference, see the [Flexiv ROS 2 Documentation](https://www.flexiv.com/software/rdk/manual/ros2_bridge.html) for more information.

The prerequisites of using ROS 2 with Flexiv Rizon robot are [enable RDK on the robot server](https://www.flexiv.com/software/rdk/manual/activate_rdk_server.html) and [establish connection](https://www.flexiv.com/software/rdk/manual/establish_connection.html) between the workstation PC and the robot.

> [!TIP]
> If ROS 2 discovery is slow, or topics and services of the driver are missing, while connected to the robot, restrict ROS 2 discovery to the workstation PC in every terminal, including the one running the driver. This also stops ROS 2 nodes on other machines from communicating with it.
>
> ```bash
> export ROS_LOCALHOST_ONLY=1
> ```

The main launch file to start the robot driver is the `rizon.launch.py` - it loads and starts the robot hardware, joint states broadcaster, Flexiv robot states broadcasters, and robot controller and opens RViZ. The arguments for the launch file are as follows:

- `robot_sn` (*required*) - Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
- `robot_type` (default: *Rizon4*) - type of the Flexiv robot. (Rizon4, Rizon4M, Rizon4R, Rizon4s, Rizon10, Rizon10s or Rizon10R)
- `rdk_control_mode` (default: *joint_position*) - Flexiv RDK control mode for ROS 2 joint position and velocity interfaces. Options: *joint_position* or *joint_impedance*. In joint impedance mode the controller's impedance properties can be set at runtime, see [Joint Impedance Configuration](#joint-impedance-configuration)
- `load_gripper` (default: *false*) - loads the Flexiv Grav gripper as the end-effector of the robot and the gripper control node.
- `gripper_name` (default: *Flexiv-GN01*) - full name of the gripper to be controlled, as shown in Flexiv Elements -> Settings -> Device.
- `load_mounted_ft_sensor` (default: *false*) - loads the mounted force-torque sensor. Only available for Rizon4, Rizon4R, Rizon10 and Rizon10R.
- `use_fake_hardware` (default: *false*) - starts `mock_components/GenericSystem` instead of real hardware. This is a simple simulation that mirrors joint commands to their states. The gripper, GPIO and Flexiv robot states are not available with it.
- `start_rviz` (default: *true*) - starts RViz automatically with the launch file.
- `fake_sensor_commands` (default: *false*) - enables fake command interfaces for sensors used for simulations. Used only if `use_fake_hardware` parameter is true.
- `robot_controller` (default: *rizon_arm_controller*) - robot controller to start. Available controllers: *rizon_arm_controller*, *cartesian_motion_force_controller* (see [Cartesian Motion-Force Control](#cartesian-motion-force-control))
- `kinematics_params_file` (default: *empty*) - kinematics YAML file holding this robot's measured parameters, as generated by [`flexiv_calibration`](#robot-calibration). When empty, `flexiv_description/config/[robot_type]/default_kinematics.yaml` is used.

There are extra or different launch arguments for Flexiv AICO1, AICO2, and dual robot setups. *(Details about other launch files can be found in [`flexiv_bringup`](/flexiv_bringup))*

- `robot_sn_left` (*required for dual robot setup*) - Serial number of the left robot to connect to. Remove any space, for example: Rizon4-123456
- `robot_sn_right` (*required for dual robot setup*) - Serial number of the right robot to connect to. Remove any space, for example: Rizon4R-654321
- `robot_type` (default: *AICO1-4-V1* for `aico1.launch.py`, *AICO2-4-V1* for `aico2.launch.py`) - type of the Flexiv AICO robot platform. AICO1 options: *AICO1-4-V1*, *AICO1-4-V2*. AICO2 options: *AICO2-4-V1*, *AICO2-4-V2*, *AICO2-4-D3*, *AICO2-4E-D1*, *AICO2-4U-D1*, *AICO2-10-V1*, *AICO2-10-D2*, *AICO2-10E-D1*, *AICO2-10U-D1*
- `kinematics_params_file_left`, `kinematics_params_file_right` (default: *empty*, dual robot setups) - per-arm equivalents of `kinematics_params_file`.
- `robot_controller` (default: *rizon_arm_controller*, dual robot setups) - *rizon_arm_controller* starts `left_rizon_arm_controller` and `right_rizon_arm_controller`. *cartesian_motion_force_controller* starts one controller for both arms and loads the arm controllers inactive.
- `load_gripper_left`, `load_gripper_right`, `gripper_name_left`, `gripper_name_right`, `load_mounted_ft_sensor_left`, `load_mounted_ft_sensor_right` (dual robot setups) - per-arm equivalents of `load_gripper`, `gripper_name` and `load_mounted_ft_sensor`.
- `arm_type_left`, `arm_type_right` (default: *Rizon4* and *Rizon4R*, `rizon_dual.launch.py`) - type of each arm.
- `arm_type` (AICO setups) - arm carried by the platform. `aico1.launch.py` defaults to *Rizon4* (options: *Rizon4*, *Rizon4s*). `aico2.launch.py` defaults to empty, which picks the arm the selected `robot_type` carries: *Rizon4* for the AICO2-4 platforms, *Rizon10* for the AICO2-10 ones.
- `external_axis_prefix` (default: *empty*, AICO setups) - prefix for the external axis links and joints.

### Example Commands

1. Start robot, or fake hardware:

   - Test with real robot:

     ```bash
     ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] robot_type:=Rizon4
     ```

   - Test with fake hardware (`ros2_control` capability):

     ```bash
     ros2 launch flexiv_bringup rizon.launch.py robot_sn:=Rizon4-123456 use_fake_hardware:=true
     ```

> [!TIP]
> To test whether the connection between ROS and the robot is established, you could disable the starting of RViz first by setting the `start_rviz` launch argument to false.

2. Publish commands to controllers

   - To send the goal position to the controller by using the node from `flexiv_test_nodes`, start the following command in a new terminal:

     ```bash
     ros2 launch flexiv_bringup test_joint_trajectory_controller.launch.py robot_sn:=[robot_sn]
     ```

     The joint position goals can be changed in `flexiv_bringup/config/joint_trajectory_position_publisher.yaml`

#### AICO1 and AICO2 Example Commands

**AICO1-4** robot:

```bash
ros2 launch flexiv_bringup aico1.launch.py robot_sn:=[robot_sn] robot_type:=AICO1-4-V1
```

**AICO2-4** robot:

```bash
ros2 launch flexiv_bringup aico2.launch.py robot_sn_left:=[robot_sn_left] robot_sn_right:=[robot_sn_right] robot_type:=AICO2-4-V1
```

**AICO2-10** robot:

```bash
ros2 launch flexiv_bringup aico2.launch.py robot_sn_left:=[robot_sn_left] robot_sn_right:=[robot_sn_right] robot_type:=AICO2-10E-D1
```

### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=[robot_sn]
```

Test with fake hardware:

```bash
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=Rizon4-123456 use_fake_hardware:=true
```

With dual robot setup:

```bash
ros2 launch flexiv_bringup rizon_dual_moveit.launch.py robot_sn_left:=[robot_sn_left] robot_sn_right:=[robot_sn_right]
```

With AICO1-4 setup:

```bash
ros2 launch flexiv_bringup aico1_moveit.launch.py robot_sn:=[robot_sn] robot_type:=AICO1-4-V1
```

With AICO2-4 setup:

```bash
ros2 launch flexiv_bringup aico2_moveit.launch.py robot_sn_left:=[robot_sn_left] robot_sn_right:=[robot_sn_right] robot_type:=AICO2-4-V1
```

With AICO2-10 setup:

```bash
ros2 launch flexiv_bringup aico2_moveit.launch.py robot_sn_left:=[robot_sn_left] robot_sn_right:=[robot_sn_right] robot_type:=AICO2-10E-D1
```

### Robot States

The robot driver (`rizon.launch.py`) publishes the following feedback states to the respective ROS topics. In the topic names, dashes in `${robot_sn}` become underscores (`Rizon4-123456` -> `Rizon4_123456`); in dual robot setups `${robot_sn}` is `left_${robot_sn_left}` or `right_${robot_sn_right}`:

- `/${robot_sn}/flexiv_robot_states`: [Flexiv robot states](https://www.flexiv.com/software/rdk/api/structflexiv_1_1rdk_1_1_robot_states.html) including the joint- and Cartesian-space robot states. [[`flexiv_msgs/msg/RobotStates.msg`](flexiv_msgs/msg/RobotStates.msg)]
- `/joint_states`: Measured joint states of the robot: joint position, velocity and torque. [[`sensor_msgs/msg/JointState`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/JointState.html)]
- `/${robot_sn}/tcp_pose`: Measured TCP pose expressed in world frame $^{0}T_{TCP}$ in position $[m]$ and quaternion. [[`geometry_msgs/msg/PoseStamped`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/PoseStamped.html)]
- `/${robot_sn}/tcp_velocity`: Measured TCP velocity expressed in world frame $^{0}\dot{x}$ in linear $[m/s]$ and angular $[rad/s]$ velocity, carried in the `accel` field. [[`geometry_msgs/msg/AccelStamped`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/AccelStamped.html)]
- `/${robot_sn}/flange_pose`: Measured flange pose expressed in world frame $^{0}T_{flange}$ in position $[m]$ and quaternion. [[`geometry_msgs/msg/PoseStamped`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/PoseStamped.html)]
- `/${robot_sn}/ft_sensor_wrench`: Force-torque (FT) sensor raw reading in flange frame $^{flange}F_{raw}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/msg/WrenchStamped`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/WrenchStamped.html)]
- `/${robot_sn}/external_wrench_in_tcp`: Estimated external wrench applied on TCP and expressed in TCP frame $^{TCP}F_{ext}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/msg/WrenchStamped`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/WrenchStamped.html)]
- `/${robot_sn}/external_wrench_in_world`: Estimated external wrench applied on TCP and expressed in world frame $^{0}F_{ext}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/msg/WrenchStamped`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/WrenchStamped.html)]

### Fault Handling and Recovery

A fault stops the robot and drops it to `IDLE` control mode. The driver keeps running, publishes the
reason, and exposes a recovery action:

```bash
# Step 1: Diagnose the fault
ros2 topic echo /Rizon4_123456/flexiv_recovery_node/operational_status

# Step 2: Clear the fault and re-enable
ros2 action send_goal /Rizon4_123456/flexiv_recovery_node/error_recovery \
  flexiv_msgs/action/ErrorRecovery "{}" --feedback

# Step 3: Restore the control mode, e.g. NRT_JOINT_POSITION for the position interface
ros2 control switch_controllers --deactivate rizon_arm_controller
ros2 control switch_controllers --activate rizon_arm_controller
```

See [`flexiv_hardware/README.md`](flexiv_hardware/README.md#error-recovery) for the recovery policies, the `ClearFault()` guidance and the dual-robot notes.

### Joint Impedance Configuration

In joint impedance control mode (`rdk_control_mode:=joint_impedance`) the impedance properties of the robot's joint motion controller can be set at runtime, one service per RDK call:

```bash
# Read the joint order and the per-joint bounds first, they differ per robot model
ros2 topic echo /Rizon4_123456/flexiv_joint_impedance_config_node/joint_impedance --once

# Joint motion stiffness K_q and damping ratio Z_q, one value per joint in URDF order
ros2 service call /Rizon4_123456/flexiv_joint_impedance_config_node/set_joint_impedance flexiv_msgs/srv/SetJointImpedance "{k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"

# Maximum contact torque
ros2 service call /Rizon4_123456/flexiv_joint_impedance_config_node/set_max_contact_torque flexiv_msgs/srv/SetMaxContactTorque "{max_contact_torques: [50.0, 50.0, 30.0, 30.0, 10.0, 10.0, 10.0]}"

# Inertia shaping scale
ros2 service call /Rizon4_123456/flexiv_joint_impedance_config_node/set_joint_inertia_scale flexiv_msgs/srv/SetJointInertiaScale "{inertia_scales: [1.0, 1.0, 0.9, 0.9, 0.8, 0.8, 0.8]}"
```

The robot resets these properties whenever it enters a control mode, so the driver re-applies what was set on every controller start.

See [`flexiv_hardware/README.md`](flexiv_hardware/README.md#joint-impedance-configuration) for the valid ranges, the hold-and-reapply behaviour and the dual-robot notes.

### Cartesian Motion-Force Control

`cartesian_motion_force_controller` sends TCP pose, wrench and velocity targets to the robot's unified motion-force controller (RDK `SendCartesianMotionForce()` in `NRT_CARTESIAN_MOTION_FORCE` mode). Any Cartesian axes can be force-controlled while the rest stay motion-controlled.

```bash
# Start the robot driver with the Cartesian motion-force controller
ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] robot_controller:=cartesian_motion_force_controller

# Send a target: pose in world frame, wrench in the force control frame, velocity in world frame
# This moves the TCP 5 cm up from the home position
ros2 topic pub /cartesian_motion_force_controller/cartesian_motion_force flexiv_msgs/msg/CartesianMotionForce "{pose: {position: {x: 0.68, y: -0.11, z: 0.34}, orientation: {w: 0.0, x: 0.0, y: 1.0, z: 0.0}}}" --once
```

The force control settings, such as the force-controlled axes and the Cartesian impedance, are set at runtime with services on `/[robot_sn]/flexiv_cartesian_motion_force_config_node/`, while the controller is running. Every controller start, including the one after a fault recovery, begins from the robot's defaults, so set them again after each start.

See [`flexiv_hardware/README.md`](flexiv_hardware/README.md#cartesian-motion-force-configuration) for the interfaces, the services with their valid ranges, and the dual-robot notes.

### GPIO

All digital inputs on the robot control box can be accessed via the ROS topic `/{robot_sn}/gpio_inputs`, which publishes the current state of all the 18 *(16 on control box + 2 inside the wrist connector)* digital input ports *(True: port high, false: port low)*.

The digital output ports on the control box can be set by publishing to the topic `/{robot_sn}/gpio_outputs`. For example:

```bash
ros2 topic pub /Rizon4_123456/gpio_outputs flexiv_msgs/msg/GPIOStates "{states: [{pin: 0, state: true}, {pin: 2, state: true}]}"
```

### Robot Calibration

Every robot leaves the factory with measured kinematic parameters that differ slightly from the nominal ones shipped in `flexiv_description`. The `flexiv_calibration` package reads the actual parameters from a connected robot and syncs them into a kinematics YAML file, so that the URDF describes your specific robot rather than the model.

```bash
ros2 launch flexiv_calibration calibration_correction.launch.py robot_sn:=[robot_sn]
```

By default this updates `flexiv_description/config/[robot_type]/default_kinematics.yaml` in place, which is the file every launch file already reads, so nothing else has to change. It does show up as a local change in `flexiv_description`.

You can also specify a different file to write to, for example if you want to keep the default file intact:

```bash
ros2 launch flexiv_calibration calibration_correction.launch.py robot_sn:=[robot_sn] target_filename:="${HOME}/[robot_sn]_kinematics.yaml"

ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] robot_type:=[robot_type] kinematics_params_file:="${HOME}/[robot_sn]_kinematics.yaml"
```

*(Dual robot setups and the remaining arguments are described in [`flexiv_calibration`](/flexiv_calibration))*

### Gripper Control

The gripper control is implemented in the `flexiv_gripper` package to interface with the gripper that is connected to the robot.

Start the `flexiv_gripper_node` with the following launch file, the default gripper is Flexiv Grav (Flexiv-GN01). This standalone launch uses a normal RDK instance by default, so it can run without the ROS 2 robot driver:

```bash
ros2 launch flexiv_gripper flexiv_gripper.launch.py robot_sn:=[robot_sn] gripper_name:=Flexiv-GN01
```

If the robot driver is already running and you want to avoid creating another normal RDK instance, launch the gripper separately with a lite instance:

```bash
ros2 launch flexiv_gripper flexiv_gripper.launch.py robot_sn:=[robot_sn] gripper_name:=Flexiv-GN01 use_lite_rdk:=true
```

The lite instance requires another normal RDK instance to already be connected to the robot, for example the one created by the ROS 2 robot driver.

Or, you can also start the gripper control with the robot driver if the gripper is Flexiv Grav. In this path the gripper launch is configured to use a lite RDK instance automatically:

```bash
ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] load_gripper:=true
```

#### Gripper Actions

The gripper actions finish when the fingers stop moving, not when the command is sent. `move` succeeds when the gripper reaches the target width and aborts if it stops short, for example on an object, so use `grasp` to hold objects. MoveIt uses the `gripper_action` (`control_msgs/action/GripperCommand`) interface. See [`flexiv_gripper`](flexiv_gripper/README.md) for the parameters and the completion rules.

In a new terminal, send the gripper action `move` goal to open or close the gripper:

```bash
# Closing the gripper
ros2 action send_goal /flexiv_gripper_node/move flexiv_msgs/action/Move "{width: 0.01, velocity: 0.1, max_force: 20}"
# Opening the gripper
ros2 action send_goal /flexiv_gripper_node/move flexiv_msgs/action/Move "{width: 0.09, velocity: 0.1, max_force: 20}"
```

The `grasp` action enables the gripper to grasp with direct force control, but it requires the mounted gripper to support direct force control. Send a `grasp` command to the gripper:

```bash
ros2 action send_goal /flexiv_gripper_node/grasp flexiv_msgs/action/Grasp "{force: 0}"
```

To stop the gripper, send a `stop` service call:

```bash
ros2 service call /flexiv_gripper_node/stop std_srvs/srv/Trigger {}
```

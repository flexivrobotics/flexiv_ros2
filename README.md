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
| Ubuntu 26.04     | [Lyrical Luth](https://docs.ros.org/en/lyrical/index.html)    |

### Release Status

| **ROS 2 Distro**   | Humble                                                              | Jazzy                                                 | Lyrical                                               |
| ------------------ | ------------------------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| **Branch**         | [humble](https://github.com/flexivrobotics/flexiv_ros2/tree/humble) | [jazzy](https://github.com/flexivrobotics/flexiv_ros2/tree/jazzy) | [lyrical](https://github.com/flexivrobotics/flexiv_ros2/tree/lyrical) |
| **Release Status** | [![Humble Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml) | [![Jazzy Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=jazzy)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/jazzy-binary-build.yml) | [![Lyrical Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/lyrical-binary-build.yml/badge.svg?branch=lyrical)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/lyrical-binary-build.yml) |

> [!NOTE]
> **Release lines.** Two parallel version lines are maintained per ROS 2 distribution:
>
> | Line   | Branch                                                                    | RDK      | Supported robots                                        |
> | ------ | ------------------------------------------------------------------------- | -------- | ------------------------------------------------------- |
> | **v2** | [jazzy](https://github.com/flexivrobotics/flexiv_ros2/tree/jazzy)       | RDK v2.x | Enlight-L, Enlight-LL, MICO-Core, MICO-Plus, MICO-Ultra |
> | **v1** | [jazzy-v1](https://github.com/flexivrobotics/flexiv_ros2/tree/jazzy-v1) | RDK v1.x | Rizon 4/4s/4M/4R/10/10s, AICO1/2 |
>
> The default `humble`/`jazzy`/`lyrical` branches track the current **v2** line. For Rizon support on RDK v1.x, use the `humble-v1`/`jazzy-v1`/`lyrical-v1` branches. The two lines use incompatible RDK APIs and are not interchangeable.

## Getting Started

This branch targets ROS 2 Jazzy (Ubuntu 24.04); for ROS 2 Humble use the [humble](https://github.com/flexivrobotics/flexiv_ros2/tree/humble) branch, and for ROS 2 Lyrical use the [lyrical](https://github.com/flexivrobotics/flexiv_ros2/tree/lyrical) branch. Other versions of Ubuntu and ROS 2 may work, but are not officially supported.

1. Install [ROS 2 Jazzy via Debian Packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

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
   git clone https://github.com/flexivrobotics/flexiv_ros2.git -b jazzy
   ```

4. Install dependencies:

   ```bash
   cd ~/flexiv_ros2_ws
   vcs import src < src/flexiv_ros2/flexiv.jazzy.repos --recursive --skip-existing
   touch src/flexiv_rdk/COLCON_IGNORE
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y --skip-keys flexiv_rdk
   ```

5. Choose a directory for installing `flexiv_rdk` library and all its dependencies. For example, a new folder named `rdk_install` under the home directory: `~/rdk_install`. Compile and install to the installation directory:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_rdk/thirdparty
   bash build_and_install_dependencies.sh ~/rdk_install
   ```

6. Configure and install `flexiv_rdk`:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_rdk
   rm -rf build && mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install
   cmake --build . --target install --config Release
   ```

7. Build and source the workspace:

   ```bash
   cd ~/flexiv_ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=~/rdk_install
   source install/setup.bash
   ```

> [!IMPORTANT]
> Remember to source the setup file and the workspace whenever a new terminal is opened:
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/flexiv_ros2_ws/install/setup.bash
> ```

## Usage

> [!NOTE]
> The instruction below is only a quick reference, see the [Flexiv ROS 2 Documentation](https://www.flexiv.com/software/rdk/manual/ros2_bridge.html) for more information.

The prerequisites of using ROS 2 with Flexiv robots are [enable RDK on the robot server](https://www.flexiv.com/software/rdk/manual/activate_rdk_server.html) and [establish connection](https://www.flexiv.com/software/rdk/manual/establish_connection.html) between the workstation PC and the robot.

> [!TIP]
> If ROS 2 discovery is slow, or topics and services of the driver are missing, while connected to the robot, restrict ROS 2 discovery to the workstation PC in every terminal, including the one running the driver. This also stops ROS 2 nodes on other machines from communicating with it.
>
> ```bash
> export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
> ```

All provided launch files prepend `${rdk_install_prefix}/lib` to `LD_LIBRARY_PATH` before starting Flexiv-backed nodes. The default launch argument assumes `flexiv_rdk` was installed to `~/rdk_install`, matching the build steps above. If you installed `flexiv_rdk` to a different prefix, pass `rdk_install_prefix:=/path/to/prefix` to the launch command.

The launch file to start the single-arm robot driver is `flexiv.launch.py` - it loads and starts the robot hardware, joint states broadcaster, Flexiv robot states broadcasters, and robot controller and opens RViZ. The arguments for the launch file are as follows:

- `robot_sn` (*required*) - Serial number of the robot to connect to. Remove any space, for example: Enlight-L-123456
- `robot_type` (default: *Enlight-L*) - type of the Flexiv robot. Supported values: *Enlight-L* (single-arm), *Enlight-LL*, *MICO-Core*, *MICO-Plus*, *MICO-Ultra* (dual-arm). Dual-arm models share one RDK connection (one `robot_sn`) and expose `left_`/`right_` prefixed joints. **Note:** *MICO-Ultra*'s mobile base is not yet supported in `ros2_control` — its arms (and pan-tilt torso) are controllable, but the mobile base is not driven by this stack.
- `rdk_control_mode` (default: *joint_position*) - Flexiv RDK control mode for ROS 2 joint position and velocity interfaces. Options: *joint_position* or *joint_impedance*. In joint impedance mode the controller's impedance properties can be set at runtime, see [Joint Impedance Configuration](#joint-impedance-configuration)
- `rdk_realtime_mode` (default: *true*) - use the real-time RDK modes (`RT_JOINT_POSITION`, `RT_JOINT_IMPEDANCE`, `RT_CARTESIAN_MOTION_FORCE`), streamed at the controller manager rate. *false* selects the non-real-time ones, smoothed by the robot's motion generator within the joint and Cartesian motion limits. See [Real-time control modes](flexiv_hardware/README.md#real-time-control-modes)
- `withhold_on_timeliness_failure` (default: *true*) - with `rdk_realtime_mode:=true`, withhold motion until the controllers are restarted once real-time commands arrive late too often. *false* only logs a warning and keeps streaming.
- `load_gripper` (default: *false*) - loads the Flexiv Grav gripper as the end-effector of the robot and the gripper control node.
- `use_fake_hardware` (default: *false*) - starts `mock_components/GenericSystem` instead of real hardware. This is a simple simulation that mirrors joint commands to their states. The gripper, GPIO and Flexiv robot states are not available with it.
- `start_rviz` (default: *true*) - starts RViz automatically with the launch file.
- `fake_sensor_commands` (default: *false*) - enables fake command interfaces for sensors used for simulations. Used only if `use_fake_hardware` parameter is true.
- `robot_controller` (default: *empty*) - robot controller(s) to start, comma-separated, overriding the default set for the robot type. Leave unset to get the default: *flexiv_arm_controller* on a single-arm robot, *left_flexiv_arm_controller, right_flexiv_arm_controller* on a dual-arm robot, plus *flexiv_torso_controller* on *MICO-Plus* / *MICO-Ultra*. *cartesian_motion_force_controller* starts the Cartesian motion-force controller for all arms instead, see [Cartesian Motion-Force Control](#cartesian-motion-force-control)
- `kinematics_params_file` (default: *empty*) - kinematics YAML file holding this robot's measured parameters, as generated by [`flexiv_calibration`](#robot-calibration). When empty, `flexiv_description/config/[robot_type]/default_kinematics.yaml` is used. One file describes the whole robot: a dual-arm model's file carries an `ARM_1` and an `ARM_2` section (plus `EXT_AXIS` on *MICO-Plus*/*MICO-Ultra*).

### Example Commands

1. Start robot, or fake hardware:

   - Test with real robot:

      ```bash
      ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=[robot_sn] robot_type:=Enlight-L
      ```

   - Test with fake hardware (`ros2_control` capability):

      ```bash
      ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=Enlight-L-123456 use_fake_hardware:=true
      ```

> [!TIP]
> To test whether the connection between ROS and the robot is established, you could disable the starting of RViz first by setting the `start_rviz` launch argument to false.

2. Publish commands to controllers

   - To send the goal position to the controller by using the node from `flexiv_test_nodes`, start the following command in a new terminal:

     ```bash
     ros2 launch flexiv_bringup test_joint_trajectory_controller.launch.py robot_sn:=[robot_sn]
     ```

     The joint position goals can be changed in `flexiv_bringup/config/joint_trajectory_position_publisher.yaml`

     On a dual-arm robot, pass `arm:=left`, `arm:=right` or `arm:=both` to drive the per-arm controllers instead. Each arm has its own controller, so one arm can move while the other holds or runs a different goal.

### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
ros2 launch flexiv_bringup flexiv_moveit.launch.py robot_sn:=[robot_sn]
```

Test with fake hardware:

```bash
ros2 launch flexiv_bringup flexiv_moveit.launch.py robot_sn:=Enlight-L-123456 use_fake_hardware:=true
```

### Robot States

The robot driver (`flexiv.launch.py`) publishes the following feedback states to the respective ROS topics:

- `/${robot_sn}/flexiv_robot_states`: [Flexiv robot states](https://www.flexiv.com/software/rdk/api/structflexiv_1_1rdk_1_1_robot_states.html) including the joint- and Cartesian-space robot states. [[`flexiv_msgs/msg/RobotStates.msg`](flexiv_msgs/msg/RobotStates.msg)]
- `/joint_states`: Measured joint states of the robot: joint position, velocity and torque. [[`sensor_msgs/JointState.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)]
- `/${robot_sn}/tcp_pose`: Measured TCP pose expressed in world frame $^{0}T_{TCP}$ in position $[m]$ and quaternion. [[`geometry_msgs/PoseStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]
- `/${robot_sn}/tcp_twist`: Measured TCP twist expressed in world frame $^{0}\dot{X}$ in linear velocity $[m/s]$ and angular velocity $[rad/s]$. [[`geometry_msgs/TwistStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)]
- `/${robot_sn}/flange_pose`: Measured flange pose expressed in world frame $^{0}T_{flange}$ in position $[m]$ and quaternion. [[`geometry_msgs/PoseStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]
- `/${robot_sn}/raw_ft_sensor`: Raw force-torque sensor reading expressed in flange frame $^{flange}F_{raw}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/WrenchStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)]
- `/${robot_sn}/tcp_wrench_local`: Estimated external wrench applied on TCP and expressed in the local TCP frame $^{TCP}F_{ext}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/WrenchStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)]
- `/${robot_sn}/tcp_wrench`: Estimated external wrench applied on TCP and expressed in world frame $^{0}F_{ext}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/WrenchStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)]
- `/tf`: the measured TCP as frame `${robot_sn}_tcp`, child of `${robot_sn}_flange`, computed from the flange and TCP poses so it follows the active tool. The `tcp_wrench_local` topics are expressed in this frame. Disable it with the broadcaster parameter `publish_tcp_tf: false`.

The aggregated `/${robot_sn}/flexiv_robot_states` message also includes the unfiltered wrench fields `raw_tcp_wrench_local` and `raw_tcp_wrench`, which are not published as separate topics.

For dual-arm models (*Enlight-LL*, *MICO-Core*, *MICO-Plus*, *MICO-Ultra*) two robot-states broadcasters run, publishing per-arm states under `left_`/`right_`-prefixed names, e.g. `/left_${robot_sn}/flexiv_robot_states` and `/right_${robot_sn}/flexiv_robot_states` (and the corresponding `tcp_pose`, `tcp_twist`, wrench topics). `/joint_states` contains all 14 arm joints with `left_`/`right_` prefixes (plus the `${robot_sn}_torso_joint1/2` joints on *MICO-Plus*/*MICO-Ultra*).

### Fault Handling and Recovery

A fault stops the robot and drops it to `IDLE` control mode. The driver keeps running, publishes the
reason, and exposes a recovery action:

```bash
# Step 1: Diagnose the fault
ros2 topic echo /Enlight_L_123456/flexiv_recovery_node/operational_status

# Step 2: Clear the fault and servo the robot back on
ros2 action send_goal /Enlight_L_123456/flexiv_recovery_node/error_recovery \
  flexiv_msgs/action/ErrorRecovery "{}" --feedback

# Step 3: Restore the control mode, e.g. RT_JOINT_POSITION for the position interface
ros2 control switch_controllers \
  --deactivate flexiv_arm_controller --activate flexiv_arm_controller
```

On a dual-arm robot, restart every controller that was running in one switch, e.g. `--deactivate left_flexiv_arm_controller right_flexiv_arm_controller --activate left_flexiv_arm_controller right_flexiv_arm_controller`.

See [`flexiv_hardware/README.md`](flexiv_hardware/README.md#error-recovery) for the recovery policies, the `ClearFault()` guidance and the dual-arm notes.

### Joint Impedance Configuration

In joint impedance control mode (`rdk_control_mode:=joint_impedance`) the impedance properties of the robot's joint motion controller can be set at runtime, one service per RDK call:

```bash
# Read the joint order and the per-joint bounds first, they differ per robot model
ros2 topic echo /Enlight_L_123456/flexiv_joint_impedance_config_node/joint_impedance --once

# Joint motion stiffness K_q and damping ratio Z_q, one value per joint in URDF order
ros2 service call /Enlight_L_123456/flexiv_joint_impedance_config_node/set_joint_impedance flexiv_msgs/srv/SetJointImpedance "{k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"

# Maximum contact torque
ros2 service call /Enlight_L_123456/flexiv_joint_impedance_config_node/set_max_contact_torque flexiv_msgs/srv/SetMaxContactTorque "{max_contact_torques: [50.0, 50.0, 30.0, 30.0, 10.0, 10.0, 10.0]}"

# Inertia shaping scale
ros2 service call /Enlight_L_123456/flexiv_joint_impedance_config_node/set_joint_inertia_scale flexiv_msgs/srv/SetJointInertiaScale "{inertia_scales: [1.0, 1.0, 0.9, 0.9, 0.8, 0.8, 0.8]}"
```

On a dual-arm model this is how one arm is set on its own, and the other arm is left untouched:

```bash
ros2 service call /Enlight_LL_123456/flexiv_joint_impedance_config_node/set_joint_impedance flexiv_msgs/srv/SetJointImpedance \
  "{joint_names: [left_Enlight_LL_123456_joint1, left_Enlight_LL_123456_joint2,
                  left_Enlight_LL_123456_joint3, left_Enlight_LL_123456_joint4,
                  left_Enlight_LL_123456_joint5, left_Enlight_LL_123456_joint6,
                  left_Enlight_LL_123456_joint7],
    k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"
```

The robot resets these properties whenever it enters a control mode, so the driver re-applies what was set on every controller start. The pan-tilt torso of *MICO-Plus*/*MICO-Ultra* is not covered.

See [`flexiv_hardware/README.md`](flexiv_hardware/README.md#joint-impedance-configuration) for the valid ranges, the hold-and-reapply behaviour and the dual-arm notes.

### Cartesian Motion-Force Control

`cartesian_motion_force_controller` sends TCP pose, wrench and velocity targets to the robot's unified motion-force controller (RDK `StreamCartesianMotionForce()` in `RT_CARTESIAN_MOTION_FORCE` mode, or `SendCartesianMotionForce()` in `NRT_CARTESIAN_MOTION_FORCE` mode with `rdk_realtime_mode:=false`). Any Cartesian axes can be force-controlled while the rest stay motion-controlled.

```bash
# Start the robot driver with the Cartesian motion-force controller
ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=[robot_sn] robot_controller:=cartesian_motion_force_controller

# Send a target: pose in world frame, wrench in the force control frame, velocity in world frame
ros2 topic pub /cartesian_motion_force_controller/cartesian_motion_force flexiv_msgs/msg/CartesianMotionForce "{pose: {position: {x: 0.68, y: -0.11, z: 0.34}, orientation: {w: 0.0, x: 0.0, y: 1.0, z: 0.0}}}" --once
```

The force/torque sensors are zeroed when the controller starts, so nothing may be in contact with the robot then. On a dual-arm robot one controller drives both arms, with one topic per arm: `/cartesian_motion_force_controller/left_[robot_sn]/cartesian_motion_force` and `.../right_[robot_sn]/...` (dashes in the serial number become underscores).

The force control settings, such as the force-controlled axes and the Cartesian impedance, are set at runtime with services on `/[robot_sn]/flexiv_cartesian_motion_force_config_node/`, while the controller is running. Every controller start, including the one after a fault recovery, begins from the robot's defaults, so set them again after each start.

See [`flexiv_hardware/README.md`](flexiv_hardware/README.md#cartesian-motion-force-configuration) for the interfaces, the services with their valid ranges, and the dual-arm notes.

### GPIO

All digital inputs can be accessed via the ROS topic `/{robot_sn}/gpio_inputs`, which publishes the current state of all 24 digital input ports (16 on the control box, then the M8 connector of each wrist, then the pogo pin connector of each wrist, 2 ports each) exposed through the Flexiv control interface *(True: port high, false: port low)*.

The digital output ports on the control box can be set by publishing to the topic `/{robot_sn}/gpio_outputs`. For example:

```bash
ros2 topic pub /Enlight_L_123456/gpio_outputs flexiv_msgs/msg/GPIOStates "{states: [{pin: 0, state: true}, {pin: 2, state: true}]}"
```

### Robot Calibration

Every robot leaves the factory with measured kinematic parameters that differ slightly from the nominal ones shipped in `flexiv_description`. The `flexiv_calibration` package reads the actual parameters from a connected robot and syncs them into a kinematics YAML file, so that the URDF describes your specific robot rather than the model.

```bash
ros2 launch flexiv_calibration calibration_correction.launch.py robot_sn:=[robot_sn]
```

By default this updates `flexiv_description/config/[robot_type]/default_kinematics.yaml` in place, which is the file every launch file already reads, so nothing else has to change. It does show up as a local change in `flexiv_description`.

One file describes the whole robot, whatever its topology: a single-arm model's file lists the joints directly, while a dual-arm model's carries an `ARM_1` and an `ARM_2` section (plus `EXT_AXIS` for the *MICO-Plus*/*MICO-Ultra* torso). A single run therefore calibrates both arms.

You can also write to a different file, for example to keep several robots of the same type side by side, and pass it back to the driver:

```bash
ros2 launch flexiv_calibration calibration_correction.launch.py robot_sn:=[robot_sn] target_filename:="${HOME}/[robot_sn]_kinematics.yaml"

ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=[robot_sn] robot_type:=[robot_type] kinematics_params_file:="${HOME}/[robot_sn]_kinematics.yaml"
```

*(The remaining arguments are described in [`flexiv_calibration`](/flexiv_calibration))*

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
ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=[robot_sn] load_gripper:=true
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

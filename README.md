# Flexiv ROS 2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![docs](https://img.shields.io/badge/docs-sphinx-yellow)](https://rdk.flexiv.com/manual/ros2_packages.html)

For ROS 2 users to easily work with [RDK](https://github.com/flexivrobotics/flexiv_rdk), the APIs of RDK are wrapped into ROS packages in `flexiv_ros2`. Key functionalities like real-time joint torque and position control are supported, and the integration with `ros2_control` framework and MoveIt! 2 is also implemented.

## References

[Flexiv RDK main webpage](https://rdk.flexiv.com/) contains important information like RDK user manual and network setup.

## Compatibility

| **Supported OS** | **Supported ROS 2 distribution**                              |
| ---------------- | ------------------------------------------------------------- |
| Ubuntu 20.04     | [Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html)       |
| Ubuntu 22.04     | [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) |

### Release Status

| **ROS 2 Distro**   | Foxy                                                            | Humble                                                |
| ------------------ | --------------------------------------------------------------- | ----------------------------------------------------- |
| **Branch**         | [foxy](https://github.com/flexivrobotics/flexiv_ros2/tree/foxy) | [humble](https://github.com/flexivrobotics/flexiv_ros2) |
| **Release Status** | [![Foxy Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/foxy-binary-build.yml) | [![Humble Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml) |

## Getting Started

This project was developed for ROS 2 Foxy (Ubuntu 20.04) and Humble (Ubuntu 22.04). Other versions of Ubuntu and ROS 2 may work, but are not officially supported.

1. Install [ROS 2 Humble via Debian Packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. Install `colcon` and additional ROS packages:

   ```bash
   sudo apt install -y \
   python3-colcon-common-extensions \
   python3-rosdep2 \
   libeigen3-dev \
   ros-humble-xacro \
   ros-humble-tinyxml2-vendor \
   ros-humble-ros2-control \
   ros-humble-realtime-tools \
   ros-humble-control-toolbox \
   ros-humble-moveit \
   ros-humble-ros2-controllers \
   ros-humble-test-msgs \
   ros-humble-joint-state-publisher \
   ros-humble-joint-state-publisher-gui \
   ros-humble-robot-state-publisher \
   ros-humble-rviz2
   ```

3. Setup workspace:

   ```bash
   mkdir -p ~/flexiv_ros2_ws/src
   cd ~/flexiv_ros2_ws/src
   git clone https://github.com/flexivrobotics/flexiv_ros2.git
   cd flexiv_ros2/
   git submodule update --init --recursive
   ```

4. Install dependencies:

   ```bash
   cd ~/flexiv_ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
   ```

> [!NOTE]
> Skip step 5 and 6 if you have compile and install [flexiv_rdk](https://github.com/flexivrobotics/flexiv_rdk).

5. Choose a directory for installing `flexiv_rdk` library and all its dependencies. For example, a new folder named `rdk_install` under the home directory: `~/rdk_install`. Compile and install to the installation directory:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_ros2/flexiv_hardware/rdk/thirdparty
   bash build_and_install_dependencies.sh ~/rdk_install
   ```

6. Configure and install `flexiv_rdk`:

   ```bash
   cd ~/flexiv_ros2_ws/src/flexiv_ros2/flexiv_hardware/rdk
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install
   cmake --build . --target install --config Release
   ```

7. Build and source the workspace:

   ```bash
   cd ~/flexiv_ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=~/rdk_install
   source install/setup.bash
   ```

> [!NOTE]
> Remember to source the setup file and the workspace whenever a new terminal is opened:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/flexiv_ros2_ws/install/setup.bash
> ```

## Usage

> [!NOTE]
> The instruction below is only a quick reference, see the [Flexiv ROS 2 Documentation](https://rdk.flexiv.com/manual/ros2_bridge.html) for more information.

The prerequisites of using ROS 2 with Flexiv Rizon robot are [enable RDK on the robot server](https://rdk.flexiv.com/manual/activate_rdk_server.html) and [establish connection](https://rdk.flexiv.com/manual/establish_connection.html) between the workstation PC and the robot.

The main launch file to start the robot driver is the `rizon.launch.py` - it loads and starts the robot hardware, joint states broadcaster, Flexiv robot states broadcasters, and robot controller and opens RViZ. The arguments for the launch file are as follows:

- `robot_sn` (*required*) - Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
- `rizon_type` (default: *rizon4*) - type of the Flexiv Rizon robot. (rizon4, rizon4s, rizon10 or rizon10s)
- `use_fake_hardware` (default: *false*) - starts `FakeSystem` instead of real hardware. This is a simple simulation that mimics joint command to their states.
- `start_rviz` (deafult: *true*) - starts RViz automatically with the launch file.
- `fake_sensor_commands` (default: *false*) - enables fake command interfaces for sensors used for simulations. Used only if `use_fake_hardware` parameter is true.
- `robot_controller` (default: *rizon_arm_controller*) - robot controller to start. Available controllers: *forward_position_controller*, *rizon_arm_controller*, *joint_impedance_controller*.

*(Details about other launch files can be found in [`flexiv_bringup`](/flexiv_bringup))*

### Example Commands

1. Start robot, or fake hardware:

   - Test with real robot:

     ```bash
     ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] rizon_type:=rizon4
     ```

   - Test with fake hardware (`ros2_control` capability):

     ```bash
     ros2 launch flexiv_bringup rizon.launch.py robot_sn:=dont-care use_fake_hardware:=true
     ```

> [!TIP]
> To test whether the connection between ROS and the robot is established, you could disable the starting of RViz first by setting the `start_rviz` launch argument to false.

2. Publish commands to controllers

   - To send the goal position to the controller by using the node from `flexiv_test_nodes`, start the following command in a new terminal:

     ```bash
     ros2 launch flexiv_bringup test_joint_trajectory_controller.launch.py
     ```

     The joint position goals can be changed in `flexiv_bringup/config/joint_trajectory_position_publisher.yaml`
   - To test another controller, define it using the `robot_controller` launch argument, for example the `joint_impedance_controller`:

     ```bash
     ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] robot_controller:=joint_impedance_controller
     ```

     Open a new terminal and run the launch file:

     ```bash
     ros2 launch flexiv_bringup sine_sweep_impedance.launch.py
     ```

     The robot should run a sine-sweep motion with joint impedance control.

> [!NOTE]
> The command starts the robot in the joint torque mode. In this mode, gravity and friction are compensated **only** for the robot **without** any attached objects (e.g. the gripper, camera).

> [!NOTE]
> Joint impedance control is not supported in fake/simulated hardware.

### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=[robot_sn]
```

Test with fake hardware:

```bash
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=dont-care use_fake_hardware:=true
```

### Robot States

The robot driver (`rizon.launch.py`) publishes the following feedback states to the respective ROS topics:

- `/${robot_sn}/flexiv_robot_states`: [Flexiv robot states](https://rdk.flexiv.com/api/structflexiv_1_1rdk_1_1_robot_states.html#details) including the joint- and Cartesian-space robot states. [[`flexiv_msgs/msg/RobotStates.msg`](flexiv_msgs/msg/RobotStates.msg)]
- `/joint_states`: Measured joint states of the robot: joint position, velocity and torque. [[`sensor_msgs/JointState.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)]
- `/flexiv_robot_states_broadcaster/tcp_pose`: Measured TCP pose expressed in world frame $^{0}T_{TCP}$ in position $[m]$ and quaternion. [[`geometry_msgs/PoseStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]
- `/flexiv_robot_states_broadcaster/external_wrench_in_tcp`: Estimated external wrench applied on TCP and expressed in TCP frame $^{TCP}F_{ext}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/WrenchStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)]
- `/flexiv_robot_states_broadcaster/external_wrench_in_world`: Estimated external wrench applied on TCP and expressed in world frame $^{0}F_{ext}$ in force $[N]$ and torque $[Nm]$. [[`geometry_msgs/WrenchStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)]

### GPIO

All digital inputs on the robot control box can be accessed via the ROS topic `/gpio_controller/gpio_inputs`, which publishes the current state of all the 16 digital input ports *(True: port high, false: port low)*.

The digital output ports on the control box can be set by publishing to the topic `/gpio_controller/gpio_outputs`. For example:

```bash
ros2 topic pub /gpio_controller/gpio_outputs flexiv_msgs/msg/GPIOStates "{states: [{pin: 0, state: true}, {pin: 2, state: true}]}"
```

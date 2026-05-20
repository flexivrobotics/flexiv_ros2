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

| **ROS 2 Distro**   | Humble                                                              | Jazzy                                                 |
| ------------------ | ------------------------------------------------------------------- | ----------------------------------------------------- |
| **Branch**         | [humble](https://github.com/flexivrobotics/flexiv_ros2/tree/humble) | [jazzy](https://github.com/flexivrobotics/flexiv_ros2/tree/jazzy) |
| **Release Status** | [![Humble Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/humble-binary-build.yml) | [![Jazzy Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=jazzy)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/jazzy-binary-build.yml) |

## Getting Started

This project was developed for ROS 2 Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04). Other versions of Ubuntu and ROS 2 may work, but are not officially supported.

This project uses CycloneDDS middleware (`rmw_cyclonedds_cpp`) for ROS 2 communication.

> [!WARNING]
> Fast DDS middleware (`rmw_fastrtps_cpp`) is not supported in this project.
> It causes a compilation conflict with `flexiv_rdk` (conflicting DDS/CMake targets).
> This guide uses CycloneDDS (`rmw_cyclonedds_cpp`) in all setup and runtime examples.

1. Install [ROS 2 Humble via Debian Packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. Install `colcon` and additional ROS packages:

   ```bash
   sudo apt install -y \
   python3-colcon-common-extensions \
   libeigen3-dev \
   wget \
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
   ros-humble-rviz2 \
   ros-humble-rmw-cyclonedds-cpp
   ```

3. Setup workspace:

   ```bash
   mkdir -p ~/flexiv_ros2_ws/src
   cd ~/flexiv_ros2_ws/src
   git clone https://github.com/flexivrobotics/flexiv_ros2.git -b humble
   ```

4. Install dependencies:

   ```bash
   cd ~/flexiv_ros2_ws
   vcs import src < src/flexiv_ros2/flexiv.humble.repos --recursive --skip-existing
   touch src/flexiv_rdk/COLCON_IGNORE
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
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
   make install
   ```

7. Build and source the workspace:

   ```bash
   cd ~/flexiv_ros2_ws
   source /opt/ros/humble/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=~/rdk_install
   source install/setup.bash
   ```

> [!IMPORTANT]
> Remember to source the setup file and the workspace whenever a new terminal is opened:
>
> ```bash
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> source ~/flexiv_ros2_ws/install/setup.bash
> ```

## Usage

> [!NOTE]
> The instruction below is only a quick reference, see the [Flexiv ROS 2 Documentation](https://www.flexiv.com/software/rdk/manual/ros2_bridge.html) for more information.

The prerequisites of using ROS 2 with Flexiv robots are [enable RDK on the robot server](https://www.flexiv.com/software/rdk/manual/activate_rdk_server.html) and [establish connection](https://www.flexiv.com/software/rdk/manual/establish_connection.html) between the workstation PC and the robot.

Before running any `ros2 launch` command below, make sure CycloneDDS is selected:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

All provided launch files prepend `${rdk_install_prefix}/lib` to `LD_LIBRARY_PATH` before starting Flexiv-backed nodes. The default launch argument assumes `flexiv_rdk` was installed to `~/rdk_install`, matching the build steps above. If you installed `flexiv_rdk` to a different prefix, pass `rdk_install_prefix:=/path/to/prefix` to the launch command.

The launch file to start the single-arm robot driver is `flexiv.launch.py` - it loads and starts the robot hardware, joint states broadcaster, Flexiv robot states broadcasters, and robot controller and opens RViZ. The arguments for the launch file are as follows:

- `robot_sn` (*required*) - Serial number of the robot to connect to. Remove any space, for example: EnlightL-123456
- `robot_type` (default: *EnlightL*) - type of the Flexiv single-arm robot. Supported values: *EnlightL*
- `rdk_control_mode` (default: *joint_position*) - Flexiv RDK control mode for ROS 2 joint position and velocity interfaces. Options: *joint_position* or *joint_impedance*
- `load_gripper` (default: *false*) - loads the Flexiv Grav gripper as the end-effector of the robot and the gripper control node.
- `use_fake_hardware` (default: *false*) - starts `FakeSystem` instead of real hardware. This is a simple simulation that mimics joint command to their states.
- `start_rviz` (default: *true*) - starts RViz automatically with the launch file.
- `fake_sensor_commands` (default: *false*) - enables fake command interfaces for sensors used for simulations. Used only if `use_fake_hardware` parameter is true.
- `robot_controller` (default: *flexiv_arm_controller*) - robot controller to start. Available controllers: *flexiv_arm_controller*

### Example Commands

1. Start robot, or fake hardware:

   - Test with real robot:

      ```bash
      ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=[robot_sn] robot_type:=EnlightL
      ```

   - Test with fake hardware (`ros2_control` capability):

      ```bash
      ros2 launch flexiv_bringup flexiv.launch.py robot_sn:=EnlightL-123456 use_fake_hardware:=true
      ```

> [!TIP]
> To test whether the connection between ROS and the robot is established, you could disable the starting of RViz first by setting the `start_rviz` launch argument to false.

2. Publish commands to controllers

   - To send the goal position to the controller by using the node from `flexiv_test_nodes`, start the following command in a new terminal:

     ```bash
     ros2 launch flexiv_bringup test_joint_trajectory_controller.launch.py robot_sn:=[robot_sn]
     ```

     The joint position goals can be changed in `flexiv_bringup/config/joint_trajectory_position_publisher.yaml`

### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
ros2 launch flexiv_bringup flexiv_moveit.launch.py robot_sn:=[robot_sn]
```

Test with fake hardware:

```bash
ros2 launch flexiv_bringup flexiv_moveit.launch.py robot_sn:=EnlightL-123456 use_fake_hardware:=true
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

The aggregated `/${robot_sn}/flexiv_robot_states` message also includes the unfiltered wrench fields `raw_tcp_wrench_local` and `raw_tcp_wrench`, which are not published as separate topics.

For single-arm launch files, `robot_type` is the model selector.

### GPIO

All digital inputs can be accessed via the ROS topic `/{robot_sn}/gpio_inputs`, which publishes the current state of all 24 digital input ports exposed through the Flexiv control interface *(True: port high, false: port low)*.

The digital output ports on the control box can be set by publishing to the topic `/{robot_sn}/gpio_outputs`. For example:

```bash
ros2 topic pub /EnlightL_123456/gpio_outputs flexiv_msgs/msg/GPIOStates "{states: [{pin: 0, state: true}, {pin: 2, state: true}]}"
```

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

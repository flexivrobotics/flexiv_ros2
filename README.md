# Flexiv ROS2

[![Foxy Binary Build](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/flexivrobotics/flexiv_ros2/actions/workflows/build.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

For ROS2 users to easily work with [RDK](https://github.com/flexivrobotics/flexiv_rdk), the APIs of RDK are wrapped into ROS packages in `flexiv_ros2`. Key functionalities like real-time joint torque and position control are supported, and the integration with `ros2_control` framework and MoveIt 2 is also implemented.

## References

[Flexiv RDK main webpage](https://rdk.flexiv.com/) contains important information like RDK user manual and network setup.

## Compatibility

| **Supported OS**          | **Supported ROS2 distribution**                         |
|---------------------------|---------------------------------------------------------|
| Ubuntu 20.04              | [Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html) |

## Getting Started

This project was developed for ROS2 Foxy on Ubuntu 20.04. Other versions of Ubuntu and ROS2 may work, but are not officially supported.

1. Install [ROS2 Foxy via Debian Packages](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

2. Install `colcon` and additional ROS packages:

    ```bash
    sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    libeigen3-dev \
    ros-foxy-xacro \
    ros-foxy-tinyxml2-vendor \
    ros-foxy-ros2-control \
    ros-foxy-realtime-tools \
    ros-foxy-control-toolbox \
    ros-foxy-moveit \
    ros-foxy-ros2-controllers \
    ros-foxy-test-msgs \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-robot-state-publisher \
    ros-foxy-rviz2
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
    rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
    ```

5. Build and source the workspace:

    ```bash
    cd ~/flexiv_ros2_ws
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    ```

**NOTE**: Remember to source the setup file and the workspace whenever a new terminal is opened:

```bash
source /opt/ros/foxy/setup.bash
source ~/flexiv_ros2_ws/install/setup.bash
```

## Usage

**NOTE**: the instruction below is only a quick reference, see the [Flexiv ROS2 Documentation](https://rdk.flexiv.com/manual/ros2_packages.html) for more information.

The prerequisites of using ROS2 with Flexiv Rizon robot are [enable RDK on the robot server](https://rdk.flexiv.com/manual/getting_started.html#enable-rdk-on-robot-server) and [establish connection](https://rdk.flexiv.com/manual/getting_started.html#establish-connection) between the workstation PC and the robot.

The main launch file to start the robot driver is the `rizon.launch.py` - it loads and starts the robot hardware, joint state broadcaster, controllers and opens RViZ. The arguments for the launch file are as follows:

- `robot_ip` (*required*) - IP address of the robot server (remote).
- `local_ip` (*required*) - IP address of the workstation PC (local).
- `rizon_type` (default: *rizon4*) - type of the Flexiv Rizon robot. (rizon4, rizon4s or rizon10)
- `use_fake_hardware` (default: *false*) - starts `FakeSystem` instead of real hardware. This is a simple simulation that mimics joint command to their states.
- `start_rviz` (deafult: *true*) - starts RViz automatically with the launch file.
- `fake_sensor_commands` (default: *false*) - enables fake command interfaces for sensors used for simulations. Used only if `use_fake_hardware` parameter is true.
- `robot_controller` (default: *rizon_arm_controller*) - robot controller to start. Available controllers: *forward_position_controller*, *rizon_arm_controller*, *joint_impedance_controller*.

*(Details about other launch files can be found in [`flexiv_bringup`](/flexiv_bringup))*

### Example Commands

1. Start robot, or fake hardware:

    - Test with real robot:

        ```bash
        ros2 launch flexiv_bringup rizon.launch.py robot_ip:=[robot_ip] local_ip:=[local_ip]
        ```

        **NOTE**: Getting the following output in terminal is OK: `Warning: Invalid frame ID "link1" passed to canTransform argument source_frame - frame does not exist`. This happens because `joint_state_broadcaster` node need some time to start.

    - Test with fake hardware (`ros2_control` capability):

        ```bash
        ros2 launch flexiv_bringup rizon.launch.py robot_ip:=dont-care local_ip:=dont-care use_fake_hardware:=true
        ```

2. Publish commands to controllers

   - To send the goal position to the controller by using the node from `flexiv_test_nodes`, start the following command in a new terminal:

        ```bash
        ros2 launch flexiv_bringup test_joint_trajectory_controller.launch.py
        ```

        The joint position goals can be changed in `flexiv_bringup/config/joint_trajectory_position_publisher.yaml`.

   - To test another controller, define it using the `robot_controller` launch argument, for example the `joint_impedance_controller`:

        ```bash
        ros2 launch flexiv_bringup rizon.launch.py robot_ip:=[robot_ip] local_ip:=[local_ip] robot_controller:=joint_impedance_controller
        ```

        **NOTE**: The command starts the robot in the joint torque mode. In this mode, gravity and friction are compensated **only** for the robot **without** any attached objects (e.g. the gripper, camera).

        Open a new terminal and run the launch file:

        ```bash
        ros2 launch flexiv_bringup sine_sweep_impedance.launch.py
        ```

        The robot should run a sine-sweep motion with joint impedance control.

        **NOTE**: joint impedance control is not supported in fake/simulated hardware.

### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_ip:=[robot_ip] local_ip:=[local_ip]
```

Test with fake hardware:

```bash
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_ip:=dont-care local_ip:=dont-care use_fake_hardware:=true
```

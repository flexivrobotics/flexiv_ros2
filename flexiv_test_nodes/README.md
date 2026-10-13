# Flexiv Test Nodes

This package contains the example demo nodes to use with/without Flexiv ROS 2 driver.

## Robot States Publisher and Monitor

These nodes demonstrate how to publish and monitor Flexiv robot states directly from the Flexiv RDK using the Python `flexivrdk` package, without relying on the main ROS 2 driver stack.

### Requirements

- ROS 2 Jazzy
- `flexivrdk` Python package (`pip install flexivrdk`), version must match the Flexiv robot software version
- `flexiv_msgs` package (built with `flexiv_ros2`)

### 1. Robot States Publisher

Publishes robot states directly from Flexiv RDK to the ROS 2 topic, bypassing the main ROS 2 driver.

**Features:**

- Direct RDK integration using Python `flexivrdk` package
- Publishes Flexiv robot states at 100 Hz
- Monitors robot status (busy, operational, fault, reduced)

**Use case:** When you need direct robot state monitoring instead of the `flexiv_robot_states_broadcaster` node from the main driver stack.

**Launch:**

```bash
ros2 launch flexiv_test_nodes robot_states_publisher.launch.py robot_sn:=[robot_sn]
```

**Published topic:**

- `/${robot_sn}/flexiv_robot_states` ([`flexiv_msgs/msg/RobotStates.msg`](../flexiv_msgs/msg/RobotStates.msg))

**Parameters:**

- `robot_sn`: Robot serial number (required)
- `network_interface`: IPv4 address of the local network interface to reach the robot through, e.g. `192.168.2.100` (optional, all interfaces are tried if empty).
- `publish_rate`: Publish rate in Hz (default: 100)

### 2. Robot States Monitor

Example subscriber node demonstrating how to receive and process robot states.

**Run:**

```bash
ros2 run flexiv_test_nodes robot_states_monitor --ros-args -p robot_sn:=[robot_sn]
```

> [!NOTE]
>
> - The ROS/RDK version must match the Flexiv robot software version.
> - Topic names are automatically sanitized for robot serial numbers (dash becomes underscore).

## Publisher Joint Trajectory Controller

Example node to send joint position commands to the joint trajectory controller. It is started by `flexiv_bringup`'s `test_joint_trajectory_controller.launch.py`.

## Cartesian Motion-Force Example

Start the driver with the Cartesian motion-force controller first:

```bash
ros2 launch flexiv_bringup rizon.launch.py robot_sn:=[robot_sn] robot_controller:=cartesian_motion_force_controller
```

- `mode:=pure_motion` sweeps the TCP along world Y (or holds it with `hold:=true`), and changes the null-space posture, the Cartesian stiffness and the maximum contact wrench online every 20 seconds.
- `mode:=motion_force` searches for contact along -Z at 0.02 m/s, then presses with 5 N along Z of the `force_frame` (`world` or `tcp`), optionally sweeping along world Y with `polish:=true`.

```bash
ros2 launch flexiv_bringup test_cartesian_motion_force_controller.launch.py robot_sn:=[robot_sn] mode:=motion_force polish:=true
```

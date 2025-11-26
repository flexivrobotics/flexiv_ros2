# Flexiv Test Nodes

This package contains the example demo nodes to use with/without Flexiv ROS 2 driver.

## Robot States Publisher and Monitor

These nodes demonstrate how to publish and monitor Flexiv robot states directly from the Flexiv RDK using the Python `flexivrdk` package, without relying on the main ROS 2 driver stack.

### Requirements

- ROS 2 Humble or Jazzy
- `flexivrdk` Python package (install via `pip install flexivrdk`)
- `flexiv_msgs` package (built with `flexiv_ros2`)

### 1. Robot States Publisher

Publishes robot states directly from Flexiv RDK to the ROS 2 topic, bypassing the main ROS 2 driver.

**Features:**

- Direct RDK integration using Python `flexivrdk` package
- Publishes Flexiv robot states at 100 Hz
- Monitors robot status (busy, operational, fault, reduced)
- Compatible with ROS 2 Humble and Jazzy

**Use case:** When you need direct robot state monitoring instead of the `flexiv_robot_states_broadcaster` node from the main driver stack.

**Launch:**

```bash
ros2 launch flexiv_test_nodes robot_states_publisher.launch.py robot_sn:=[robot_sn]
```

**Published topic:**

- `/${robot_sn}/flexiv_robot_states` ([`flexiv_msgs/msg/RobotStates.msg`](../flexiv_msgs/msg/RobotStates.msg))

**Parameters:**

- `robot_sn`: Robot serial number (required)
- `network_interface`: Network interface name (optional, auto-detect if empty)
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

Example node to send joint position commands to the joint trajectory controller.

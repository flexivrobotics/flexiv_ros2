# Flexiv Test Nodes

Demo nodes for the Flexiv ROS2 driver.

## Nodes

### 1. Robot States Publisher

Publishes robot states directly from Flexiv RDK to ROS2 topics, bypassing the main ROS2 driver.

**Use case:** When you need direct robot state monitoring alongside or instead of the main flexiv_ros2 driver.

**Installation:**
```bash
pip install flexivrdk
```

**Launch:**
```bash
ros2 launch flexiv_test_nodes robot_states_publisher.launch.py robot_sn:=[robot_sn]
```

**Published topic:**
- `/<robot_sn_with_underscores>/flexiv_robot_states` ([flexiv_msgs/RobotStates])

**Parameters:**
- `robot_sn`: Robot serial number (required)
- `network_interface`: Network interface name (optional, auto-detect if empty)
- `publish_rate`: Publishing rate in Hz (default: 100)

**Features:**
- Direct RDK integration using Python flexivrdk package
- Publishes complete robot states at 100 Hz
- Monitors robot status (busy, operational, fault, reduced)
- Compatible with ROS2 Humble and Jazzy

### 2. Robot States Monitor

Example subscriber node demonstrating how to receive and process robot states.

**Run:**
```bash
ros2 run flexiv_test_nodes robot_states_monitor --ros-args -p robot_sn:=[robot_sn]
```

### 3. Publisher Joint Trajectory Controller

Existing test node for joint trajectory control.

## Requirements

- ROS2 Humble or Jazzy
- flexivrdk Python package (install via `pip install flexivrdk`)
- flexiv_msgs package (built with flexiv_ros2)

## Notes

- The RDK version must match your robot firmware version
- Topic names automatically sanitize robot serial numbers (dashes become underscores)
- This node provides state monitoring without requiring the full flexiv_ros2 driver stack
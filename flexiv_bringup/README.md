# flexiv_bringup

This package contains 6 launch files: the main driver launcher, the MoveIt launch file and 4 examples:

- `rizon.launch.py` - the main launcher: starts ros2_control node including hardware interface, joint state broadcaster and a controller, and visualizes the current robot pose in RViZ.

- `rizon_moveit.launch.py` - runs MoveIt together with the driver. The controller for robot joints started in this launch file is *rizon_arm_controller*.

- `test_joint_position_controller.launch.py` - sends joint position goals to the *forward_position_controller*.

- `test_joint_trajectory_controller.launch` - sends joint trajectory goals to the *rizon_arm_controller*.

- `sine_sweep_position.launch.py` - gets current joint states and then performs a sine-sweep motion with *forward_position_controller*.

- `sine_sweep_impedance.launch.py` - gets current joint states and then performs a sine-sweep motion with *joint_impedance_controller*.

**NOTE**: The 4 example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

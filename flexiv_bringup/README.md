# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `rizon.launch.py` - the main launcher: starts *ros2_control* node including hardware interface, runs joint states, Flexiv robot states broadcaster, and a controller, and visualizes the current robot pose in RViZ. The default controller is `rizon_arm_controller`, a joint trajectory controller.
- `rizon_moveit.launch.py` - runs MoveIt together with the driver. The controller for robot joints started in this launch file is *rizon_arm_controller*.
- `test_joint_trajectory_controller.launch` - sends joint trajectory goals to the *rizon_arm_controller*.
- `sine_sweep_position.launch.py` - gets current joint states and then performs a sine-sweep motion with *forward_position_controller*.
- `sine_sweep_impedance.launch.py` - gets current joint states and then performs a sine-sweep motion with *joint_impedance_controller*.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

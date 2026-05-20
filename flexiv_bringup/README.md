# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `flexiv.launch.py` - the main single-arm launcher: starts *ros2_control* node including hardware interface, runs joint states, Flexiv robot states broadcaster, and a controller, and visualizes the current robot pose in RViZ. The default controller is `flexiv_arm_controller`, a joint trajectory controller.
- `flexiv_moveit.launch.py` - runs MoveIt together with the main driver. The controller for robot joints started in this launch file is *flexiv_arm_controller*.
- `test_joint_trajectory_controller.launch.py` - sends joint trajectory goals to the *flexiv_arm_controller*.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

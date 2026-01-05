# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `rizon.launch.py` - the main launcher: starts *ros2_control* node including hardware interface, runs joint states, Flexiv robot states broadcaster, and a controller, and visualizes the current robot pose in RViZ. The default controller is `rizon_arm_controller`, a joint trajectory controller.
- `rizon_moveit.launch.py` - runs MoveIt together with the main driver. The controller for robot joints started in this launch file is *rizon_arm_controller*.
- `test_joint_trajectory_controller.launch` - sends joint trajectory goals to the *rizon_arm_controller*.

There are also launch files for other robot setups:

- `aico1.launch.py` - the main launcher for Flexiv AICO1 robot.
- `aico1_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv AICO1 robot.
- `aico2.launch.py` - the main launcher for Flexiv AICO2 robot.
- `aico2_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv AICO2 robot.
- `rizon_dual.launch.py` - the main launcher for Flexiv Rizon dual robot setup.
- `rizon_dual_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv Rizon dual robot setup.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

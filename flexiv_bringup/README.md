# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `flexiv.launch.py` - preferred generic alias for the main single-arm launcher.
- `flexiv_moveit.launch.py` - preferred generic alias for the single-arm MoveIt launcher.
- `rizon.launch.py` - the main single-arm launcher: starts *ros2_control* node including hardware interface, runs joint states, Flexiv robot states broadcaster, and a controller, and visualizes the current robot pose in RViZ. The default controller is `rizon_arm_controller`, a joint trajectory controller. Single-arm model selection now uses the `robot_type` argument and supports Enlight and the existing Rizon variants.
- `rizon_moveit.launch.py` - runs MoveIt together with the main driver. The controller for robot joints started in this launch file is *rizon_arm_controller*. Single-arm model selection now uses the `robot_type` argument and supports Enlight and the existing Rizon variants.
- `test_joint_trajectory_controller.launch` - sends joint trajectory goals to the *rizon_arm_controller*.

The `rizon.launch.py` and `rizon_moveit.launch.py` filenames are kept as compatibility aliases for existing commands and scripts.

There are also launch files for other robot setups:

- `aico1.launch.py` - the main launcher for Flexiv AICO1 robot.
- `aico1_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv AICO1 robot.
- `aico2.launch.py` - the main launcher for Flexiv AICO2 robot.
- `aico2_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv AICO2 robot.
- `rizon_dual.launch.py` - the main launcher for Flexiv Rizon dual robot setup.
- `rizon_dual_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv Rizon dual robot setup.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

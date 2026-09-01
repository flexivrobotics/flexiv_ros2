# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `flexiv.launch.py` - the main driver launcher: starts the *ros2_control* node including the hardware interface, runs joint states and Flexiv robot states broadcaster(s), starts the arm controller(s), and visualizes the current robot pose in RViZ. A single-arm robot runs one 7-joint joint trajectory controller, `flexiv_arm_controller`. A dual-arm robot runs one per arm, `left_flexiv_arm_controller` and `right_flexiv_arm_controller`, plus `flexiv_torso_controller` on *MICO-Plus* / *MICO-Ultra* - each arm is a separate RDK joint group, so each accepts and preempts trajectory goals independently of the other.
- `flexiv_moveit.launch.py` - runs MoveIt together with the main driver.
- `test_joint_trajectory_controller.launch.py` - sends joint trajectory goals to the arm controller(s). `arm:=single` (default) targets `flexiv_arm_controller`; `arm:=left` / `arm:=right` target one per-arm controller; `arm:=both` runs the independent control demo, starting the right-arm publisher `right_start_delay` seconds into the left arm's trajectory.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

## Controllers by robot type

| robot_type | controllers config | arm controllers |
| --- | --- | --- |
| Enlight-L | `flexiv_controllers.yaml` | `flexiv_arm_controller` (7) |
| Enlight-LL, MICO-Core | `flexiv_dual_controllers.yaml` | `left_flexiv_arm_controller`, `right_flexiv_arm_controller` (7 + 7) |
| MICO-Plus, MICO-Ultra | `flexiv_mico_controllers.yaml` | the above + `flexiv_torso_controller` (2) |

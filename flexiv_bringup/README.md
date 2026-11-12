# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `flexiv.launch.py` - the main driver launcher: starts the *ros2_control* node including the hardware interface, runs joint states and Flexiv robot states broadcaster(s), starts the arm controller(s), and visualizes the current robot pose in RViZ. A single-arm robot runs one 7-joint joint trajectory controller, `flexiv_arm_controller`. A dual-arm robot runs one per arm, `left_flexiv_arm_controller` and `right_flexiv_arm_controller`, plus `flexiv_torso_controller` on *MICO-Plus* / *MICO-Ultra* - each arm is a separate RDK joint group, so each accepts and preempts trajectory goals independently of the other. `robot_controller:=cartesian_motion_force_controller` starts the Cartesian motion-force controller instead, which commands every arm of the robot.
- `flexiv_moveit.launch.py` - runs MoveIt together with the main driver. With `start_servo:=true` it also starts MoveIt Servo (single-arm robots).
- `test_joint_trajectory_controller.launch.py` - sends joint trajectory goals to the arm controller(s). `arm:=single` (default) targets `flexiv_arm_controller`; `arm:=left` / `arm:=right` target one per-arm controller; `arm:=both` runs the independent control demo, starting the right-arm publisher `right_start_delay` seconds into the left arm's trajectory.
- `test_cartesian_motion_force_controller.launch.py` - runs the Cartesian motion-force example against the *cartesian_motion_force_controller* of a single-arm robot, started with `flexiv.launch.py robot_controller:=cartesian_motion_force_controller`.

The RDK control modes are selected with `rdk_control_mode` (`joint_position` or `joint_impedance`) and `rdk_realtime_mode`. With `rdk_realtime_mode:=true` (default) the joint and Cartesian commands are streamed in the real-time modes at the controller manager rate; `false` selects the non-real-time modes, where the robot's motion generator smooths the commands within the joint and Cartesian motion limits.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

## Controllers by robot type

| robot_type | controllers config | arm controllers |
| --- | --- | --- |
| Enlight-L | `flexiv_controllers.yaml` | `flexiv_arm_controller` (7) |
| Enlight-LL, MICO-Core | `flexiv_dual_controllers.yaml` | `left_flexiv_arm_controller`, `right_flexiv_arm_controller` (7 + 7) |
| MICO-Plus, MICO-Ultra | `flexiv_mico_controllers.yaml` | the above + `flexiv_torso_controller` (2) |

Every config also defines `cartesian_motion_force_controller`, claiming the TCP interfaces of all arms of the robot.

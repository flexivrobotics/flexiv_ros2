# Flexiv Gripper

Action server for grippers connected to a Flexiv robot, such as Flexiv Grav (`Flexiv-GN01`).

```bash
ros2 launch flexiv_gripper flexiv_gripper.launch.py robot_sn:=[robot_sn] gripper_name:=Flexiv-GN01
```

By default the node opens a normal RDK connection, so it runs without the robot driver. When the
driver is already connected, pass `use_lite_rdk:=true` to use a lite RDK instance instead.
`flexiv_bringup` does this automatically with `load_gripper:=true`.

## Interfaces

All names are relative to the node, `flexiv_gripper_node` by default.

- `~/move` (`flexiv_msgs/action/Move`): position control. A `velocity` or `max_force` of 0 uses
  `default_velocity` or `default_max_force`.
- `~/grasp` (`flexiv_msgs/action/Grasp`): direct force control, for grippers that support it.
  Positive `force` closes, negative opens.
- `~/gripper_action` (`control_msgs/action/GripperCommand`): for MoveIt. Moves at
  `default_velocity`, with `max_effort` as the force limit, or `default_max_force` when it is 0.
- `~/stop` (`std_srvs/srv/Trigger`): stops the gripper and aborts the active goal.
- `~/gripper_joint_states` (`sensor_msgs/msg/JointState`): finger width as the first entry of
  `gripper_joint_names`.
- `~/ready` (`std_msgs/msg/Bool`, transient local): published once the gripper is initialized.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `robot_sn` | required | Serial number of the robot, for example `Rizon4s-123456` |
| `gripper_name` | required | Gripper name as listed in Flexiv Elements -> Settings -> Device |
| `tool_name` | `gripper_name` | Robot tool to switch to, as listed in Flexiv Elements -> Settings -> Tool, when it is named differently from the gripper |
| `use_lite_rdk` | `false` | Use a lite RDK instance alongside the robot driver |
| `gripper_joint_names` | `[finger_width_joint]` | Joint name published on `~/gripper_joint_states`; prefix it to match the URDF |
| `default_velocity` | `0.1` | Finger velocity when none is given [m/s] |
| `default_max_force` | `20.0` | Force limit when none is given [N] |
| `width_tolerance` | `0.002` | How close to the target width a move counts as reached [m] |
| `action_timeout` | `10.0` | Maximum wait for an action to finish [s] |
| `state_publish_rate` | `50` | Rate of `~/gripper_joint_states` [Hz] |
| `feedback_publish_rate` | `30` | Rate of action feedback and of completion polling [Hz] |

## When an action finishes

The RDK gripper commands return as soon as the robot receives them, so the node polls the gripper
states and reports the result once the fingers stop:

- **Move** succeeds when the fingers stop within `width_tolerance` of the target. If they stop
  short, for example on an object, it aborts with the width reached in `error`. Use `grasp` or
  `gripper_action` to hold an object.
- **Grasp** succeeds once the fingers stop. The gripper keeps applying the force afterwards. If
  the gripper states do not change at all, the gripper ignored the command, which happens when it
  does not support direct force control, and the grasp aborts.
- **GripperCommand** sets `reached_goal` when the target is reached. Stopping short is reported as
  success with `stalled: true`, as when closing on an object, so MoveIt grasps do not fail.

The fingers count as stopped when the gripper reports `is_moving: false`, or when the width has
stayed within `width_tolerance` for 0.5 s (longer at low velocities). Commands that do not finish
within `action_timeout` abort. A new goal or a `~/stop` call aborts the active goal, and
canceling a goal stops the gripper.

> [!NOTE]
> A lite RDK instance does not receive robot states, and gripper states may be affected too. Until
> the gripper states have been seen to change with `use_lite_rdk:=true`, the node cannot tell
> whether the gripper moved. It then waits for a full stroke at the commanded velocity plus 0.5 s,
> reports success, and logs a warning. Once they change, the states are known to be received and
> every action is verified.

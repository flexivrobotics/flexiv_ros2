# flexiv_hardware

`ros2_control` hardware interfaces for Flexiv robots, backed by Flexiv RDK.

- `FlexivHardwareInterface` — a single robot.
- `FlexivDualHardwareInterface` — a robot pair, backed by Flexiv DRDK. Built only when
  `flexiv_drdk` is found at configure time.

Both export position, velocity and effort command and state interfaces, plus the 18 digital I/O
ports and the aggregated Flexiv robot states.

## Error recovery

A fault stops the robot and drops it to `IDLE` control mode. The driver detects this, withholds all
commands, and publishes the reason. The hardware component stays `ACTIVE`, so topics, broadcasters
and the recovery interface remain available while the robot is faulted.

The interface is namespaced by the robot serial number with `-` replaced by `_`, matching the rest
of the package. For `Rizon4-123456`:

| Type    | Name                                                         |
| ------- | ------------------------------------------------------------ |
| Action  | `/Rizon4_123456/flexiv_recovery_node/error_recovery`         |
| Service | `/Rizon4_123456/flexiv_recovery_node/get_operational_status` |
| Topic   | `/Rizon4_123456/flexiv_recovery_node/operational_status`     |

`ClearFault()` and `Enable()` are not exposed directly.

### Step 1: Diagnosing a fault

```bash
ros2 topic echo /Rizon4_123456/flexiv_recovery_node/operational_status
```

[`OperationalStatus.msg`](../flexiv_msgs/msg/OperationalStatus.msg) reports the robot's operational
status, the driver state, the control mode, the applicable recovery policy, a `message` naming the
required operator action, and `recent_events` — the robot's own error descriptions, probable causes
and recommended actions.

### Step 2: Clearing a fault

```bash
ros2 action send_goal /Rizon4_123456/flexiv_recovery_node/error_recovery flexiv_msgs/action/ErrorRecovery "{}" --feedback
```

The sequence is `Stop` → `ClearFault` → `Enable` → wait for operational. Each step has its own deadline, so an unrecoverable robot fails with a message instead of hanging.

On success the robot is **operational and in `IDLE`**, and the result reports `requires_controller_restart: true`.

### Step 3: Restoring a control mode

Recovery does not restore the control mode. Restart the controller:

```bash
ros2 control switch_controllers --deactivate rizon_arm_controller --activate rizon_arm_controller
```

The switch triggers `perform_command_mode_switch()`, which calls `SwitchMode()` — e.g. `NRT_JOINT_POSITION` for the position interface — and re-synchronizes the command buffer with the measured joint positions in the same step. In a joint impedance control mode it also re-applies whatever joint impedance properties were set, see [Joint impedance configuration](#joint-impedance-configuration).

**The restart is required after every interruption, not only after a recovery action.** Once the driver has left `READY` for any reason, motion stays withheld until a controller restart, even if the robot became operational again on its own or the operator resolved the condition in Flexiv Elements.

### Recovery policies

The action classifies `operational_status()` before acting, and refuses conditions that need a
human.

| Condition                                | Policy             | Behavior                                                 |
| ---------------------------------------- | ------------------ | -------------------------------------------------------- |
| Robot is already ready                   | `NONE`             | Succeeds immediately, robot is left untouched            |
| Minor fault, critical fault, not enabled | `AUTO_RECOVERABLE` | Cleared and re-enabled                                   |
| Booting, releasing brakes                | `TRANSIENT`        | Enabled, then waited out for up to 20 s                  |
| E-stop pressed                           | `SAFETY_LOCKOUT`   | Refused. Release the E-stop                              |
| Recovery state                           | `WAIT_OPERATOR`    | Refused unless `run_auto_recovery: true`                 |
| Reduced state                            | `WAIT_OPERATOR`    | Refused. The TCP crossed a safety plane                  |
| Manual or regular Auto mode              | `WAIT_OPERATOR`    | Refused. Switch to Auto (Remote) in Flexiv Elements      |
| Connection lost                          | `CONNECTION_LOST`  | Hardware component deactivates; reconfigure to reconnect |

Recovery state follows a joint position limit violation and cannot be cleared by `ClearFault()`.
Setting `run_auto_recovery: true` calls `RunAutoRecovery()`, which moves the affected joints slowly
back into range and **requires a robot reboot afterwards**.

### Using ClearFault()

- One call handles both minor and critical faults. It returns as soon as the fault clears, so the
  timeout is only an upper bound. Minor faults clear in under 3 s, critical ones in under 30 s.
- It returns `false` on failure, it does not throw. `false` means a power cycle is needed.
- Clearing a critical fault without a power cycle needs a dedicated device, which may not be
  installed on older robot models.
- Call `estop_released()` before `Enable()`. `Enable()` throws `std::logic_error` when the E-stop is
  pressed.

### Dual robot setups

The recovery interface is namespaced by the **left** robot's serial number and acts on the pair as
one unit: either robot faulted means the pair is faulted, and both must clear for the pair to be
considered clear. DRDK exposes no timeliness accessor for a pair, so that field is always false;
a timeliness failure surfaces as an exception from the streaming call instead.

## Joint impedance configuration

In the joint impedance control modes the robot tracks the streamed positions with its joint impedance controller instead of its position controller. Three properties of that controller can be set at runtime, one service per RDK call:

| RDK call | Service | Latched topic |
| -------- | ------- | ------------- |
| `SetJointImpedance()`    | `~/set_joint_impedance`     | `~/joint_impedance`     |
| `SetMaxContactTorque()`  | `~/set_max_contact_torque`  | `~/max_contact_torque`  |
| `SetJointInertiaScale()` | `~/set_joint_inertia_scale` | `~/joint_inertia_scale` |

The node is namespaced like the recovery interface, by the robot serial number with `-` replaced by
`_`. For `Rizon4-123456`, the first service is
`/Rizon4_123456/flexiv_joint_impedance_config_node/set_joint_impedance`.

Requires `rdk_control_mode:=joint_impedance`. The services are advertised either way, and explain
themselves rather than disappearing when the driver runs in `joint_position` mode.

| Property | Valid range | Unit |
| -------- | ----------- | ---- |
| Joint motion stiffness `K_q`     | `[0, RobotInfo::k_q_nom]`, per joint | Nm/rad |
| Joint motion damping ratio `Z_q` | `[0.3, 0.8]`, nominal 0.7 | –      |
| Maximum contact torque           | `[0, RobotInfo::tau_max]`, per joint | Nm     |
| Inertia shaping scale            | `[0.75, 1.0]`, nominal 1.0 | –     |

The bounds are per joint and differ per robot model, so read them from the topics rather than assuming:

```bash
ros2 topic echo /Rizon4_123456/flexiv_joint_impedance_config_node/joint_impedance --once

ros2 service call /Rizon4_123456/flexiv_joint_impedance_config_node/set_joint_impedance flexiv_msgs/srv/SetJointImpedance "{k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"
```

Notes:
- A stiffness of 0 makes that joint free-floating. This driver streams position commands, so such a joint will sag under gravity, the trajectory controller will accumulate tracking error, and the joint can drift into a soft limit and trigger a safety fault.
- A damping ratio away from the nominal 0.7 may lead to performance and stability issues.

### Dual robot setups

DRDK sets both robots in a single call, so the pair has one joint impedance interface, namespaced by
the **left** robot's serial number. A request covers every joint of both arms in URDF order, and the
`left_`/`right_` joint name prefixes on the topics tell you which entry is which. Both halves are
always sent in full, because an empty half means "nominal" to DRDK and would silently reset the
other arm.

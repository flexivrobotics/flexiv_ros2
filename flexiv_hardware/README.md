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

The switch triggers `perform_command_mode_switch()`, which calls `SwitchMode()` — e.g. `NRT_JOINT_POSITION` for the position interface — and re-synchronizes the command buffer with the measured joint positions in the same step.

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

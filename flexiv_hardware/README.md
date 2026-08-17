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

| Type | Name |
| --- | --- |
| Action | `/Rizon4_123456/flexiv_recovery_node/error_recovery` |
| Service | `/Rizon4_123456/flexiv_recovery_node/get_operational_status` |
| Topic | `/Rizon4_123456/flexiv_recovery_node/operational_status` |

`ClearFault()` and `Enable()` are not exposed directly.

### Diagnosing a fault

```bash
ros2 topic echo /Rizon4_123456/flexiv_recovery_node/operational_status
```

[`OperationalStatus.msg`](../flexiv_msgs/msg/OperationalStatus.msg) reports the robot's operational
status, the driver state, the control mode, the applicable recovery policy, a `message` naming the
required operator action, and `recent_events` — the robot's own error descriptions, probable causes
and recommended actions.

### Clearing a fault

```bash
ros2 action send_goal /Rizon4_123456/flexiv_recovery_node/error_recovery \
  flexiv_msgs/action/ErrorRecovery "{}" --feedback
```

The sequence is `Stop` → `ClearFault` → `Enable` → wait for operational. Each step has its own
deadline, so an unrecoverable robot fails with a message instead of hanging.

On success the robot is **operational and in `IDLE`**, and the result reports
`requires_controller_restart: true`.

### Restoring a control mode

Recovery does not restore the control mode. Restart the controller:

```bash
ros2 control switch_controllers \
  --deactivate rizon_arm_controller --activate rizon_arm_controller
```

The switch triggers `perform_command_mode_switch()`, which calls `SwitchMode()` — `RT_JOINT_TORQUE`
for the effort interface — and re-synchronizes the command buffer with the measured joint positions
in the same step.

Restarting the controller is required rather than optional: the controller re-initializes its own
setpoint, so it cannot apply the stale pre-fault command that would otherwise cause a jump on the
first cycle.

### Recovery policies

The action classifies `operational_status()` before acting, and refuses conditions that need a
human.

| Condition | Policy | Behavior |
| --- | --- | --- |
| Minor fault, critical fault, not enabled | `AUTO_RECOVERABLE` | Cleared and re-enabled |
| Booting, releasing brakes | `TRANSIENT` | Waited out |
| E-stop pressed | `SAFETY_LOCKOUT` | Refused. Release the E-stop |
| Recovery state | `WAIT_OPERATOR` | Refused unless `run_auto_recovery: true` |
| Reduced state | `WAIT_OPERATOR` | Refused. The TCP crossed a safety plane |
| Manual or regular Auto mode | `WAIT_OPERATOR` | Refused. Switch to Auto (Remote) in Flexiv Elements |
| Connection lost | `CONNECTION_LOST` | Hardware component deactivates; reconfigure to reconnect |

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

## Design

`read()` and `write()` stay deterministic. They call only non-blocking RDK accessors and never
`ClearFault()`, `Enable()` or `SwitchMode()`.

- `read()` latches the robot condition into atomics and returns `OK`. It returns `ERROR` only when
  the connection is lost, which is the one fault the driver cannot report in place.
- `write()` returns early unless the driver state is `READY`. This is what guarantees the real-time
  loop issues no RDK call while recovery mutates the robot, without a lock on the hot path.
- `RecoveryNode` runs on the controller manager's executor and owns every blocking call. The
  sequence itself runs on a detached worker so a 30 s `ClearFault()` cannot stall the executor.
- `on_error()` returns `SUCCESS`, leaving the component in `UNCONFIGURED` so it can be reconfigured
  and reactivated. Returning `FAILURE` would finalize it and force a process restart.

| File | Contents |
| --- | --- |
| `robot_system_control.hpp` | Abstraction over the RDK system control API, with a single-robot and a robot-pair implementation, so one recovery sequence serves both hardware interfaces |
| `fault_recovery.hpp` | Fault classification, the latched driver status, and the recovery state machine. No ROS, no live connection, unit tested without hardware |
| `recovery_node.hpp` | The ROS surface: action, service and status publisher |

## Linking against the RDK

```cmake
find_package(flexiv_rdk REQUIRED)
target_link_libraries(<target> flexiv::flexiv_rdk)
```

`flexiv_rdk` is deliberately absent from `package.xml` — it is not a rosdep key and is resolved only
through `-DCMAKE_PREFIX_PATH=~/flexiv_install`.

`libflexiv_rdk.a` is a static archive that calls into fastcdr and spdlog, and the RDK install prefix
ships older copies of both than the ROS build uses. This matters only when linking the RDK into an
**executable**:

- Shared libraries such as `flexiv_hardware` may leave those symbols undefined; the loading ROS
  process supplies them.
- Executables must `find_package(fastcdr REQUIRED NO_CMAKE_PATH)` and
  `find_package(spdlog REQUIRED NO_CMAKE_PATH)` **before** `find_package(flexiv_rdk)`, and link
  both. Getting this wrong loads two ABI-incompatible copies into one process and segfaults.

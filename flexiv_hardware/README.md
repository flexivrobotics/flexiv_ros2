# flexiv_hardware

`ros2_control` hardware interface for Flexiv Enlight and MICO robots, backed by Flexiv RDK v2.x.

`FlexivHardwareInterface` drives the whole robot through a single RDK connection, whether that
robot has one arm (Enlight-L), two arms (Enlight-LL, MICO-Core) or two arms plus a pan-tilt torso
(MICO-Plus, MICO-Ultra). It exports position, velocity and effort command and state interfaces for
every joint, plus the 24 digital I/O ports and the per-joint-group Flexiv robot states.

## Error recovery

A fault stops the robot and drops it to `IDLE` control mode. The driver detects this, withholds all
commands, and publishes the reason. The hardware component stays `ACTIVE`, so topics, broadcasters
and the recovery interface remain available while the robot is faulted.

The interface is namespaced by the robot serial number with `-` replaced by `_`, matching the rest
of the package. For `Enlight-L-123456`:

| Type    | Name                                                              |
| ------- | ----------------------------------------------------------------- |
| Action  | `/Enlight_L_123456/flexiv_recovery_node/error_recovery`           |
| Service | `/Enlight_L_123456/flexiv_recovery_node/get_operational_status`   |
| Topic   | `/Enlight_L_123456/flexiv_recovery_node/operational_status`       |

`ClearFault()` and `ServoOn()` are not exposed directly.

### Step 1: Diagnosing a fault

```bash
ros2 topic echo /Enlight_L_123456/flexiv_recovery_node/operational_status
```

[`OperationalStatus.msg`](../flexiv_msgs/msg/OperationalStatus.msg) reports the robot's operational
status, the driver state, the control mode, the applicable recovery policy, a `message` naming the
required operator action, and `recent_events` — the robot's own error descriptions, probable causes
and recommended actions.

### Step 2: Clearing a fault

```bash
ros2 action send_goal /Enlight_L_123456/flexiv_recovery_node/error_recovery flexiv_msgs/action/ErrorRecovery "{}" --feedback
```

The sequence is `Stop` → `ClearFault` → `ServoOn` → wait for operational. Each step has its own deadline, so an unrecoverable robot fails with a message instead of hanging.

On success the robot is **operational and in `IDLE`**, and the result reports `requires_controller_restart: true`.

### Step 3: Restoring a control mode

Recovery does not restore the control mode. Restart the controller:

```bash
ros2 control switch_controllers --deactivate flexiv_arm_controller --activate flexiv_arm_controller
```

The switch triggers `perform_command_mode_switch()`, which calls `SwitchMode()` — e.g. `RT_JOINT_POSITION` for the position interface — and re-synchronizes the command buffer with the measured joint positions in the same step. In a joint impedance control mode it also re-applies whatever joint impedance properties were set, see [Joint impedance configuration](#joint-impedance-configuration).

**The restart is required after every interruption, not only after a recovery action.** Once the driver has left `READY` for any reason, motion stays withheld until a controller restart, even if the robot became operational again on its own or the operator resolved the condition in Flexiv Elements.

On a dual-arm robot each arm runs its own joint trajectory controller (`left_`/`right_flexiv_arm_controller`, plus `flexiv_torso_controller` on MICO-Plus/Ultra), so restart every controller that was running — the command mode switch covers all joints of the component at once.

### Recovery policies

The action classifies `operational_status()` before acting, and refuses conditions that need a
human.

| Condition                                 | Policy             | Behavior                                                 |
| ----------------------------------------- | ------------------ | -------------------------------------------------------- |
| Robot is already ready                    | `NONE`             | Succeeds immediately, robot is left untouched            |
| Minor fault, critical fault, not servo on | `AUTO_RECOVERABLE` | Cleared and servoed on again                             |
| Booting, releasing brakes                 | `TRANSIENT`        | Servoed on, then waited out for up to 20 s               |
| E-stop pressed                            | `SAFETY_LOCKOUT`   | Refused. Release the E-stop                              |
| Recovery state                            | `WAIT_OPERATOR`    | Refused. Run the recovery operation in Flexiv Elements   |
| Reduced state                             | `WAIT_OPERATOR`    | Refused. The TCP crossed a safety plane                  |
| Manual or regular Auto mode               | `WAIT_OPERATOR`    | Refused. Switch to Auto (Remote) in Flexiv Elements      |
| Connection lost                           | `CONNECTION_LOST`  | Hardware component deactivates; reconfigure to reconnect |

Recovery state follows a joint position limit violation and cannot be cleared by `ClearFault()`.
Run the recovery operation in Flexiv Elements, then reboot the robot and restart the driver.

### Using ClearFault()

- One call handles both minor and critical faults. It returns as soon as the fault clears, so the
  timeout is only an upper bound. Minor faults clear in under 3 s, critical ones in under 30 s.
- It returns `false` on failure, it does not throw. `false` means a power cycle is needed.
- Clearing a critical fault without a power cycle needs a dedicated device, which may not be
  installed on older robot models.
- Call `estop_released()` before `ServoOn()`. `ServoOn()` throws `std::logic_error` when the E-stop
  is pressed.

## Joint impedance configuration

In the joint impedance control modes the robot tracks the streamed positions with its joint impedance controller instead of its position controller. Three properties of that controller can be set at runtime, one service per RDK call:

| RDK call | Service | Latched topic |
| -------- | ------- | ------------- |
| `SetJointImpedance()`    | `~/set_joint_impedance`     | `~/joint_impedance`     |
| `SetMaxContactTorque()`  | `~/set_max_contact_torque`  | `~/max_contact_torque`  |
| `SetJointInertiaScale()` | `~/set_joint_inertia_scale` | `~/joint_inertia_scale` |

The node is namespaced like the recovery interface, by the robot serial number with `-` replaced by
`_`. For `Enlight-L-123456`, the first service is
`/Enlight_L_123456/flexiv_joint_impedance_config_node/set_joint_impedance`.

Requires `rdk_control_mode:=joint_impedance`. The services are advertised either way, and explain
themselves rather than disappearing when the driver runs in `joint_position` mode.

| Property | Valid range | Unit |
| -------- | ----------- | ---- |
| Joint motion stiffness `K_q`     | `[0, RobotInfo::K_q_nom]`, per joint | Nm/rad |
| Joint motion damping ratio `Z_q` | `[0.3, 0.8]`, nominal 0.7 | –      |
| Maximum contact torque           | `[0, RobotInfo::tau_max]`, per joint | Nm     |
| Inertia shaping scale            | `[0.75, 1.0]`, nominal 1.0 | –     |

The bounds are per joint and differ per robot model, so read them from the topics rather than assuming:

```bash
NS=/Enlight_L_123456/flexiv_joint_impedance_config_node

ros2 topic echo $NS/joint_impedance --once

ros2 service call $NS/set_joint_impedance flexiv_msgs/srv/SetJointImpedance \
  "{k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"
```

Every request may name the joints it sets. Leave `joint_names` empty to address every covered
joint in the published order, or name a subset to change only those and leave the rest as they are:

```bash
ros2 service call $NS/set_joint_impedance flexiv_msgs/srv/SetJointImpedance \
  "{joint_names: [Enlight_L_123456_joint6, Enlight_L_123456_joint7], k_q: [15.0, 15.0]}"
```

A name that the interface does not cover, a joint named twice, or a value count that does not match
`joint_names` is rejected as a whole, and nothing changes. The response's `setting` always reports
every covered joint, so it shows the merged result rather than just what was sent.

### Dual-arm robots

A dual-arm Enlight/MICO is one RDK connection, so there is one set of services for the whole robot,
namespaced by the single `robot_sn`. `joint_names` is how one arm is set on its own — name that
arm's joints and the other arm is left untouched, both in what the driver holds and in what reaches
the robot:

```bash
NS=/Enlight_LL_123456/flexiv_joint_impedance_config_node

# Left arm only. The right arm keeps whatever it already had.
ros2 service call $NS/set_joint_impedance flexiv_msgs/srv/SetJointImpedance \
  "{joint_names: [left_Enlight_LL_123456_joint1, left_Enlight_LL_123456_joint2,
                  left_Enlight_LL_123456_joint3, left_Enlight_LL_123456_joint4,
                  left_Enlight_LL_123456_joint5, left_Enlight_LL_123456_joint6,
                  left_Enlight_LL_123456_joint7],
    k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"

# Both arms at once: leave joint_names empty and give 14 values, left joint1..7 then right joint1..7.
ros2 service call $NS/set_joint_impedance flexiv_msgs/srv/SetJointImpedance \
  "{k_q: [3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0,
          3000.0, 3000.0, 800.0, 800.0, 50.0, 25.0, 25.0]}"
```

The RDK sets a whole joint group at a time, so the driver merges the named joints into the values
it holds and sends only the joint groups the request actually touched: a left-arm request costs one
RDK call, not two. The one exception is the first delivery after a controller start, which carries
every group because nothing has reached the robot yet.

Notes:
- A stiffness of 0 makes that joint free-floating. This driver streams position commands, so such a joint will sag under gravity, the trajectory controller will accumulate tracking error, and the joint can drift into a soft limit and trigger a safety fault.
- A damping ratio away from the nominal 0.7 may lead to performance and stability issues.
- The robot resets these properties on every control mode entry, so the driver re-applies whatever
  was set on every controller start.
- The MICO-Plus and MICO-Ultra pan-tilt torso joints are not covered and cannot be named: the
  robot reports no nominal joint stiffness for them.

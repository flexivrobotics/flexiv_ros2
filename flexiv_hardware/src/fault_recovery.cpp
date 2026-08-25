/**
 * @file fault_recovery.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <algorithm>
#include <cmath>

#include "flexiv_hardware/fault_recovery.hpp"

namespace {

// Per-state deadlines. A minor fault normally clears in under 3 seconds and a critical one in under
// 30, so the clear deadline covers the worst case.
constexpr std::chrono::seconds kClearFaultTimeout {30};
constexpr std::chrono::seconds kWaitFaultClearedTimeout {5};
constexpr std::chrono::seconds kEnableTimeout {5};
constexpr std::chrono::seconds kWaitOperationalTimeout {20};

}

namespace flexiv_hardware {

using flexiv::rdk::OperationalStatus;

//====================================== FAULT CLASSIFICATION ======================================

RecoveryPolicy ClassifyRecoveryPolicy(const RobotCondition& condition)
{
    if (!condition.connected) {
        return RecoveryPolicy::CONNECTION_LOST;
    }

    switch (condition.operational_status) {
        case OperationalStatus::READY:
            // Missing the 1 kHz deadline too often makes the robot reject real-time commands. The
            // robot is still ready, but the condition needs clearing and reporting.
            return condition.reached_timeliness_failure_limit ? RecoveryPolicy::AUTO_RECOVERABLE
                                                              : RecoveryPolicy::NONE;

        case OperationalStatus::BOOTING:
        case OperationalStatus::RELEASING_BRAKE:
            return RecoveryPolicy::TRANSIENT;

        case OperationalStatus::NOT_ENABLED:
        case OperationalStatus::MINOR_FAULT:
        case OperationalStatus::CRITICAL_FAULT:
            return RecoveryPolicy::AUTO_RECOVERABLE;

        case OperationalStatus::ESTOP_NOT_RELEASED:
            return RecoveryPolicy::SAFETY_LOCKOUT;

        case OperationalStatus::IN_REDUCED_STATE:
        case OperationalStatus::IN_RECOVERY_STATE:
        case OperationalStatus::IN_MANUAL_MODE:
        case OperationalStatus::IN_AUTO_MODE:
            return RecoveryPolicy::WAIT_OPERATOR;

        case OperationalStatus::UNKNOWN:
        default:
            return RecoveryPolicy::WAIT_OPERATOR;
    }
}

std::string DescribeRobotCondition(const RobotCondition& condition)
{
    if (!condition.connected) {
        return "Connection lost. Check the network and robot power, then reconfigure the hardware.";
    }

    switch (condition.operational_status) {
        case OperationalStatus::READY:
            if (condition.reached_timeliness_failure_limit) {
                return "Robot missed too many 1 kHz deadlines. Check CPU load and real-time "
                       "scheduling.";
            }
            return "Robot is ready.";

        case OperationalStatus::BOOTING:
            return "Robot is still booting, please wait.";

        case OperationalStatus::RELEASING_BRAKE:
            return "Brake release is in progress, please wait.";

        case OperationalStatus::NOT_ENABLED:
            return "Robot is not enabled. Recovery will enable it.";

        case OperationalStatus::MINOR_FAULT:
            return "Minor fault occurred. Recovery can clear it, normally within 3 seconds.";

        case OperationalStatus::CRITICAL_FAULT:
            return "Critical fault occurred. Recovery can try to clear it; if it fails, power "
                   "cycle the robot.";

        case OperationalStatus::ESTOP_NOT_RELEASED:
            return "Emergency stop is pressed. Release the E-stop, then retry recovery.";

        case OperationalStatus::IN_REDUCED_STATE:
            return "Robot is in reduced state. Move the TCP back inside the safety plane or "
                   "restore the safety input.";

        case OperationalStatus::IN_RECOVERY_STATE:
            return "Joint position limit violated. Send the goal with run_auto_recovery: true.";

        case OperationalStatus::IN_MANUAL_MODE:
            return "Robot is in Manual mode. Switch to Auto (Remote) in Flexiv Elements.";

        case OperationalStatus::IN_AUTO_MODE:
            return "Robot is in regular Auto mode. Switch to Auto (Remote) in Flexiv Elements.";

        case OperationalStatus::UNKNOWN:
        default:
            return "Robot status is unknown. Check the robot in Flexiv Elements.";
    }
}

std::string OperationalStatusName(flexiv::rdk::OperationalStatus status)
{
    const auto index = static_cast<size_t>(status);
    if (index >= flexiv::rdk::kOpStatusNames.size()) {
        return "Unknown status";
    }
    return flexiv::rdk::kOpStatusNames[index];
}

std::string ControlModeName(flexiv::rdk::Mode mode)
{
    const auto index = static_cast<size_t>(mode);
    if (index >= flexiv::rdk::kModeNames.size()) {
        return "UNKNOWN";
    }
    return flexiv::rdk::kModeNames[index];
}

std::string RecoveryPolicyName(RecoveryPolicy policy)
{
    switch (policy) {
        case RecoveryPolicy::NONE:
            return "NONE";
        case RecoveryPolicy::TRANSIENT:
            return "TRANSIENT";
        case RecoveryPolicy::AUTO_RECOVERABLE:
            return "AUTO_RECOVERABLE";
        case RecoveryPolicy::WAIT_OPERATOR:
            return "WAIT_OPERATOR";
        case RecoveryPolicy::SAFETY_LOCKOUT:
            return "SAFETY_LOCKOUT";
        case RecoveryPolicy::CONNECTION_LOST:
            return "CONNECTION_LOST";
        default:
            return "UNKNOWN";
    }
}

double MaxJointDeviation(const std::vector<double>& before, const std::vector<double>& after)
{
    if (before.size() != after.size()) {
        return 0.0;
    }
    double max_deviation = 0.0;
    for (size_t i = 0; i < before.size(); ++i) {
        // A joint buffer that was never populated holds NaN, and every comparison against NaN is
        // false, so such an entry simply never becomes the maximum.
        const double deviation = std::fabs(after[i] - before[i]);
        if (deviation > max_deviation) {
            max_deviation = deviation;
        }
    }
    return max_deviation;
}

std::string SanitizeNamespace(const std::string& robot_sn)
{
    std::string sanitized = robot_sn;
    std::replace(sanitized.begin(), sanitized.end(), '-', '_');
    return sanitized;
}

//========================================= DRIVER STATUS ==========================================

void DriverStatus::Latch(const RobotSystemControl& robot)
{
    connected.store(robot.connected());
    fault.store(robot.fault());
    operational.store(robot.operational());
    estop_released.store(robot.estop_released());
    reduced.store(robot.reduced());
    recovery_state.store(robot.recovery());
    reached_timeliness_failure_limit.store(robot.reached_timeliness_failure_limit());
    operational_status.store(robot.operational_status());
    control_mode.store(robot.mode());
}

RobotCondition DriverStatus::condition() const
{
    return {connected.load(), operational_status.load(), reached_timeliness_failure_limit.load()};
}

DriverState DriverStatus::DeriveDriverState() const
{
    if (!connected.load()) {
        return DriverState::DISCONNECTED;
    }
    if (operational.load()) {
        return DriverState::READY;
    }
    // Distinguish a safety lockout from an ordinary fault, so that the status topic shows why
    // recovery would refuse to run.
    if (ClassifyRecoveryPolicy(condition()) == RecoveryPolicy::SAFETY_LOCKOUT) {
        return DriverState::LOCKOUT;
    }
    return DriverState::FAULT;
}

bool DriverStatus::TryApplyDerivedDriverState()
{
    auto expected = driver_state.load();
    if (expected == DriverState::RECOVERING) {
        return false;
    }
    const auto derived = DeriveDriverState();
    // Anything other than READY means the robot is no longer following the commands it was given,
    // so whatever the controller holds is stale from here on.
    if (derived != DriverState::READY) {
        commands_synchronized.store(false);
    }
    return driver_state.compare_exchange_strong(expected, derived);
}

bool DriverStatus::RequiresControllerRestart() const
{
    return driver_state.load() == DriverState::READY && !commands_synchronized.load();
}

//====================================== RECOVERY SEQUENCE =========================================

std::string RecoveryStateName(RecoveryState state)
{
    switch (state) {
        case RecoveryState::IDLE:
            return "IDLE";
        case RecoveryState::CLASSIFY:
            return "CLASSIFY";
        case RecoveryState::STOP:
            return "STOP";
        case RecoveryState::CLEAR_FAULT:
            return "CLEAR_FAULT";
        case RecoveryState::WAIT_FAULT_CLEARED:
            return "WAIT_FAULT_CLEARED";
        case RecoveryState::ENABLE:
            return "ENABLE";
        case RecoveryState::WAIT_OPERATIONAL:
            return "WAIT_OPERATIONAL";
        case RecoveryState::UNLOCK_EXTERNAL_AXES:
            return "UNLOCK_EXTERNAL_AXES";
        case RecoveryState::RUN_AUTO_RECOVERY:
            return "RUN_AUTO_RECOVERY";
        case RecoveryState::COMPLETE:
            return "COMPLETE";
        case RecoveryState::FAILED:
            return "FAILED";
        default:
            return "UNKNOWN";
    }
}

RecoveryStateMachine::RecoveryStateMachine(RobotSystemControl& robot, bool run_auto_recovery)
: robot_(robot)
, run_auto_recovery_(run_auto_recovery)
, started_at_(Clock::now())
, state_entered_at_(Clock::now())
{
    TransitionTo(RecoveryState::CLASSIFY);
}

double RecoveryStateMachine::elapsed_seconds() const
{
    return std::chrono::duration<double>(Clock::now() - started_at_).count();
}

void RecoveryStateMachine::TransitionTo(RecoveryState next)
{
    state_ = next;
    state_entered_at_ = Clock::now();
}

void RecoveryStateMachine::Fail(const std::string& message)
{
    message_ = message;
    TransitionTo(RecoveryState::FAILED);
}

bool RecoveryStateMachine::DeadlineExceeded(std::chrono::seconds timeout) const
{
    return Clock::now() - state_entered_at_ > timeout;
}

bool RecoveryStateMachine::Step()
{
    try {
        switch (state_) {
            case RecoveryState::CLASSIFY: {
                const auto condition = robot_.condition();
                policy_ = ClassifyRecoveryPolicy(condition);
                message_ = DescribeRobotCondition(condition);

                switch (policy_) {
                    case RecoveryPolicy::NONE:
                        message_ = "Robot is ready, nothing to recover.";
                        TransitionTo(RecoveryState::COMPLETE);
                        break;

                    case RecoveryPolicy::TRANSIENT:
                    case RecoveryPolicy::AUTO_RECOVERABLE:
                        TransitionTo(RecoveryState::STOP);
                        break;

                    case RecoveryPolicy::WAIT_OPERATOR:
                        // A joint position limit violation is the one operator-gated condition
                        // that can be resolved from here, and only on explicit opt-in. It must be
                        // handled before ENABLE, because operational() never becomes true while
                        // the robot is in recovery state.
                        if (robot_.recovery() && run_auto_recovery_) {
                            TransitionTo(RecoveryState::RUN_AUTO_RECOVERY);
                        } else {
                            Fail(message_);
                        }
                        break;

                    case RecoveryPolicy::SAFETY_LOCKOUT:
                    case RecoveryPolicy::CONNECTION_LOST:
                    default:
                        Fail(message_);
                        break;
                }
                break;
            }

            case RecoveryState::STOP:
                // Bring the robot to a complete stop and to IDLE control mode before touching the
                // fault state, so that no motion command is pending when it becomes operational.
                if (robot_.operational()) {
                    try {
                        robot_.Stop();
                    } catch (const std::exception&) {
                    }
                }
                TransitionTo(robot_.fault() ? RecoveryState::CLEAR_FAULT : RecoveryState::ENABLE);
                break;

            case RecoveryState::CLEAR_FAULT:
                // ClearFault() blocks until the fault clears or its own timeout elapses, and
                // reports failure by returning false rather than throwing.
                if (!robot_.ClearFault()) {
                    Fail(
                        "Fault could not be cleared. Power cycle the robot, then restart the "
                        "driver.");
                    break;
                }
                TransitionTo(RecoveryState::WAIT_FAULT_CLEARED);
                break;

            case RecoveryState::WAIT_FAULT_CLEARED:
                if (!robot_.fault()) {
                    TransitionTo(RecoveryState::ENABLE);
                } else if (DeadlineExceeded(kWaitFaultClearedTimeout)) {
                    Fail(
                        "Fault still present after it was reported cleared. Power cycle the "
                        "robot.");
                }
                break;

            case RecoveryState::ENABLE:
                // Enable() throws if the E-stop is not released, so check first and report the
                // real cause instead of an exception.
                if (!robot_.estop_released()) {
                    Fail("Emergency stop is pressed. Release the E-stop and retry recovery.");
                    break;
                }
                robot_.Enable();
                TransitionTo(RecoveryState::WAIT_OPERATIONAL);
                break;

            case RecoveryState::WAIT_OPERATIONAL:
                if (robot_.operational()) {
                    TransitionTo(RecoveryState::UNLOCK_EXTERNAL_AXES);
                } else if (DeadlineExceeded(kWaitOperationalTimeout)) {
                    Fail("Robot did not become operational within the timeout. "
                         + DescribeRobotCondition(robot_.condition()));
                }
                break;

            case RecoveryState::UNLOCK_EXTERNAL_AXES:
                if (robot_.has_external_axes()) {
                    robot_.UnlockExternalAxes();
                }
                message_ = "Robot recovered and is operational in " + ControlModeName(robot_.mode())
                           + " control mode.";
                TransitionTo(RecoveryState::COMPLETE);
                break;

            case RecoveryState::RUN_AUTO_RECOVERY:
                // Moves the affected joints slowly back into the allowed range. A reboot is
                // required afterwards, so this never continues into ENABLE.
                robot_.RunAutoRecovery();
                message_
                    = "Automatic recovery finished. Reboot the robot to complete the "
                      "recovery procedure, then restart the driver.";
                TransitionTo(RecoveryState::COMPLETE);
                break;

            case RecoveryState::COMPLETE:
            case RecoveryState::FAILED:
            case RecoveryState::IDLE:
            default:
                return false;
        }
    } catch (const std::exception& e) {
        Fail("Recovery step " + RecoveryStateName(state_) + " failed: " + e.what());
    }

    // CLEAR_FAULT blocks internally for as long as its own timeout, so guard it here only as a
    // backstop against an RDK call that returns without clearing and without reporting failure.
    if (state_ == RecoveryState::CLEAR_FAULT && DeadlineExceeded(kClearFaultTimeout)) {
        Fail("Timed out waiting for the fault to clear.");
    }
    if (state_ == RecoveryState::ENABLE && DeadlineExceeded(kEnableTimeout)) {
        Fail("Timed out delivering the enable request to the robot.");
    }

    return state_ != RecoveryState::COMPLETE && state_ != RecoveryState::FAILED;
}

} /* namespace flexiv_hardware */

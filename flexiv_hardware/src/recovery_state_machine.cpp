/**
 * @file recovery_state_machine.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "flexiv_hardware/recovery_state_machine.hpp"

namespace {

// Per-state deadlines. A minor fault normally clears in under 3 seconds and a critical one in
// under 30, so the clear deadline covers the worst case.
constexpr std::chrono::seconds kClearFaultTimeout {30};
constexpr std::chrono::seconds kWaitFaultClearedTimeout {5};
constexpr std::chrono::seconds kEnableTimeout {5};
constexpr std::chrono::seconds kWaitOperationalTimeout {20};

}

namespace flexiv_hardware {

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
                        // Nothing to recover, but still leave the robot in a known control mode.
                        TransitionTo(RecoveryState::STOP);
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
                robot_.Stop();
                TransitionTo(
                    robot_.fault() ? RecoveryState::CLEAR_FAULT : RecoveryState::ENABLE);
                break;

            case RecoveryState::CLEAR_FAULT:
                // ClearFault() blocks until the fault clears or its own timeout elapses, and
                // reports failure by returning false rather than throwing.
                if (!robot_.ClearFault()) {
                    Fail("Fault could not be cleared. " + DescribeRobotCondition(robot_.condition())
                         + " A power cycle may be required.");
                    break;
                }
                TransitionTo(RecoveryState::WAIT_FAULT_CLEARED);
                break;

            case RecoveryState::WAIT_FAULT_CLEARED:
                if (!robot_.fault()) {
                    TransitionTo(RecoveryState::ENABLE);
                } else if (DeadlineExceeded(kWaitFaultClearedTimeout)) {
                    Fail("Fault still present after it was reported cleared. "
                         + DescribeRobotCondition(robot_.condition()));
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
                message_ = "Robot recovered and is operational in IDLE control mode. Restart the "
                           "controllers to resume motion.";
                TransitionTo(RecoveryState::COMPLETE);
                break;

            case RecoveryState::RUN_AUTO_RECOVERY:
                // Moves the affected joints slowly back into the allowed range. A reboot is
                // required afterwards, so this never continues into ENABLE.
                robot_.RunAutoRecovery();
                message_ = "Automatic recovery finished. Reboot the robot to complete the "
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
        Fail(std::string("Recovery step ") + RecoveryStateName(state_) + " failed: " + e.what());
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

/**
 * @file operational_status.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "flexiv_hardware/operational_status.hpp"

namespace flexiv_hardware {

using flexiv::rdk::OperationalStatus;

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
        return "Connection with the robot is lost. Check the network link and the robot power, "
               "then reconfigure the hardware component.";
    }

    switch (condition.operational_status) {
        case OperationalStatus::READY:
            if (condition.reached_timeliness_failure_limit) {
                return "Robot is ready but the real-time command timeliness limit was reached. "
                       "The control loop is not meeting its 1 kHz deadline; check CPU load and "
                       "real-time scheduling before resuming a real-time control mode.";
            }
            return "Robot is ready.";

        case OperationalStatus::BOOTING:
            return "Robot system is still booting, please wait.";

        case OperationalStatus::RELEASING_BRAKE:
            return "Brake release is in progress, please wait.";

        case OperationalStatus::NOT_ENABLED:
            return "Robot is not enabled. Recovery will enable it.";

        case OperationalStatus::MINOR_FAULT:
            return "Minor fault occurred. Recovery will clear it, which normally takes no more "
                   "than 3 seconds.";

        case OperationalStatus::CRITICAL_FAULT:
            return "Critical fault occurred. Recovery will try to clear it, which can take up to "
                   "30 seconds. Clearing a critical fault without a power cycle requires a "
                   "dedicated device that may not be installed on older robot models; if it "
                   "cannot be cleared, power cycle the robot.";

        case OperationalStatus::ESTOP_NOT_RELEASED:
            return "Emergency stop is pressed. Release the E-stop before attempting recovery.";

        case OperationalStatus::IN_REDUCED_STATE:
            return "Robot is in reduced state and will not execute commands. The TCP passed "
                   "through a safety plane or the reduced state safety input went low. Move the "
                   "robot back inside the allowed region or restore the safety input.";

        case OperationalStatus::IN_RECOVERY_STATE:
            return "Robot is in recovery state after a joint position limit violation. This "
                   "cannot be cleared with ClearFault(). Send the recovery goal with "
                   "run_auto_recovery set to true to move the affected joints slowly back into "
                   "the allowed range, then reboot the robot as required by the recovery "
                   "procedure.";

        case OperationalStatus::IN_MANUAL_MODE:
            return "Robot is in Manual mode. Switch it to Auto (Remote) mode in Flexiv Elements.";

        case OperationalStatus::IN_AUTO_MODE:
            return "Robot is in regular Auto mode. Switch it to Auto (Remote) mode in Flexiv "
                   "Elements so that it accepts RDK commands.";

        case OperationalStatus::UNKNOWN:
        default:
            return "Robot operational status is unknown. Check the robot in Flexiv Elements.";
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

} /* namespace flexiv_hardware */

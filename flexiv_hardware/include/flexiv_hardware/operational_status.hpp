/**
 * @file operational_status.hpp
 * @brief Maps flexiv::rdk::OperationalStatus onto a recovery policy and an operator-facing
 * message. Pure functions, no RDK connection required.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__OPERATIONAL_STATUS_HPP_
#define FLEXIV_HARDWARE__OPERATIONAL_STATUS_HPP_

#include <string>

#include "flexiv/rdk/data.hpp"

namespace flexiv_hardware {

/**
 * @brief How a given robot condition should be recovered from. Values match the constants in
 * flexiv_msgs/msg/RecoveryPolicy.msg.
 */
enum class RecoveryPolicy : uint8_t
{
    /** Robot is ready, nothing to recover. */
    NONE = 0,

    /** Transient condition that resolves on its own, just wait. */
    TRANSIENT = 1,

    /** Recoverable by ClearFault()/Enable() without operator intervention. */
    AUTO_RECOVERABLE = 2,

    /** Needs a physical or Flexiv Elements action before recovery can proceed. */
    WAIT_OPERATOR = 3,

    /** Safety system is engaged, recovery is refused until it is released. */
    SAFETY_LOCKOUT = 4,

    /** Connection with the robot is lost, the hardware component must be reconfigured. */
    CONNECTION_LOST = 5,
};

/**
 * @brief The robot condition to classify, gathered from the rdk::Robot accessors.
 */
struct RobotCondition
{
    bool connected = false;
    flexiv::rdk::OperationalStatus operational_status
        = flexiv::rdk::OperationalStatus::UNKNOWN;
    bool reached_timeliness_failure_limit = false;
};

/**
 * @brief [Non-blocking] Classify a robot condition into a recovery policy.
 * @param[in] condition Robot condition to classify.
 * @return Recovery policy that applies to [condition].
 */
RecoveryPolicy ClassifyRecoveryPolicy(const RobotCondition& condition);

/**
 * @brief [Non-blocking] Operator-facing explanation of a robot condition and the action needed to
 * resolve it.
 * @param[in] condition Robot condition to describe.
 * @return Human-readable message.
 */
std::string DescribeRobotCondition(const RobotCondition& condition);

/**
 * @brief [Non-blocking] Name of an operational status, using the RDK's own strings.
 * @param[in] status Operational status to name.
 * @return Human-readable name.
 */
std::string OperationalStatusName(flexiv::rdk::OperationalStatus status);

/**
 * @brief [Non-blocking] Name of a recovery policy.
 * @param[in] policy Recovery policy to name.
 * @return Human-readable name.
 */
std::string RecoveryPolicyName(RecoveryPolicy policy);

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__OPERATIONAL_STATUS_HPP_ */

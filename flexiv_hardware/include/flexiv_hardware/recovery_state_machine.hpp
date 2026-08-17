/**
 * @file recovery_state_machine.hpp
 * @brief Explicit state machine that brings a faulted robot back to an operational state. Every
 * state has a deadline, so no step can block forever.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__RECOVERY_STATE_MACHINE_HPP_
#define FLEXIV_HARDWARE__RECOVERY_STATE_MACHINE_HPP_

#include <chrono>
#include <string>

#include "flexiv_hardware/operational_status.hpp"
#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

/**
 * @brief Steps of the recovery sequence. Values match the constants in the feedback section of
 * flexiv_msgs/action/ErrorRecovery.action.
 */
enum class RecoveryState : uint8_t
{
    IDLE = 0,
    CLASSIFY = 1,
    STOP = 2,
    CLEAR_FAULT = 3,
    WAIT_FAULT_CLEARED = 4,
    ENABLE = 5,
    WAIT_OPERATIONAL = 6,
    UNLOCK_EXTERNAL_AXES = 7,
    RUN_AUTO_RECOVERY = 8,
    COMPLETE = 9,
    FAILED = 10,
};

/**
 * @brief [Non-blocking] Name of a recovery state.
 */
std::string RecoveryStateName(RecoveryState state);

/**
 * @brief Recovery sequence driven one step per Step() call, so the caller stays responsive to
 * cancellation and can publish feedback.
 *
 * The sequence always ends with the robot enabled, operational and in IDLE control mode. Restoring
 * the control mode is deliberately left to a controller restart, which re-initializes the
 * controller's own setpoint and therefore cannot apply a stale pre-fault command.
 */
class RecoveryStateMachine
{
public:
    /**
     * @brief Construct the state machine.
     * @param[in] robot System control interface of the robot to recover. Must outlive this object.
     * @param[in] run_auto_recovery Whether the operator opted in to RunAutoRecovery() for a joint
     * position limit violation.
     */
    RecoveryStateMachine(RobotSystemControl& robot, bool run_auto_recovery);

    /**
     * @brief [Blocking] Advance the sequence by one step. Each call performs at most one RDK
     * operation, so it blocks only for as long as that single operation takes.
     * @return True if the sequence is still running; false once it reached COMPLETE or FAILED.
     */
    bool Step();

    /** @brief [Non-blocking] Current step of the sequence. */
    RecoveryState state() const { return state_; }

    /** @brief [Non-blocking] Whether the sequence finished successfully. */
    bool succeeded() const { return state_ == RecoveryState::COMPLETE; }

    /** @brief [Non-blocking] Operator-facing explanation of the outcome so far. */
    const std::string& message() const { return message_; }

    /** @brief [Non-blocking] Policy that applied to the condition found at CLASSIFY. */
    RecoveryPolicy policy() const { return policy_; }

    /** @brief [Non-blocking] Seconds elapsed since the sequence started. */
    double elapsed_seconds() const;

private:
    using Clock = std::chrono::steady_clock;

    /** @brief Move to [next] and reset the per-state deadline. */
    void TransitionTo(RecoveryState next);

    /** @brief Move to FAILED with [message]. */
    void Fail(const std::string& message);

    /** @brief Whether the current state exceeded [timeout]. */
    bool DeadlineExceeded(std::chrono::seconds timeout) const;

    RobotSystemControl& robot_;
    bool run_auto_recovery_;

    RecoveryState state_ = RecoveryState::IDLE;
    RecoveryPolicy policy_ = RecoveryPolicy::NONE;
    std::string message_;

    Clock::time_point started_at_;
    Clock::time_point state_entered_at_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__RECOVERY_STATE_MACHINE_HPP_ */

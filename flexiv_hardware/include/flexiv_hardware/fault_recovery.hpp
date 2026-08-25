/**
 * @file fault_recovery.hpp
 * @brief Fault classification and the recovery sequence.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FAULT_RECOVERY_HPP_
#define FLEXIV_HARDWARE__FAULT_RECOVERY_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/mode.hpp"

#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

//====================================== FAULT CLASSIFICATION ======================================

/**
 * @brief How a robot condition should be recovered from. Values match the constants in
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
 * @brief [Non-blocking] Classify a robot condition into a recovery policy.
 */
RecoveryPolicy ClassifyRecoveryPolicy(const RobotCondition& condition);

/**
 * @brief [Non-blocking] Operator-facing explanation of a robot condition and the action needed to
 * resolve it.
 */
std::string DescribeRobotCondition(const RobotCondition& condition);

/**
 * @brief [Non-blocking] Name of an operational status, using the RDK's own strings.
 */
std::string OperationalStatusName(flexiv::rdk::OperationalStatus status);

/**
 * @brief [Non-blocking] Name of a recovery policy.
 */
std::string RecoveryPolicyName(RecoveryPolicy policy);

/**
 * @brief [Non-blocking] Name of a control mode, using the RDK's own strings.
 */
std::string ControlModeName(flexiv::rdk::Mode mode);

/**
 * @brief [Non-blocking] Make a robot serial number usable as a ROS namespace. Serial numbers
 * contain a hyphen, e.g. Rizon4s-123456, which is not a valid character in a ROS name.
 */
std::string SanitizeNamespace(const std::string& robot_sn);

/**
 * @brief [Non-blocking] Largest absolute per-joint difference between two joint position vectors,
 * in radians. Used to report how far the robot ended up from the last position it was commanded to.
 * @return 0 if the vectors differ in size, so an unpopulated buffer never reports a deviation.
 */
double MaxJointDeviation(const std::vector<double>& before, const std::vector<double>& after);

//========================================= DRIVER STATUS ==========================================

/**
 * @brief State of the driver, tracked separately from the ros2_control lifecycle state. The
 * hardware component stays ACTIVE while the robot is faulted, so that the status topic and the
 * recovery action remain available. Values match the DRIVER_* constants in
 * flexiv_msgs/msg/OperationalStatus.msg.
 */
enum class DriverState : uint8_t
{
    /** Not connected to a robot yet. */
    UNINITIALIZED = 0,

    /** Robot is operational and accepting commands. */
    READY = 1,

    /** Robot is faulted or otherwise not operational, commands are withheld. */
    FAULT = 2,

    /** Recovery is in progress, the real-time loop must not touch the robot. */
    RECOVERING = 3,

    /** Safety system is engaged, recovery is refused until it is released. */
    LOCKOUT = 4,

    /** Connection with the robot is lost. */
    DISCONNECTED = 5,
};

/**
 * @brief Robot condition latched from the robot, shared between the real-time control loop and the
 * recovery node.
 *
 * read() latches the condition every control cycle. The recovery node latches it once more when a
 * recovery sequence ends, so that write() is released without waiting for the next read(). All
 * members are atomic, so neither side needs a lock on the hot path.
 *
 * driver_state is the one field both sides write: recovery claims it for the duration of a
 * sequence, and read() only re-derives it while that claim is not held.
 */
struct DriverStatus
{
    std::atomic<DriverState> driver_state {DriverState::UNINITIALIZED};

    std::atomic<bool> connected {false};
    std::atomic<bool> fault {false};
    std::atomic<bool> operational {false};
    std::atomic<bool> estop_released {false};
    std::atomic<bool> reduced {false};
    std::atomic<bool> recovery_state {false};
    std::atomic<bool> reached_timeliness_failure_limit {false};

    std::atomic<flexiv::rdk::OperationalStatus> operational_status {
        flexiv::rdk::OperationalStatus::UNKNOWN};
    std::atomic<flexiv::rdk::Mode> control_mode {flexiv::rdk::Mode::UNKNOWN};

    /**
     * Whether the joint command buffers are known to match the robot's measured position.
     *
     * Cleared whenever the driver leaves READY, and set again only by a controller restart, which
     * re-initializes the controller's setpoint. Until then write() withholds motion even once the
     * robot is operational again: the commands the controller still holds describe where the robot
     * was before it stopped.
     */
    std::atomic<bool> commands_synchronized {false};

    /**
     * @brief [Non-blocking] Latch every condition field from the robot. Does not touch
     * driver_state; use DeriveDriverState() for that, so that a caller can refresh the condition
     * without also committing to the driver state it implies.
     */
    void Latch(const RobotSystemControl& robot);

    /** @brief [Non-blocking] Rebuild the condition from the latched values. */
    RobotCondition condition() const;

    /**
     * @brief [Non-blocking] The driver state implied by the latched condition. Pure: it reads the
     * latched fields and stores nothing, so the caller decides when to apply it.
     */
    DriverState DeriveDriverState() const;

    /**
     * @brief [Non-blocking] Apply DeriveDriverState() unless a recovery sequence holds the driver
     * state. Called from read().
     *
     * Compare-exchange rather than a plain store: a recovery starting in the instant between
     * reading the state and writing it back must not have its RECOVERING claim overwritten, which
     * would release write() for the rest of the sequence. A lost race just skips this cycle, and
     * the next read() re-derives.
     * @return True if the derived state was applied, false if recovery holds the state.
     */
    bool TryApplyDerivedDriverState();

    /**
     * @brief [Non-blocking] Whether the controllers have to be restarted before the robot moves
     * again: it is ready, but the commands they hold have not been re-synchronized with it.
     */
    bool RequiresControllerRestart() const;
};

//====================================== RECOVERY SEQUENCE =========================================

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
 * A robot that needs no recovery is left strictly alone. Otherwise the sequence ends with the robot
 * enabled, operational and in IDLE control mode. Restoring the control mode is deliberately left to
 * a controller restart, which re-initializes the controller's own setpoint and therefore cannot
 * apply a stale pre-fault command.
 */
class RecoveryStateMachine
{
public:
    /**
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

#endif /* FLEXIV_HARDWARE__FAULT_RECOVERY_HPP_ */

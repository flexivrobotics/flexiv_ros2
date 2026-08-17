/**
 * @file driver_status.hpp
 * @brief Status shared between the real-time control loop and the recovery node.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__DRIVER_STATUS_HPP_
#define FLEXIV_HARDWARE__DRIVER_STATUS_HPP_

#include <atomic>
#include <cstdint>

#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/mode.hpp"

namespace flexiv_hardware {

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
 * @brief [Non-blocking] Name of a driver state.
 */
const char* DriverStateName(DriverState state);

/**
 * @brief Robot condition latched by the real-time control loop and read by the recovery node.
 *
 * read() is the only writer and it only ever stores; the recovery node and the status publisher
 * are the only readers. All members are atomic, so neither side needs a lock on the hot path.
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
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__DRIVER_STATUS_HPP_ */

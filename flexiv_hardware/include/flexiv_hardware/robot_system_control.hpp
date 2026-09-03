/**
 * @file robot_system_control.hpp
 * @brief Abstraction over the SYSTEM CONTROL section of the RDK robot interface, so that the
 * recovery implementation is testable without a robot.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_
#define FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_

#include <string>
#include <vector>

#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/mode.hpp"
#include "flexiv/rdk/robot.hpp"

namespace flexiv_hardware {

/**
 * @brief The robot condition that fault classification is based on.
 */
struct RobotCondition
{
    bool connected = false;
    flexiv::rdk::OperationalStatus operational_status = flexiv::rdk::OperationalStatus::UNKNOWN;
    bool reached_timeliness_failure_limit = false;
};

/**
 * @brief Interface to the robot's system control and status accessors.
 *
 * An Enlight/MICO robot is a single RDK connection even when it carries two arms, so one
 * implementation covers every supported topology. The joint groups are recovered as one unit,
 * which is what the RDK system control calls already do.
 */
class RobotSystemControl
{
public:
    virtual ~RobotSystemControl() = default;

    // Status accessors, all non-blocking
    virtual bool connected() const = 0;
    virtual bool fault() const = 0;
    virtual bool operational() const = 0;
    virtual bool estop_released() const = 0;
    virtual bool recovery() const = 0;
    virtual bool reduced() const = 0;
    virtual bool reached_timeliness_failure_limit() const = 0;
    virtual flexiv::rdk::OperationalStatus operational_status() const = 0;
    virtual flexiv::rdk::Mode mode() const = 0;
    virtual std::vector<flexiv::rdk::RobotEvent> event_log() const = 0;

    // System control, all blocking. Never call these from read() or write().
    virtual void Stop() = 0;
    virtual bool ClearFault() = 0;
    virtual void ServoOn() = 0;

    /** @brief [Non-blocking] Gather the current robot condition for classification. */
    RobotCondition condition() const
    {
        return {connected(), operational_status(), reached_timeliness_failure_limit()};
    }
};

/**
 * @brief RobotSystemControl over an rdk::Robot. Does not own the robot.
 */
class RdkRobotSystemControl : public RobotSystemControl
{
public:
    explicit RdkRobotSystemControl(flexiv::rdk::Robot& robot)
    : robot_(robot)
    {
    }

    bool connected() const override { return robot_.connected(); }
    bool fault() const override { return robot_.fault(); }
    bool operational() const override { return robot_.operational(); }
    bool estop_released() const override { return robot_.estop_released(); }
    bool recovery() const override { return robot_.recovery(); }
    bool reduced() const override { return robot_.reduced(); }
    bool reached_timeliness_failure_limit() const override
    {
        return robot_.reached_timeliness_failure_limit();
    }
    flexiv::rdk::OperationalStatus operational_status() const override
    {
        return robot_.operational_status();
    }
    flexiv::rdk::Mode mode() const override { return robot_.mode(); }
    std::vector<flexiv::rdk::RobotEvent> event_log() const override { return robot_.event_log(); }

    void Stop() override { robot_.Stop(); }
    bool ClearFault() override { return robot_.ClearFault(); }
    void ServoOn() override { robot_.ServoOn(); }

private:
    flexiv::rdk::Robot& robot_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_ */

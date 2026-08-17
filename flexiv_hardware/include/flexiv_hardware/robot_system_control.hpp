/**
 * @file robot_system_control.hpp
 * @brief Abstraction over the SYSTEM CONTROL section of the RDK robot interface, so that the
 * recovery sequence can drive either a single robot or a robot pair.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_
#define FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_

#include <string>
#include <vector>

#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/mode.hpp"
#include "flexiv/rdk/robot.hpp"

#include "flexiv_hardware/operational_status.hpp"

namespace flexiv_hardware {

/**
 * @brief Interface to the robot's system control and status accessors. Implementations wrap either
 * a single rdk::Robot or a drdk::RobotPair.
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

    /** @brief [Non-blocking] Whether the robot has any external axes. */
    virtual bool has_external_axes() const = 0;

    // System control, all blocking. Never call these from read() or write().
    virtual void Stop() = 0;
    virtual bool ClearFault() = 0;
    virtual void Enable() = 0;
    virtual void RunAutoRecovery() = 0;

    /**
     * @brief [Blocking] Unlock the external axes so that they can move. Only applicable in IDLE
     * control mode. Only called when has_external_axes() is true.
     */
    virtual void UnlockExternalAxes() = 0;

    /**
     * @brief [Non-blocking] Gather the current robot condition for classification.
     */
    RobotCondition condition() const
    {
        RobotCondition condition;
        condition.connected = connected();
        condition.operational_status = operational_status();
        condition.reached_timeliness_failure_limit = reached_timeliness_failure_limit();
        return condition;
    }
};

/**
 * @brief RobotSystemControl implementation over a single rdk::Robot. Does not own the robot.
 */
class SingleRobotSystemControl : public RobotSystemControl
{
public:
    explicit SingleRobotSystemControl(flexiv::rdk::Robot& robot)
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
    bool has_external_axes() const override { return robot_.info().DoF_e > 0; }

    void Stop() override { robot_.Stop(); }
    bool ClearFault() override { return robot_.ClearFault(); }
    void Enable() override { robot_.Enable(); }
    void RunAutoRecovery() override { robot_.RunAutoRecovery(); }
    void UnlockExternalAxes() override { robot_.LockExternalAxes(false); }

private:
    flexiv::rdk::Robot& robot_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_ */

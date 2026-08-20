/**
 * @file robot_system_control.hpp
 * @brief Abstraction over the SYSTEM CONTROL section of the RDK robot interface, so that one
 * recovery implementation serves both a single robot and a robot pair.
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

#ifdef FLEXIV_DRDK_AVAILABLE
#include "flexiv/drdk/robot_pair.hpp"
#endif

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
    virtual bool has_external_axes() const = 0;

    // System control, all blocking. Never call these from read() or write().
    virtual void Stop() = 0;
    virtual bool ClearFault() = 0;
    virtual void Enable() = 0;
    virtual void RunAutoRecovery() = 0;

    /**
     * @brief [Blocking] Unlock the external axes so that they can move. Only applicable in IDLE
     * control mode, and only called when has_external_axes() is true.
     */
    virtual void UnlockExternalAxes() = 0;

    /** @brief [Non-blocking] Gather the current robot condition for classification. */
    RobotCondition condition() const
    {
        return {connected(), operational_status(), reached_timeliness_failure_limit()};
    }
};

/**
 * @brief RobotSystemControl over a single rdk::Robot. Does not own the robot.
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

#ifdef FLEXIV_DRDK_AVAILABLE

/**
 * @brief RobotSystemControl over a drdk::RobotPair. Does not own the pair.
 *
 * DRDK reports most accessors per robot as a std::pair. They are reduced to a single value here so
 * that the pair recovers as one unit: any robot faulted means the pair is faulted, and both must be
 * clear for the pair to be considered clear.
 */
class DualRobotSystemControl : public RobotSystemControl
{
public:
    explicit DualRobotSystemControl(flexiv::drdk::RobotPair& robot_pair)
    : robot_pair_(robot_pair)
    {
    }

    bool connected() const override { return robot_pair_.connected(); }
    bool fault() const override { return robot_pair_.fault(); }
    bool operational() const override { return robot_pair_.operational(); }
    bool has_external_axes() const override
    {
        return robot_pair_.info().first.DoF_e > 0 || robot_pair_.info().second.DoF_e > 0;
    }

    bool estop_released() const override
    {
        const auto released = robot_pair_.estop_released();
        return released.first && released.second;
    }

    bool recovery() const override
    {
        const auto in_recovery = robot_pair_.recovery();
        return in_recovery.first || in_recovery.second;
    }

    bool reduced() const override
    {
        const auto is_reduced = robot_pair_.reduced();
        return is_reduced.first || is_reduced.second;
    }

    // DRDK exposes no timeliness accessor for a pair; a timeliness failure surfaces as a
    // runtime_error thrown by the streaming calls instead.
    bool reached_timeliness_failure_limit() const override { return false; }

    flexiv::rdk::OperationalStatus operational_status() const override
    {
        // Report the condition that needs attention, so that the pair is only READY when both are.
        const auto status = robot_pair_.operational_status();
        if (status.first != flexiv::rdk::OperationalStatus::READY) {
            return status.first;
        }
        return status.second;
    }

    flexiv::rdk::Mode mode() const override
    {
        // Both robots are always commanded together, so report the shared mode and fall back to
        // UNKNOWN if they ever diverge.
        const auto modes = robot_pair_.mode();
        return modes.first == modes.second ? modes.first : flexiv::rdk::Mode::UNKNOWN;
    }

    std::vector<flexiv::rdk::RobotEvent> event_log() const override
    {
        const auto logs = robot_pair_.event_log();
        std::vector<flexiv::rdk::RobotEvent> merged = logs.first;
        merged.insert(merged.end(), logs.second.begin(), logs.second.end());
        return merged;
    }

    void Stop() override { robot_pair_.Stop(); }
    void Enable() override { robot_pair_.Enable(); }
    void RunAutoRecovery() override { robot_pair_.RunAutoRecovery(); }

    bool ClearFault() override
    {
        const auto cleared = robot_pair_.ClearFault();
        return cleared.first && cleared.second;
    }

    void UnlockExternalAxes() override
    {
        robot_pair_.LockExternalAxes(
            {robot_pair_.info().first.DoF_e == 0, robot_pair_.info().second.DoF_e == 0});
    }

private:
    flexiv::drdk::RobotPair& robot_pair_;
};

#endif /* FLEXIV_DRDK_AVAILABLE */

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__ROBOT_SYSTEM_CONTROL_HPP_ */

/**
 * @file flexiv_dual_hardware_interface.hpp
 * @brief Hardware interface to a pair of Flexiv robots for ROS 2 control.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_DUAL_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_DUAL_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

// ROS
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// ros2_control hardware_interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// Flexiv
#include "flexiv/drdk/robot_pair.hpp"

#include "flexiv_hardware/driver_status.hpp"
#include "flexiv_hardware/recovery_node.hpp"
#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT
};

/**
 * @brief RobotSystemControl implementation over a drdk::RobotPair. Does not own the pair.
 *
 * DRDK exposes a smaller status surface than rdk::Robot: it reports fault() and operational() for
 * the pair, but not the individual conditions behind them. The accessors that have no DRDK
 * equivalent are derived from those two, which is enough to clear faults and re-enable, but not
 * enough to tell an E-stop apart from a robot left in Manual mode. Such a condition surfaces as a
 * failed enable rather than as a specific operator message.
 */
class DualRobotSystemControl : public RobotSystemControl
{
public:
    explicit DualRobotSystemControl(flexiv::drdk::RobotPair& robot_pair)
    : robot_pair_(robot_pair)
    {
    }

    bool fault() const override { return robot_pair_.fault(); }
    bool operational() const override { return robot_pair_.operational(); }

    flexiv::rdk::Mode mode() const override
    {
        // The pair reports a mode per robot. Both are always commanded together, so report the
        // shared mode and fall back to UNKNOWN if they ever diverge.
        const auto modes = robot_pair_.mode();
        return modes.first == modes.second ? modes.first : flexiv::rdk::Mode::UNKNOWN;
    }
    bool has_external_axes() const override
    {
        return robot_pair_.info().first.DoF_e > 0 || robot_pair_.info().second.DoF_e > 0;
    }

    // Derived, see the class note above.
    bool connected() const override { return true; }
    bool estop_released() const override { return true; }
    bool recovery() const override { return false; }
    bool reduced() const override { return false; }
    bool reached_timeliness_failure_limit() const override { return false; }

    flexiv::rdk::OperationalStatus operational_status() const override
    {
        if (robot_pair_.fault()) {
            return flexiv::rdk::OperationalStatus::MINOR_FAULT;
        }
        if (robot_pair_.operational()) {
            return flexiv::rdk::OperationalStatus::READY;
        }
        return flexiv::rdk::OperationalStatus::NOT_ENABLED;
    }

    std::vector<flexiv::rdk::RobotEvent> event_log() const override { return {}; }

    void Stop() override { robot_pair_.Stop(); }
    void Enable() override { robot_pair_.Enable(); }

    bool ClearFault() override
    {
        // The pair reports per-robot results; the fault is only cleared if both succeeded.
        const auto result = robot_pair_.ClearFault();
        return result.first && result.second;
    }

    void UnlockExternalAxes() override
    {
        robot_pair_.LockExternalAxes(
            {robot_pair_.info().first.DoF_e == 0, robot_pair_.info().second.DoF_e == 0});
    }

    void RunAutoRecovery() override
    {
        // DRDK exposes no automatic recovery for a pair. recovery() reports false, so the recovery
        // sequence never reaches this state.
        throw std::runtime_error(
            "Automatic recovery is not available for a dual robot setup. Recover each robot "
            "individually using Flexiv Elements.");
    }

private:
    flexiv::drdk::RobotPair& robot_pair_;
};

class FlexivDualHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivDualHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief [Blocking] Wait for both robots to become operational, up to [timeout].
     * @return True if both became operational, false on timeout.
     */
    bool WaitUntilOperational(std::chrono::seconds timeout);

    /**
     * @brief Set the joint command buffers to hold the currently measured position, so that
     * resuming control cannot apply a stale command.
     */
    void SynchronizeCommandsWithState();

    /** @brief Remove the recovery node from the executor and destroy it. */
    void TeardownRecoveryNode();

    // Flexiv DRDK
    std::unique_ptr<flexiv::drdk::RobotPair> robot_pair_;

    // Recovery interface, hosted on the controller manager's executor
    std::unique_ptr<RobotSystemControl> robot_system_control_;
    std::shared_ptr<DriverStatus> driver_status_;
    std::shared_ptr<RecoveryNode> recovery_node_;
    rclcpp::Executor::WeakPtr executor_;

    // RDK control mode for joint position and velocity interfaces
    flexiv::rdk::Mode rdk_control_mode_;

    // External axis type
    std::string external_axis_type_ = "";

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Robot States
    flexiv::rdk::RobotStates hw_flexiv_robot_states_left_;
    flexiv::rdk::RobotStates hw_flexiv_robot_states_right_;
    flexiv::rdk::RobotStates* hw_flexiv_robot_states_addr_left_ = &hw_flexiv_robot_states_left_;
    flexiv::rdk::RobotStates* hw_flexiv_robot_states_addr_right_ = &hw_flexiv_robot_states_right_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_left_;
    std::map<unsigned int, bool> current_digital_outputs_right_;

    // Joint mapping
    struct JointMap
    {
        int robot_index; // 0: Left, 1: Right
        int dof_index;   // Index in the robot's q vector
    };
    std::vector<JointMap> joint_map_;

    static rclcpp::Logger getLogger();

    // Control modes
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_DUAL_HARDWARE_INTERFACE_HPP_ */

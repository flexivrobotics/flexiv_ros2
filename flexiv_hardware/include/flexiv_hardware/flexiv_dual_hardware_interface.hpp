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
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "flexiv_hardware/visibility_control.h"

// Flexiv
#include "flexiv/drdk/robot_pair.hpp"

namespace flexiv_hardware {

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT
};

class FlexivDualHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivDualHardwareInterface)

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    FLEXIV_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    FLEXIV_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Flexiv DRDK
    std::unique_ptr<flexiv::drdk::RobotPair> robot_pair_;

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

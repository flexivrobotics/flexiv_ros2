/**
 * @file joint_impedance_controller.cpp
 * @brief Joint impedance control as ROS 2 controller. Adapted from
 * ros2_controllers/forward_command_controller
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "joint_impedance_controller/joint_impedance_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace joint_impedance_controller {
using hardware_interface::LoanedCommandInterface;

JointImpedanceController::JointImpedanceController()
: controller_interface::ControllerInterface()
, rt_command_ptr_(nullptr)
, joints_command_subscriber_(nullptr)
{
}

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (const auto& joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size() * 2);
    for (const auto& joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template <typename T>
bool get_ordered_interfaces(std::vector<T>& unordered_interfaces,
    const std::vector<std::string>& joint_names, const std::string& interface_type,
    std::vector<std::reference_wrapper<T>>& ordered_interfaces)
{
    for (const auto& joint_name : joint_names) {
        for (auto& command_interface : unordered_interfaces) {
            if ((command_interface.get_name() == joint_name)
                && (command_interface.get_interface_name() == interface_type)) {
                ordered_interfaces.push_back(std::ref(command_interface));
            }
        }
    }

    return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn JointImpedanceController::on_init()
{
    try {
        // definition of the parameters that need to be queried from the
        // controller configuration file with default values
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
        auto_declare<std::vector<double>>("k_p", std::vector<double>());
        auto_declare<std::vector<double>>("k_d", std::vector<double>());
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // getting the names of the joints
    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter not set");
        return CallbackReturn::FAILURE;
    }
    // getting the impedance parameters
    k_p_ = get_node()->get_parameter("k_p").as_double_array();
    k_d_ = get_node()->get_parameter("k_d").as_double_array();

    if (k_p_.empty()) {
        k_p_.resize(joint_names_.size(), 50.0);
    }
    if (k_d_.empty()) {
        k_d_.resize(joint_names_.size(), 10.0);
    }

    if (k_p_.size() != static_cast<uint>(num_joints)) {
        RCLCPP_ERROR(get_node()->get_logger(), "k_p should be of size %d but is of size %ld",
            num_joints, k_p_.size());
        return CallbackReturn::FAILURE;
    }
    if (k_d_.size() != static_cast<uint>(num_joints)) {
        RCLCPP_ERROR(get_node()->get_logger(), "k_d should be of size %d but is of size %ld",
            num_joints, k_d_.size());
        return CallbackReturn::FAILURE;
    }

    for (auto i = 0ul; i < k_p_.size(); i++) {
        if (k_p_[i] < 0 || k_d_[i] < 0) {
            RCLCPP_ERROR(get_node()->get_logger(), "Wrong Impedance parameters!");
            return CallbackReturn::FAILURE;
        }
    }

    joints_command_subscriber_
        = get_node()->create_subscription<CmdType>("~/commands", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "Configure successful");

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    auto joint_commands = rt_command_ptr_.readFromRT();

    // no command received yet
    if (!joint_commands || !(*joint_commands)) {
        return controller_interface::return_type::OK;
    }

    // check command size if correct
    if ((*joint_commands)->positions.size() != joint_names_.size()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
            "command size (%zu) does not match number of joints (%zu)",
            (*joint_commands)->positions.size(), joint_names_.size());
        return controller_interface::return_type::ERROR;
    }

    // receive desired joint commands
    for (auto index = 0ul; index < joint_names_.size(); ++index) {
        double q = state_interfaces_[2 * index].get_value();
        double dq = state_interfaces_[2 * index + 1].get_value();
        double q_des = (*joint_commands)->positions[index];

        double dq_des = 0;
        if ((*joint_commands)->velocities.size() == joint_names_.size()) {
            dq_des = (*joint_commands)->velocities[index];
        }

        // compute torque
        double tau = k_p_[index] * (q_des - q) + k_d_[index] * (dq_des - dq);
        command_interfaces_[index].set_value(tau);
    }

    return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    //  check if we have all resources defined in the "points" parameter
    //  also verify that we *only* have the resources defined in the "points"
    //  parameter
    std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, joint_names_, hardware_interface::HW_IF_EFFORT, ordered_interfaces)
        || command_interfaces_.size() != ordered_interfaces.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu position command interfaces, got %zu",
            joint_names_.size(), ordered_interfaces.size());
        return CallbackReturn::ERROR;
    }

    // reset command buffer if a command came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    RCLCPP_INFO(get_node()->get_logger(), "Activate successful");

    return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    return CallbackReturn::SUCCESS;
}

} /* namespace joint_impedance_controller */
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)

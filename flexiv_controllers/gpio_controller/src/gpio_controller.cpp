/**
 * @file gpio_controller.cpp
 * @brief GPIO controller as ROS 2 controller. Adapted from
 * ros2_control_demos/example_10/gpio_controller
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "gpio_controller/gpio_controller.hpp"

#include <string>

namespace gpio_controller {

GPIOController::GPIOController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn GPIOController::on_init()
{
    try {
        initMsgs();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (size_t i = 0; i < 16; ++i) {
        config.names.emplace_back("gpio/digital_output_" + std::to_string(i));
    }

    return config;
}

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (size_t i = 0; i < 16; ++i) {
        config.names.emplace_back("gpio/digital_input_" + std::to_string(i));
    }

    return config;
}

controller_interface::return_type GPIOController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // get inputs
    for (size_t i = 0; i < 16; ++i) {
        gpio_inputs_msg_.states[i].pin = i;
        gpio_inputs_msg_.states[i].state = static_cast<bool>(state_interfaces_[i].get_value());
    }
    gpio_inputs_publisher_->publish(gpio_inputs_msg_);

    // set outputs
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        command_interfaces_[i].set_value(digital_outputs_cmd_[i]);
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GPIOController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    try {
        // register publisher
        gpio_inputs_publisher_
            = get_node()->create_publisher<CmdType>("~/gpio_inputs", rclcpp::SystemDefaultsQoS());

        // register subscriber
        gpio_outputs_command_ = get_node()->create_subscription<CmdType>(
            "~/gpio_outputs", rclcpp::SystemDefaultsQoS(), [this](const CmdType::SharedPtr msg) {
                for (size_t i = 0; i < msg->states.size(); ++i) {
                    if (msg->states[i].pin >= 16) {
                        RCLCPP_WARN(get_node()->get_logger(),
                            "Received command for pin %d, but only pins 0-15 are supported.",
                            msg->states[i].pin);
                        continue;
                    } else {
                        digital_outputs_cmd_[msg->states[i].pin]
                            = static_cast<double>(msg->states[i].state);
                    }
                }
            });
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::initMsgs()
{
    gpio_inputs_msg_.states.resize(digital_outputs_cmd_.size());
    digital_outputs_cmd_.fill(0.0);
}

controller_interface::CallbackReturn GPIOController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    try {
        // reset publisher
        gpio_inputs_publisher_.reset();
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gpio_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gpio_controller::GPIOController, controller_interface::ControllerInterface)

/**
 * @file gpio_controller.hpp
 * @brief GPIO controller as ROS 2 controller. Adapted from
 * ros2_control_demos/example_10/gpio_controller
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef GPIO_CONTROLLER__GPIO_CONTROLLER_HPP_
#define GPIO_CONTROLLER__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "flexiv_msgs/msg/gpio_states.hpp"

namespace gpio_controller {
using CmdType = flexiv_msgs::msg::GPIOStates;

class GPIOController : public controller_interface::ControllerInterface
{
public:
    GPIOController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    void initMsgs();

    // internal commands
    std::array<double, 16> digital_outputs_cmd_;

    // publisher
    std::shared_ptr<rclcpp::Publisher<CmdType>> gpio_inputs_publisher_;
    CmdType gpio_inputs_msg_;

    // subscriber
    rclcpp::Subscription<CmdType>::SharedPtr gpio_outputs_command_;
};

} /* namespace gpio_controller */
#endif /* GPIO_CONTROLLER__GPIO_CONTROLLER_HPP_ */

/**
 * @file joint_impedance_controller.hpp
 * Joint impedance control as ROS2 controller. Adapted from
 * ros2_controllers/forward_command_controller
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.h>
#include "flexiv_msgs/msg/joint_pos_vel.hpp"

namespace flexiv_controllers {
using CmdType = flexiv_msgs::msg::JointPosVel;
using CallbackReturn
    = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JointImpedanceController
: public controller_interface::ControllerInterface
{
public:
    JointImpedanceController();

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::return_type update() override;

    controller_interface::return_type init(
        const std::string& controller_name) override;

    CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::vector<std::string> joint_names_;
    std::vector<double> k_p_;
    std::vector<double> k_d_;
    const int num_joints = 7;

    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
    rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
};

} /* namespace flexiv_controllers */
#endif /* FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_ */

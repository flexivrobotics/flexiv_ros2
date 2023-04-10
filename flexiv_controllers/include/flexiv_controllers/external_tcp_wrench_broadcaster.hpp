/**
 * @file external_tcp_wrench_broadcaster.hpp
 * @brief Controller to publish the estimated external wrench applied on TCP.
 * Adapted from ros2_controllers/force_torque_sensor_broadcaster
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_CONTROLLERS__EXTERNAL_TCP_WRENCH_BROADCASTER_HPP_
#define FLEXIV_CONTROLLERS__EXTERNAL_TCP_WRENCH_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"

namespace flexiv_controllers {
using CallbackReturn
    = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ExternalTcpWrenchBroadcaster
: public controller_interface::ControllerInterface
{
public:
    ExternalTcpWrenchBroadcaster();

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
    std::string sensor_name_;
    std::array<std::string, 6> interface_names_;
    std::string frame_id_;
    std::string topic_name_;

    std::unique_ptr<semantic_components::ForceTorqueSensor>
        force_torque_sensor_;

    using StatePublisher
        = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
        sensor_state_publisher_;
    std::unique_ptr<StatePublisher> realtime_publisher_;
};

} /* namespace flexiv_controllers */

#endif /* FLEXIV_CONTROLLERS__EXTERNAL_TCP_WRENCH_BROADCASTER_HPP_ */

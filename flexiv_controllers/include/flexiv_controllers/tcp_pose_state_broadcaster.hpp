/**
 * @file tcp_pose_state_broadcaster.hpp
 * @brief Controller to publish the the measured TCP pose expressed in base
 * frame.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_CONTROLLERS__TCP_POSE_STATE_BROADCASTER_HPP_
#define FLEXIV_CONTROLLERS__TCP_POSE_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "flexiv_controllers/cartesian_pose_sensor.hpp"

namespace flexiv_controllers {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TcpPoseStateBroadcaster : public controller_interface::ControllerInterface
{
public:
    TcpPoseStateBroadcaster();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::string sensor_name_;
    std::string frame_id_;
    std::string topic_name_;

    std::unique_ptr<CartesianPoseSensor> cartesian_pose_sensor_;

    using StatePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_state_publisher_;
    std::unique_ptr<StatePublisher> realtime_publisher_;
};

} /* namespace flexiv_controllers */

#endif /* FLEXIV_CONTROLLERS__EXTERNAL_TCP_POSE_STATE_BROADCASTER_HPP_ */

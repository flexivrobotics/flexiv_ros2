/**
 * @file flexiv_robot_states_broadcaster.hpp
 * @brief Header file for FlexivRobotStatesBroadcaster class
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_ROBOT_STATES_BROADCASTER__FLEXIV_ROBOT_STATES_BROADCASTER_HPP_
#define FLEXIV_ROBOT_STATES_BROADCASTER__FLEXIV_ROBOT_STATES_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"
#include "flexiv_robot_states_broadcaster/flexiv_robot_states.hpp"
#include "flexiv_robot_states_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace flexiv_robot_states_broadcaster {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

const std::string kRobotStatesTopic = "/flexiv_robot_states";
const std::string kTcpPoseTopic = "~/tcp_pose";
const std::string kTcpPoseDesiredTopic = "~/tcp_pose_desired";
const std::string kTcpVelocityTopic = "~/tcp_velocity";
const std::string kFlangePoseTopic = "~/flange_pose";
const std::string kFTSensorTopic = "~/ft_sensor_wrench";
const std::string kExternalWrenchInTcpFrameTopic = "~/external_wrench_in_tcp";
const std::string kExternalWrenchInWorldFrameTopic = "~/external_wrench_in_world";

class FlexivRobotStatesBroadcaster : public controller_interface::ControllerInterface
{
public:
    FlexivRobotStatesBroadcaster();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::unique_ptr<semantic_components::FlexivRobotStates> flexiv_robot_states_;

    using StatePublisher = realtime_tools::RealtimePublisher<flexiv_msgs::msg::RobotStates>;
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr flexiv_robot_states_publisher_;
    std::unique_ptr<StatePublisher> realtime_flexiv_robot_states_publisher_;

    using PoseStampedPublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;
    std::shared_ptr<PoseStampedPublisher> tcp_pose_publisher_;
    std::shared_ptr<PoseStampedPublisher> tcp_pose_desired_publisher_;
    std::shared_ptr<PoseStampedPublisher> flange_pose_publisher_;

    using AccelStampedPublisher = rclcpp::Publisher<geometry_msgs::msg::AccelStamped>;
    std::shared_ptr<AccelStampedPublisher> tcp_velocity_publisher_;

    using WrenchStampedPublisher = rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>;
    std::shared_ptr<WrenchStampedPublisher> ft_sensor_publisher_;
    std::shared_ptr<WrenchStampedPublisher> external_wrench_in_tcp_publisher_;
    std::shared_ptr<WrenchStampedPublisher> external_wrench_in_world_publisher_;
};

} /* namespace flexiv_robot_states_broadcaster */

#endif /* FLEXIV_ROBOT_STATES_BROADCASTER__FLEXIV_ROBOT_STATES_BROADCASTER_HPP_ */

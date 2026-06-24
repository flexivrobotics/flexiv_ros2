/**
 * @file flexiv_robot_states.hpp
 * @brief Semantic component interface to read the Flexiv robot states.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef SEMANTIC_COMPONENTS__FLEXIV_ROBOT_STATES_HPP_
#define SEMANTIC_COMPONENTS__FLEXIV_ROBOT_STATES_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include "flexiv_hardware/flexiv_robot_states_handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"
#include "flexiv/rdk/data.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace {

const std::string kWorldFrameId = "world";
const std::string kFlangeFrameId = "flange";

} // namespace

namespace semantic_components {

class FlexivRobotStates : public SemanticComponentInterface<flexiv_msgs::msg::RobotStates>
{
public:
    FlexivRobotStates(const std::string& name)
    : SemanticComponentInterface(name, 1)
    {
        interface_names_.emplace_back(name_ + "/" + state_interface_name_);
    }

    virtual ~FlexivRobotStates() = default;

    void init_robot_states_message(flexiv_msgs::msg::RobotStates& message)
    {
        message.tcp_pose.header.frame_id = kWorldFrameId;
        message.tcp_twist.header.frame_id = kWorldFrameId;
        message.flange_pose.header.frame_id = kWorldFrameId;
        message.raw_ft_sensor.header.frame_id = name_ + "_" + kFlangeFrameId;
        message.tcp_wrench_local.header.frame_id = name_ + "_" + kFlangeFrameId;
        message.tcp_wrench.header.frame_id = kWorldFrameId;
        message.raw_tcp_wrench_local.header.frame_id = name_ + "_" + kFlangeFrameId;
        message.raw_tcp_wrench.header.frame_id = kWorldFrameId;
    }

    /// Return RobotStates message
    bool get_values_as_message(flexiv_msgs::msg::RobotStates& message)
    {
        const std::string flexiv_robot_states_interface_name = name_ + "/" + state_interface_name_;

        auto flexiv_robot_states_interface
            = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                [&flexiv_robot_states_interface_name](const auto& state_interface) {
                    return state_interface.get().get_name() == flexiv_robot_states_interface_name;
                });

        if (flexiv_robot_states_interface == state_interfaces_.end()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("FlexivRobotStates"), "Robot states interface not found.");
            return false;
        }

        const auto encoded_handle = (*flexiv_robot_states_interface).get().get_optional();
        if (!encoded_handle.has_value()) {
            RCLCPP_ERROR(rclcpp::get_logger("FlexivRobotStates"),
                "Robot states interface '%s' has no value.",
                flexiv_robot_states_interface_name.c_str());
            return false;
        }

        auto* flexiv_robot_states_ptr
            = flexiv_hardware::resolve_robot_states_handle(encoded_handle.value());
        if (flexiv_robot_states_ptr == nullptr) {
            RCLCPP_ERROR(rclcpp::get_logger("FlexivRobotStates"),
                "Robot states interface '%s' resolved to an invalid handle.",
                flexiv_robot_states_interface_name.c_str());
            return false;
        }

        // Update timestamps
        message.tcp_pose.header.stamp = message.header.stamp;
        message.tcp_twist.header.stamp = message.header.stamp;
        message.flange_pose.header.stamp = message.header.stamp;
        message.raw_ft_sensor.header.stamp = message.header.stamp;
        message.tcp_wrench_local.header.stamp = message.header.stamp;
        message.tcp_wrench.header.stamp = message.header.stamp;
        message.raw_tcp_wrench_local.header.stamp = message.header.stamp;
        message.raw_tcp_wrench.header.stamp = message.header.stamp;

        // Fill the RobotStates message
        message.robot_timestamp.sec = flexiv_robot_states_ptr->timestamp.first;
        message.robot_timestamp.nanosec = flexiv_robot_states_ptr->timestamp.second;

        message.q = flexiv_robot_states_ptr->q;
        message.theta = flexiv_robot_states_ptr->theta;
        message.dq = flexiv_robot_states_ptr->dq;
        message.dtheta = flexiv_robot_states_ptr->dtheta;
        message.tau = flexiv_robot_states_ptr->tau;
        message.tau_dot = flexiv_robot_states_ptr->tau_dot;
        message.tau_ext = flexiv_robot_states_ptr->tau_ext;
        message.tau_interact = flexiv_robot_states_ptr->tau_interact;
        message.temperature = flexiv_robot_states_ptr->temperature;

        message.tcp_pose.pose = toPoseMsg(flexiv_robot_states_ptr->tcp_pose);
        message.tcp_twist.twist = toTwistMsg(flexiv_robot_states_ptr->tcp_twist);
        message.flange_pose.pose = toPoseMsg(flexiv_robot_states_ptr->flange_pose);
        message.raw_ft_sensor.wrench = toWrenchMsg(flexiv_robot_states_ptr->raw_ft_sensor);
        message.tcp_wrench_local.wrench = toWrenchMsg(flexiv_robot_states_ptr->tcp_wrench_local);
        message.tcp_wrench.wrench = toWrenchMsg(flexiv_robot_states_ptr->tcp_wrench);
        message.raw_tcp_wrench_local.wrench
            = toWrenchMsg(flexiv_robot_states_ptr->raw_tcp_wrench_local);
        message.raw_tcp_wrench.wrench = toWrenchMsg(flexiv_robot_states_ptr->raw_tcp_wrench);

        return true;
    }

protected:
    const std::string state_interface_name_ {"flexiv_robot_states"};

    // Convert std::array to geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose toPoseMsg(
        const std::array<double, flexiv::rdk::kPoseSize>& pose_values)
    {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                .x(pose_values[0])
                                .y(pose_values[1])
                                .z(pose_values[2]);
        pose_msg.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                   .x(pose_values[4])
                                   .y(pose_values[5])
                                   .z(pose_values[6])
                                   .w(pose_values[3]);
        return pose_msg;
    }

    // Convert std::array to geometry_msgs::msg::Twist
    geometry_msgs::msg::Twist toTwistMsg(
        const std::array<double, flexiv::rdk::kCartDoF>& twist_values)
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                               .x(twist_values[0])
                               .y(twist_values[1])
                               .z(twist_values[2]);
        twist_msg.angular = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                .x(twist_values[3])
                                .y(twist_values[4])
                                .z(twist_values[5]);
        return twist_msg;
    }

    // Convert std::array to geometry_msgs::msg::Wrench
    geometry_msgs::msg::Wrench toWrenchMsg(
        const std::array<double, flexiv::rdk::kCartDoF>& wrench_values)
    {
        geometry_msgs::msg::Wrench wrench_msg;
        wrench_msg.force = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                               .x(wrench_values[0])
                               .y(wrench_values[1])
                               .z(wrench_values[2]);
        wrench_msg.torque = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                .x(wrench_values[3])
                                .y(wrench_values[4])
                                .z(wrench_values[5]);
        return wrench_msg;
    }
};

} /* namespace semantic_components */

#endif /* SEMANTIC_COMPONENTS__FLEXIV_ROBOT_STATES_HPP_ */

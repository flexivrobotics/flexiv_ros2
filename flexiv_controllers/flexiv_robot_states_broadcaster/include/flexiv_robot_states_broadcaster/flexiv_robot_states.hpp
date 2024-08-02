/**
 * @file flexiv_robot_states.hpp
 * @brief Semantic component interface to read the Flexiv robot states.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef SEMANTIC_COMPONENTS__FLEXIV_ROBOT_STATES_HPP_
#define SEMANTIC_COMPONENTS__FLEXIV_ROBOT_STATES_HPP_

#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"
#include "flexiv/rdk/data.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace {

const std::string kWorldFrameId = "world";
const std::string kFlangeFrameId = "flange";

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value
                     && std::is_trivially_copyable<To>::value,
    To>
bit_cast(const From& src) noexcept
{
    static_assert(std::is_trivially_constructible<To>::value,
        "This implementation additionally requires "
        "destination type to be trivially constructible");

    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}
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
        message.tcp_pose_des.header.frame_id = kWorldFrameId;
        message.tcp_vel.header.frame_id = kWorldFrameId;
        message.flange_pose.header.frame_id = kWorldFrameId;
        message.ft_sensor_raw.header.frame_id = kFlangeFrameId;
        message.ext_wrench_in_tcp.header.frame_id = kFlangeFrameId;
        message.ext_wrench_in_world.header.frame_id = kFlangeFrameId;
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

        if (flexiv_robot_states_interface != state_interfaces_.end()) {
            // Get the robot states pointer via bit_cast
            flexiv_robot_states_ptr = bit_cast<flexiv::rdk::RobotStates*>(
                (*flexiv_robot_states_interface).get().get_value());
        } else {
            RCLCPP_ERROR(
                rclcpp::get_logger("FlexivRobotStates"), "Robot states interface not found.");
            return false;
        }

        // Update timestamps
        message.tcp_pose.header.stamp = message.header.stamp;
        message.tcp_pose_des.header.stamp = message.header.stamp;
        message.tcp_vel.header.stamp = message.header.stamp;
        message.flange_pose.header.stamp = message.header.stamp;
        message.ft_sensor_raw.header.stamp = message.header.stamp;
        message.ext_wrench_in_tcp.header.stamp = message.header.stamp;
        message.ext_wrench_in_world.header.stamp = message.header.stamp;

        // Fill the RobotStates message
        message.q = toJointStateMsg(flexiv_robot_states_ptr->q);
        message.theta = toJointStateMsg(flexiv_robot_states_ptr->theta);
        message.dq = toJointStateMsg(flexiv_robot_states_ptr->dq);
        message.dtheta = toJointStateMsg(flexiv_robot_states_ptr->dtheta);
        message.tau = toJointStateMsg(flexiv_robot_states_ptr->tau);
        message.tau_des = toJointStateMsg(flexiv_robot_states_ptr->tau_des);
        message.tau_dot = toJointStateMsg(flexiv_robot_states_ptr->tau_dot);
        message.tau_ext = toJointStateMsg(flexiv_robot_states_ptr->tau_ext);

        message.tcp_pose.pose = toPoseMsg(flexiv_robot_states_ptr->tcp_pose);
        message.tcp_pose_des.pose = toPoseMsg(flexiv_robot_states_ptr->tcp_pose_des);
        message.tcp_vel.accel = toAccelMsg(flexiv_robot_states_ptr->tcp_vel);
        message.flange_pose.pose = toPoseMsg(flexiv_robot_states_ptr->flange_pose);
        message.ft_sensor_raw.wrench = toWrenchMsg(flexiv_robot_states_ptr->ft_sensor_raw);
        message.ext_wrench_in_tcp.wrench = toWrenchMsg(flexiv_robot_states_ptr->ext_wrench_in_tcp);
        message.ext_wrench_in_world.wrench
            = toWrenchMsg(flexiv_robot_states_ptr->ext_wrench_in_world);

        return true;
    }

protected:
    flexiv::rdk::RobotStates* flexiv_robot_states_ptr;

    const std::string state_interface_name_ {"flexiv_robot_states"};

    // Convert std::vector to std::array
    std::array<double, 7> toJointStateMsg(const std::vector<double>& joint_values)
    {
        std::array<double, 7> joint_state_msg;
        for (size_t i = 0; i < joint_values.size(); ++i) {
            joint_state_msg[i] = joint_values[i];
        }
        return joint_state_msg;
    }

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

    // Convert std::array to geometry_msgs::msg::Accel
    geometry_msgs::msg::Accel toAccelMsg(
        const std::array<double, flexiv::rdk::kCartDoF>& accel_values)
    {
        geometry_msgs::msg::Accel accel_msg;
        accel_msg.linear = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                               .x(accel_values[0])
                               .y(accel_values[1])
                               .z(accel_values[2]);
        accel_msg.angular = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                .x(accel_values[3])
                                .y(accel_values[4])
                                .z(accel_values[5]);
        return accel_msg;
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

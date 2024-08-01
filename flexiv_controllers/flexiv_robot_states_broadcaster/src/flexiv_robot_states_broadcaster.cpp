/**
 * @file flexiv_robot_states_broadcaster.cpp
 * @brief Controller to publish the Flexiv robot states.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#include "flexiv_robot_states_broadcaster/flexiv_robot_states_broadcaster.hpp"

#include <memory>
#include <string>

namespace flexiv_robot_states_broadcaster {

FlexivRobotStatesBroadcaster::FlexivRobotStatesBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
FlexivRobotStatesBroadcaster::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
FlexivRobotStatesBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = flexiv_robot_states_->get_state_interface_names();
    return state_interfaces_config;
}

CallbackReturn FlexivRobotStatesBroadcaster::on_init()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn FlexivRobotStatesBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    std::string robot_sn = params_.robot_sn;
    if (robot_sn.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'robot_sn' parameter has to be specified.");
        return CallbackReturn::ERROR;
    } else {
        // Replace "-" with "_" in robot_sn to match the topic name
        std::replace(robot_sn.begin(), robot_sn.end(), '-', '_');
    }

    if (!flexiv_robot_states_) {
        flexiv_robot_states_ = std::make_unique<semantic_components::FlexivRobotStates>(
            semantic_components::FlexivRobotStates(robot_sn));
    }

    // Create the publishers for the robot states
    tcp_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        kTcpPoseTopic, rclcpp::SystemDefaultsQoS());
    tcp_pose_desired_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        kTcpPoseDesiredTopic, rclcpp::SystemDefaultsQoS());
    tcp_velocity_publisher_ = get_node()->create_publisher<geometry_msgs::msg::AccelStamped>(
        kTcpVelocityTopic, rclcpp::SystemDefaultsQoS());
    flange_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        kFlangePoseTopic, rclcpp::SystemDefaultsQoS());
    ft_sensor_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
        kFTSensorTopic, rclcpp::SystemDefaultsQoS());
    external_wrench_in_tcp_publisher_
        = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            kExternalWrenchInTcpFrameTopic, rclcpp::SystemDefaultsQoS());
    external_wrench_in_world_publisher_
        = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            kExternalWrenchInWorldFrameTopic, rclcpp::SystemDefaultsQoS());

    try {
        flexiv_robot_states_publisher_
            = get_node()->create_publisher<flexiv_msgs::msg::RobotStates>(
                "/" + robot_sn + kRobotStatesTopic, rclcpp::SystemDefaultsQoS());
        realtime_flexiv_robot_states_publisher_
            = std::make_unique<StatePublisher>(flexiv_robot_states_publisher_);
        // Initialize the robot states message
        flexiv_robot_states_->init_robot_states_message(
            realtime_flexiv_robot_states_publisher_->msg_);
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during publisher creation with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FlexivRobotStatesBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    if (realtime_flexiv_robot_states_publisher_
        && realtime_flexiv_robot_states_publisher_->trylock()) {
        realtime_flexiv_robot_states_publisher_->msg_.header.stamp = time;

        if (!flexiv_robot_states_->get_values_as_message(
                realtime_flexiv_robot_states_publisher_->msg_)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Failed to get fleixv robot states via flexiv robot states interface.");
            realtime_flexiv_robot_states_publisher_->unlock();
            return controller_interface::return_type::ERROR;
        }

        realtime_flexiv_robot_states_publisher_->unlockAndPublish();

        const auto& flexiv_robot_states_msg = realtime_flexiv_robot_states_publisher_->msg_;
        tcp_pose_publisher_->publish(flexiv_robot_states_msg.tcp_pose);
        tcp_pose_desired_publisher_->publish(flexiv_robot_states_msg.tcp_pose_des);
        tcp_velocity_publisher_->publish(flexiv_robot_states_msg.tcp_vel);
        flange_pose_publisher_->publish(flexiv_robot_states_msg.flange_pose);
        ft_sensor_publisher_->publish(flexiv_robot_states_msg.ft_sensor_raw);
        external_wrench_in_tcp_publisher_->publish(flexiv_robot_states_msg.ext_wrench_in_tcp);
        external_wrench_in_world_publisher_->publish(flexiv_robot_states_msg.ext_wrench_in_world);

        return controller_interface::return_type::OK;
    } else {
        return controller_interface::return_type::ERROR;
    }
}

CallbackReturn FlexivRobotStatesBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    flexiv_robot_states_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn FlexivRobotStatesBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    flexiv_robot_states_->release_interfaces();
    return CallbackReturn::SUCCESS;
}

} /* namespace flexiv_robot_states_broadcaster */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(flexiv_robot_states_broadcaster::FlexivRobotStatesBroadcaster,
    controller_interface::ControllerInterface)

/**
 * @file tcp_pose_state_broadcaster.cpp
 * @brief Controller to publish the the measured TCP pose expressed in base
 * frame.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "flexiv_controllers/tcp_pose_state_broadcaster.hpp"

#include <memory>
#include <string>

namespace flexiv_controllers {

TcpPoseStateBroadcaster::TcpPoseStateBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
TcpPoseStateBroadcaster::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
TcpPoseStateBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = cartesian_pose_sensor_->get_state_interface_names();
    return state_interfaces_config;
}

CallbackReturn TcpPoseStateBroadcaster::on_init()
{
    try {
        auto_declare<std::string>("sensor_name", "");
        auto_declare<std::string>("frame_id", "");
        auto_declare<std::string>("topic_name", "");
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn TcpPoseStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{

    sensor_name_ = get_node()->get_parameter("sensor_name").as_string();

    if (sensor_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
        return CallbackReturn::ERROR;
    }

    frame_id_ = get_node()->get_parameter("frame_id").as_string();
    if (frame_id_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be specified.");
        return CallbackReturn::ERROR;
    }

    if (!sensor_name_.empty()) {
        cartesian_pose_sensor_
            = std::make_unique<CartesianPoseSensor>(CartesianPoseSensor(sensor_name_));
    }

    topic_name_ = get_node()->get_parameter("topic_name").as_string();
    if (topic_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'topic_name' parameter has to be specified.");
        return CallbackReturn::ERROR;
    }

    try {
        // register TCP pose data publisher
        sensor_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/" + topic_name_, rclcpp::SystemDefaultsQoS());
        realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
    } catch (const std::exception& e) {
        fprintf(stderr,
            "Exception thrown during publisher creation at configure stage "
            "with message : %s \n",
            e.what());
        return CallbackReturn::ERROR;
    }

    realtime_publisher_->lock();
    realtime_publisher_->msg_.header.frame_id = frame_id_;
    realtime_publisher_->unlock();

    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TcpPoseStateBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    if (realtime_publisher_ && realtime_publisher_->trylock()) {
        realtime_publisher_->msg_.header.stamp = time;
        cartesian_pose_sensor_->get_values_as_message(realtime_publisher_->msg_.pose);
        realtime_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

CallbackReturn TcpPoseStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    cartesian_pose_sensor_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn TcpPoseStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    cartesian_pose_sensor_->release_interfaces();
    return CallbackReturn::SUCCESS;
}

} /* namespace flexiv_controllers */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_controllers::TcpPoseStateBroadcaster, controller_interface::ControllerInterface)

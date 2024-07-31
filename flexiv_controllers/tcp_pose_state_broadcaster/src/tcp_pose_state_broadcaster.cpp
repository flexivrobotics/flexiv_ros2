/**
 * @file tcp_pose_state_broadcaster.cpp
 * @brief Controller to publish the the measured TCP pose expressed in world
 * frame.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "tcp_pose_state_broadcaster/tcp_pose_state_broadcaster.hpp"

#include <memory>
#include <string>

namespace tcp_pose_state_broadcaster {

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
    state_interfaces_config.names = cartesian_pose_state_->get_state_interface_names();
    return state_interfaces_config;
}

CallbackReturn TcpPoseStateBroadcaster::on_init()
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

CallbackReturn TcpPoseStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    const bool no_interface_names_defined = params_.interface_names.position.x.empty()
                                            && params_.interface_names.position.y.empty()
                                            && params_.interface_names.position.z.empty()
                                            && params_.interface_names.orientation.x.empty()
                                            && params_.interface_names.orientation.y.empty()
                                            && params_.interface_names.orientation.z.empty()
                                            && params_.interface_names.orientation.w.empty();

    if (params_.sensor_name.empty() && no_interface_names_defined) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "'sensor_name' or at least one "
            "'interface_names.[position|orientation].[x|y|z|w]' parameter has to be specified.");
        return CallbackReturn::ERROR;
    }

    if (!params_.sensor_name.empty() && !no_interface_names_defined) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "both 'sensor_name' and "
            "'interface_names.[position|orientation].[x|y|z|w]' parameters can not be specified "
            "together.");
        return CallbackReturn::ERROR;
    }

    if (!params_.sensor_name.empty()) {
        cartesian_pose_state_ = std::make_unique<semantic_components::CartesianPoseState>(
            semantic_components::CartesianPoseState(params_.sensor_name));
    } else {
        auto const& position_names = params_.interface_names.position;
        auto const& orientation_names = params_.interface_names.orientation;
        cartesian_pose_state_ = std::make_unique<semantic_components::CartesianPoseState>(
            semantic_components::CartesianPoseState(position_names.x, position_names.y,
                position_names.z, orientation_names.x, orientation_names.y, orientation_names.z,
                orientation_names.w));
    }

    try {
        // register TCP pose data publisher
        sensor_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/tcp_pose", rclcpp::SystemDefaultsQoS());
        realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
    } catch (const std::exception& e) {
        fprintf(stderr,
            "Exception thrown during publisher creation at configure stage "
            "with message : %s \n",
            e.what());
        return CallbackReturn::ERROR;
    }

    realtime_publisher_->lock();
    realtime_publisher_->msg_.header.frame_id = params_.frame_id;
    realtime_publisher_->unlock();

    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TcpPoseStateBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    if (realtime_publisher_ && realtime_publisher_->trylock()) {
        realtime_publisher_->msg_.header.stamp = time;
        cartesian_pose_state_->get_values_as_message(realtime_publisher_->msg_.pose);
        realtime_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

CallbackReturn TcpPoseStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    cartesian_pose_state_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn TcpPoseStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    cartesian_pose_state_->release_interfaces();
    return CallbackReturn::SUCCESS;
}

} /* namespace tcp_pose_state_broadcaster */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    tcp_pose_state_broadcaster::TcpPoseStateBroadcaster, controller_interface::ControllerInterface)

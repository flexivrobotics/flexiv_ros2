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

    try {
        flexiv_robot_states_publisher_
            = get_node()->create_publisher<flexiv_msgs::msg::RobotStates>(
                "/" + robot_sn + "/flexiv_robot_states", rclcpp::SystemDefaultsQoS());
        realtime_publisher_ = std::make_unique<StatePublisher>(flexiv_robot_states_publisher_);
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
    if (realtime_publisher_ && realtime_publisher_->trylock()) {
        realtime_publisher_->msg_.header.stamp = time;

        if (!flexiv_robot_states_->get_values_as_message(realtime_publisher_->msg_)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Failed to get fleixv robot states via flexiv robot states interface.");
            realtime_publisher_->unlock();
            return controller_interface::return_type::ERROR;
        }

        realtime_publisher_->unlockAndPublish();

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

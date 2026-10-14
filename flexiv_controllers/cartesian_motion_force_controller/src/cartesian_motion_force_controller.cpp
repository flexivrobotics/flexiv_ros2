/**
 * @file cartesian_motion_force_controller.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "cartesian_motion_force_controller/cartesian_motion_force_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace {

constexpr double kMinQuaternionNorm = 1e-6;
constexpr int kWarnThrottleMs = 1000;

}

namespace cartesian_motion_force_controller {

bool ToCommand(const CmdType& msg, Command& command, std::string& error)
{
    const auto& p = msg.pose.position;
    const auto& q = msg.pose.orientation;
    const auto& f = msg.wrench.force;
    const auto& m = msg.wrench.torque;
    const auto& v = msg.velocity.linear;
    const auto& w = msg.velocity.angular;
    Command converted = {p.x, p.y, p.z, q.w, q.x, q.y, q.z, f.x, f.y, f.z, m.x, m.y, m.z, v.x, v.y,
        v.z, w.x, w.y, w.z};

    if (!std::all_of(
            converted.begin(), converted.end(), [](double x) { return std::isfinite(x); })) {
        error = "a pose, wrench or velocity field is not finite";
        return false;
    }
    // No fallback to identity: that would command an arbitrary, possibly large rotation.
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < kMinQuaternionNorm) {
        error = "the orientation quaternion has zero norm";
        return false;
    }
    for (size_t i = 3; i < kPoseSize; ++i) {
        converted[i] /= norm;
    }

    command = converted;
    return true;
}

controller_interface::CallbackReturn CartesianMotionForceController::on_init()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianMotionForceController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& prefix : params_.prefixes) {
        for (const auto& names : {kPoseInterfaces, kWrenchInterfaces, kVelocityInterfaces}) {
            for (const auto& name : names) {
                config.names.push_back(prefix + "tcp/" + name);
            }
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
CartesianMotionForceController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& prefix : params_.prefixes) {
        for (const auto& name : kPoseInterfaces) {
            config.names.push_back(prefix + "tcp/" + name);
        }
    }
    return config;
}

controller_interface::CallbackReturn CartesianMotionForceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    commands_.clear();
    subscriptions_.clear();

    // A robot pair gets one topic per robot, under its prefix
    const bool single_robot = params_.prefixes.size() == 1;
    for (size_t robot = 0; robot < params_.prefixes.size(); ++robot) {
        std::string robot_ns = params_.prefixes[robot];
        if (!robot_ns.empty() && robot_ns.back() == '_') {
            robot_ns.pop_back();
        }
        std::replace(robot_ns.begin(), robot_ns.end(), '-', '_');
        const std::string topic = single_robot ? "~/cartesian_motion_force"
                                               : "~/" + robot_ns + "/cartesian_motion_force";

        commands_.push_back(std::make_unique<realtime_tools::RealtimeBuffer<Command>>());
        subscriptions_.push_back(get_node()->create_subscription<CmdType>(
            topic, rclcpp::SystemDefaultsQoS(), [this, robot](const CmdType::SharedPtr msg) {
                Command command;
                std::string error;
                if (!ToCommand(*msg, command, error)) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                        kWarnThrottleMs, "Dropped a Cartesian motion-force command: %s",
                        error.c_str());
                    return;
                }
                commands_[robot]->writeFromNonRT(command);
            }));
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianMotionForceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // Start by holding the measured TCP pose, discarding anything received while inactive.
    for (size_t robot = 0; robot < commands_.size(); ++robot) {
        Command hold {};
        for (size_t i = 0; i < kPoseSize; ++i) {
            const auto value = state_interfaces_[robot * kPoseSize + i].get_optional();
            if (!value) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not read the TCP pose of '%s'",
                    params_.prefixes[robot].c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            hold[i] = *value;
        }
        commands_[robot]->writeFromNonRT(hold);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMotionForceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    for (size_t robot = 0; robot < commands_.size(); ++robot) {
        const Command& command = *commands_[robot]->readFromRT();
        for (size_t i = 0; i < kCommandSize; ++i) {
            if (!command_interfaces_[robot * kCommandSize + i].set_value(command[i])) {
                return controller_interface::return_type::ERROR;
            }
        }
    }
    return controller_interface::return_type::OK;
}

} /* namespace cartesian_motion_force_controller */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cartesian_motion_force_controller::CartesianMotionForceController,
    controller_interface::ControllerInterface)

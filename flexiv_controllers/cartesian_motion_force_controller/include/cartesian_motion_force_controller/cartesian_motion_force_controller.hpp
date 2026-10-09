/**
 * @file cartesian_motion_force_controller.hpp
 * @brief Controller forwarding Cartesian motion-force commands to the Flexiv hardware interface.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef CARTESIAN_MOTION_FORCE_CONTROLLER__CARTESIAN_MOTION_FORCE_CONTROLLER_HPP_
#define CARTESIAN_MOTION_FORCE_CONTROLLER__CARTESIAN_MOTION_FORCE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "cartesian_motion_force_controller/cartesian_motion_force_controller_parameters.hpp"
#include "flexiv_msgs/msg/cartesian_motion_force.hpp"

namespace cartesian_motion_force_controller {

/** Interfaces of the "<prefix>tcp" component, in RDK order: pose, wrench, velocity. */
const std::vector<std::string> kPoseInterfaces
    = {"cartesian_pose_x", "cartesian_pose_y", "cartesian_pose_z", "cartesian_pose_qw",
        "cartesian_pose_qx", "cartesian_pose_qy", "cartesian_pose_qz"};
const std::vector<std::string> kWrenchInterfaces = {"cartesian_wrench_fx", "cartesian_wrench_fy",
    "cartesian_wrench_fz", "cartesian_wrench_mx", "cartesian_wrench_my", "cartesian_wrench_mz"};
const std::vector<std::string> kVelocityInterfaces
    = {"cartesian_velocity_vx", "cartesian_velocity_vy", "cartesian_velocity_vz",
        "cartesian_velocity_wx", "cartesian_velocity_wy", "cartesian_velocity_wz"};

constexpr size_t kPoseSize = 7;
constexpr size_t kCommandSize = 19;

/** One robot's command, in the order of the command interfaces. */
using Command = std::array<double, kCommandSize>;

using CmdType = flexiv_msgs::msg::CartesianMotionForce;

/**
 * @brief [Non-blocking] Convert a message into a command, normalizing the quaternion.
 * @return False if any field is not finite or the quaternion has zero norm, with [error] saying
 * which. [command] is then left unchanged.
 */
bool ToCommand(const CmdType& msg, Command& command, std::string& error);

/**
 * @brief Forwards the latest valid command of each robot to its command interfaces, holding it
 * until a new one arrives.
 */
class CartesianMotionForceController : public controller_interface::ControllerInterface
{
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // One per robot, in the order of params_.prefixes
    std::vector<std::unique_ptr<realtime_tools::RealtimeBuffer<Command>>> commands_;
    std::vector<rclcpp::Subscription<CmdType>::SharedPtr> subscriptions_;
};

} /* namespace cartesian_motion_force_controller */

#endif /* CARTESIAN_MOTION_FORCE_CONTROLLER__CARTESIAN_MOTION_FORCE_CONTROLLER_HPP_ */

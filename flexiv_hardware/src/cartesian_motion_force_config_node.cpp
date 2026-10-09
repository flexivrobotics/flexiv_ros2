/**
 * @file cartesian_motion_force_config_node.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "flexiv_hardware/cartesian_motion_force_config_node.hpp"

namespace {

using flexiv::rdk::kCartDoF;
using flexiv::rdk::kPoseSize;

constexpr double kMinCartesianDampingRatio = 0.3;
constexpr double kMaxCartesianDampingRatio = 0.8;
constexpr double kMinForceControlLinearVel = 0.005;
constexpr double kMaxForceControlLinearVel = 2.0;
constexpr double kDefaultForceControlLinearVel = 1.0;
constexpr double kMinQuaternionNorm = 1e-6;

bool InCartesianMode(flexiv::rdk::Mode mode)
{
    return mode == flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE
           || mode == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE;
}

/** @return An error if [values] does not hold [expected] entries, otherwise empty. */
std::string CheckLength(size_t actual, size_t expected, const std::string& field)
{
    if (actual == expected) {
        return "";
    }
    return "'" + field + "' has " + std::to_string(actual) + " values, expected "
           + std::to_string(expected);
}

/**
 * @return An error naming the first entry that is NaN, infinite (unless [allow_infinity]) or
 * outside [lower, upper[i]], otherwise empty.
 */
std::string CheckRange(const std::vector<double>& values, double lower,
    const std::vector<double>& upper, const std::string& field, bool allow_infinity = false)
{
    for (size_t i = 0; i < values.size(); ++i) {
        // Checked before the range comparison, which a NaN would pass by being false both ways.
        if (std::isnan(values[i]) || (!allow_infinity && std::isinf(values[i]))) {
            return "'" + field + "[" + std::to_string(i) + "]' is not a finite value";
        }
        if (values[i] < lower || values[i] > upper[i]) {
            std::ostringstream stream;
            stream << "'" << field << "[" << i << "]' is " << values[i]
                   << ", outside the valid range [" << lower << ", " << upper[i] << "]";
            return stream.str();
        }
    }
    return "";
}

std::string CheckRange(const std::vector<double>& values, double lower, double upper,
    const std::string& field, bool allow_infinity = false)
{
    return CheckRange(
        values, lower, std::vector<double>(values.size(), upper), field, allow_infinity);
}

/** @return An error naming the first entry that is not finite and positive, otherwise empty. */
std::string CheckPositive(const std::vector<double>& values, const std::string& field)
{
    for (size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i]) || values[i] <= 0.0) {
            return "'" + field + "[" + std::to_string(i) + "]' must be finite and positive";
        }
    }
    return "";
}

/** @brief Split a flat per-robot array into one fixed-size array per robot. */
template <size_t N, typename T>
std::vector<std::array<T, N>> Split(const std::vector<T>& flat)
{
    std::vector<std::array<T, N>> split(flat.size() / N);
    for (size_t i = 0; i < flat.size(); ++i) {
        split[i / N][i % N] = flat[i];
    }
    return split;
}

template <size_t N>
std::vector<double> Flatten(const std::vector<std::array<double, N>>& split)
{
    std::vector<double> flat;
    for (const auto& values : split) {
        flat.insert(flat.end(), values.begin(), values.end());
    }
    return flat;
}

}

namespace flexiv_hardware {

std::vector<std::string> CartesianCommandInterfaceNames(const std::string& prefix)
{
    std::vector<std::string> names;
    for (const auto* name : kCartesianPoseInterfaces) {
        names.push_back(prefix + "tcp/" + name);
    }
    for (const auto* name : kCartesianWrenchInterfaces) {
        names.push_back(prefix + "tcp/" + name);
    }
    for (const auto* name : kCartesianVelocityInterfaces) {
        names.push_back(prefix + "tcp/" + name);
    }
    return names;
}

CartesianMotionForceConfigNode::CartesianMotionForceConfigNode(const std::string& robot_sn,
    std::vector<std::string> joint_names, CartesianMotionForceBounds bounds,
    std::shared_ptr<DriverStatus> status, CartesianMotionForceSetters setters)
: rclcpp::Node("flexiv_cartesian_motion_force_config_node", SanitizeNamespace(robot_sn))
, joint_names_(std::move(joint_names))
, bounds_(std::move(bounds))
, status_(std::move(status))
, setters_(std::move(setters))
{
    // One group, so two blocking RDK calls are never in flight at once
    service_callback_group_
        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    using std::placeholders::_1;
    using std::placeholders::_2;
    const auto& qos = rmw_qos_profile_services_default;
    set_cartesian_impedance_service_
        = this->create_service<SetCartesianImpedance>("~/set_cartesian_impedance",
            std::bind(&CartesianMotionForceConfigNode::HandleSetCartesianImpedance, this, _1, _2),
            qos, service_callback_group_);
    set_cartesian_motion_limits_service_ = this->create_service<SetCartesianMotionLimits>(
        "~/set_cartesian_motion_limits",
        std::bind(&CartesianMotionForceConfigNode::HandleSetCartesianMotionLimits, this, _1, _2),
        qos, service_callback_group_);
    set_force_control_axis_service_
        = this->create_service<SetForceControlAxis>("~/set_force_control_axis",
            std::bind(&CartesianMotionForceConfigNode::HandleSetForceControlAxis, this, _1, _2),
            qos, service_callback_group_);
    set_force_control_frame_service_
        = this->create_service<SetForceControlFrame>("~/set_force_control_frame",
            std::bind(&CartesianMotionForceConfigNode::HandleSetForceControlFrame, this, _1, _2),
            qos, service_callback_group_);
    set_max_contact_wrench_service_
        = this->create_service<SetMaxContactWrench>("~/set_max_contact_wrench",
            std::bind(&CartesianMotionForceConfigNode::HandleSetMaxContactWrench, this, _1, _2),
            qos, service_callback_group_);
    set_null_space_objectives_service_
        = this->create_service<SetNullSpaceObjectives>("~/set_null_space_objectives",
            std::bind(&CartesianMotionForceConfigNode::HandleSetNullSpaceObjectives, this, _1, _2),
            qos, service_callback_group_);
    set_null_space_posture_service_
        = this->create_service<SetNullSpacePosture>("~/set_null_space_posture",
            std::bind(&CartesianMotionForceConfigNode::HandleSetNullSpacePosture, this, _1, _2),
            qos, service_callback_group_);
    set_passive_force_control_service_
        = this->create_service<SetPassiveForceControl>("~/set_passive_force_control",
            std::bind(&CartesianMotionForceConfigNode::HandleSetPassiveForceControl, this, _1, _2),
            qos, service_callback_group_);

    RCLCPP_INFO(this->get_logger(), "Cartesian motion-force interface ready: services under '%s'",
        this->get_fully_qualified_name());
}

CartesianMotionForceConfigNode::~CartesianMotionForceConfigNode() = default;

bool CartesianMotionForceConfigNode::CheckPreconditions(bool idle_only, std::string& message) const
{
    const auto condition = status_->condition();
    if (!condition.connected) {
        message = "Not connected to the robot.";
        return false;
    }
    if (status_->driver_state.load() != DriverState::READY) {
        message = "The robot is not ready. " + DescribeRobotCondition(condition)
                  + " Set it once the robot is ready again.";
        return false;
    }
    if (status_->reduced.load() || status_->recovery_state.load()) {
        message = "The robot is in a reduced or recovery state. "
                  + DescribeRobotCondition(condition) + " Set it once it has left that state.";
        return false;
    }

    const auto mode = status_->control_mode.load();
    if (idle_only && mode != flexiv::rdk::Mode::IDLE) {
        message
            = "Only accepted while the robot is idle, i.e. before the Cartesian motion-force "
              "controller is started. The robot is in "
              + ControlModeName(mode) + " control mode.";
        return false;
    }
    if (!idle_only && !InCartesianMode(mode)) {
        message
            = "Only accepted while the Cartesian motion-force controller is running. The "
              "robot is in "
              + ControlModeName(mode) + " control mode.";
        return false;
    }
    return true;
}

void CartesianMotionForceConfigNode::Serve(const std::string& property, const std::string& error,
    bool idle_only, const std::function<void()>& deliver, bool& success, std::string& message)
{
    success = false;
    if (!error.empty()) {
        message = error;
    } else if (CheckPreconditions(idle_only, message)) {
        try {
            deliver();
            success = true;
            message = "The " + property + " is applied.";
        } catch (const std::exception& e) {
            message = "The robot rejected the " + property + ": " + e.what();
        }
    }

    if (success) {
        RCLCPP_WARN(this->get_logger(), "The %s is applied", property.c_str());
    } else {
        RCLCPP_WARN(
            this->get_logger(), "Refused %s request: %s", property.c_str(), message.c_str());
    }
}

bool CartesianMotionForceConfigNode::DisablePassiveForceControl()
{
    if (!passive_force_control_enabled_.load()) {
        return true;
    }
    try {
        setters_.set_passive_force_control(std::vector<bool>(num_robots(), false));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            this->get_logger(), "Could not disable the passive force control: %s", e.what());
        return false;
    }
    passive_force_control_enabled_.store(false);
    RCLCPP_WARN(this->get_logger(), "Passive force control disabled");
    return true;
}

void CartesianMotionForceConfigNode::HandleSetCartesianImpedance(
    const std::shared_ptr<SetCartesianImpedance::Request> request,
    std::shared_ptr<SetCartesianImpedance::Response> response)
{
    const size_t size = num_robots() * kCartDoF;
    const auto k_x_nom = Flatten(bounds_.k_x_nom);
    // Empty means nominal, matching RDK usage SetCartesianImpedance(robot.info().K_x_nom).
    const auto k_x_flat = request->k_x.empty() ? k_x_nom : request->k_x;
    const auto z_x_flat = request->z_x.empty()
                              ? std::vector<double>(size, kNominalCartesianDampingRatio)
                              : request->z_x;

    auto error = CheckLength(k_x_flat.size(), size, "k_x");
    if (error.empty()) {
        error = CheckLength(z_x_flat.size(), size, "z_x");
    }
    if (error.empty()) {
        error = CheckRange(k_x_flat, 0.0, k_x_nom, "k_x");
    }
    if (error.empty()) {
        error = CheckRange(z_x_flat, kMinCartesianDampingRatio, kMaxCartesianDampingRatio, "z_x");
    }

    std::vector<CartesianArray> k_x;
    std::vector<CartesianArray> z_x;
    if (error.empty()) {
        k_x = Split<kCartDoF>(k_x_flat);
        z_x = Split<kCartDoF>(z_x_flat);
    }
    Serve(
        "Cartesian impedance", error, false, [&]() { setters_.set_cartesian_impedance(k_x, z_x); },
        response->success, response->message);
    response->k_x_nom = k_x_nom;
}

void CartesianMotionForceConfigNode::HandleSetCartesianMotionLimits(
    const std::shared_ptr<SetCartesianMotionLimits::Request> request,
    std::shared_ptr<SetCartesianMotionLimits::Response> response)
{
    std::string error;
    for (const auto& [values, field] : {std::make_pair(&request->max_linear_vel, "max_linear_vel"),
             std::make_pair(&request->max_angular_vel, "max_angular_vel"),
             std::make_pair(&request->max_linear_acc, "max_linear_acc"),
             std::make_pair(&request->max_angular_acc, "max_angular_acc")}) {
        if (error.empty()) {
            error = CheckLength(values->size(), num_robots(), field);
        }
        if (error.empty()) {
            error = CheckPositive(*values, field);
        }
    }
    std::vector<CartesianMotionLimits> limits;
    for (size_t i = 0; error.empty() && i < num_robots(); ++i) {
        limits.push_back({request->max_linear_vel[i], request->max_angular_vel[i],
            request->max_linear_acc[i], request->max_angular_acc[i]});
    }
    Serve(
        "Cartesian motion limits", error, false, [&]() { setters_.set_motion_limits(limits); },
        response->success, response->message);
}

void CartesianMotionForceConfigNode::HandleSetForceControlAxis(
    const std::shared_ptr<SetForceControlAxis::Request> request,
    std::shared_ptr<SetForceControlAxis::Response> response)
{
    const size_t linear_size = num_robots() * (kCartDoF / 2);
    const auto max_linear_vel_flat = request->max_linear_vel.empty() ? std::vector<double>(
                                         linear_size, kDefaultForceControlLinearVel)
                                                                     : request->max_linear_vel;

    auto error = CheckLength(request->enabled_axes.size(), num_robots() * kCartDoF, "enabled_axes");
    if (error.empty()) {
        error = CheckLength(max_linear_vel_flat.size(), linear_size, "max_linear_vel");
    }
    if (error.empty()) {
        error = CheckRange(max_linear_vel_flat, kMinForceControlLinearVel,
            kMaxForceControlLinearVel, "max_linear_vel");
    }

    std::vector<CartesianFlags> enabled_axes;
    std::vector<LinearArray> max_linear_vel;
    if (error.empty()) {
        enabled_axes = Split<kCartDoF>(std::vector<bool>(request->enabled_axes));
        max_linear_vel = Split<kCartDoF / 2>(max_linear_vel_flat);
    }
    Serve(
        "force control axis", error, false,
        [&]() { setters_.set_force_control_axis(enabled_axes, max_linear_vel); }, response->success,
        response->message);
}

void CartesianMotionForceConfigNode::HandleSetForceControlFrame(
    const std::shared_ptr<SetForceControlFrame::Request> request,
    std::shared_ptr<SetForceControlFrame::Response> response)
{
    auto t_in_root_flat = request->t_in_root;
    if (t_in_root_flat.empty()) {
        for (size_t i = 0; i < num_robots(); ++i) {
            t_in_root_flat.insert(t_in_root_flat.end(), {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
        }
    }

    auto error = CheckLength(request->root_coord.size(), num_robots(), "root_coord");
    if (error.empty()) {
        error = CheckLength(t_in_root_flat.size(), num_robots() * kPoseSize, "t_in_root");
    }
    if (error.empty()) {
        error = CheckRange(t_in_root_flat, -std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(), "t_in_root");
    }
    std::vector<flexiv::rdk::CoordType> root_coord;
    for (size_t i = 0; error.empty() && i < request->root_coord.size(); ++i) {
        if (request->root_coord[i] == SetForceControlFrame::Request::WORLD) {
            root_coord.push_back(flexiv::rdk::CoordType::WORLD);
        } else if (request->root_coord[i] == SetForceControlFrame::Request::TCP) {
            root_coord.push_back(flexiv::rdk::CoordType::TCP);
        } else {
            error = "'root_coord[" + std::to_string(i) + "]' must be WORLD (0) or TCP (1)";
        }
    }

    std::vector<PoseArray> t_in_root;
    if (error.empty()) {
        t_in_root = Split<kPoseSize>(t_in_root_flat);
        for (size_t i = 0; error.empty() && i < t_in_root.size(); ++i) {
            auto& pose = t_in_root[i];
            const double norm = std::sqrt(
                pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5] + pose[6] * pose[6]);
            if (norm < kMinQuaternionNorm) {
                error = "The quaternion of 't_in_root' for robot " + std::to_string(i)
                        + " has zero norm";
            }
            for (size_t j = 3; error.empty() && j < kPoseSize; ++j) {
                pose[j] /= norm;
            }
        }
    }
    Serve(
        "force control frame", error, false,
        [&]() { setters_.set_force_control_frame(root_coord, t_in_root); }, response->success,
        response->message);
}

void CartesianMotionForceConfigNode::HandleSetMaxContactWrench(
    const std::shared_ptr<SetMaxContactWrench::Request> request,
    std::shared_ptr<SetMaxContactWrench::Response> response)
{
    auto error = CheckLength(request->max_wrench.size(), num_robots() * kCartDoF, "max_wrench");
    if (error.empty()) {
        // Infinity is the RDK's way of disabling the regulation.
        error = CheckRange(
            request->max_wrench, 0.0, std::numeric_limits<double>::infinity(), "max_wrench", true);
    }

    std::vector<CartesianArray> max_wrench;
    if (error.empty()) {
        max_wrench = Split<kCartDoF>(request->max_wrench);
    }
    Serve(
        "maximum contact wrench", error, false,
        [&]() { setters_.set_max_contact_wrench(max_wrench); }, response->success,
        response->message);
}

void CartesianMotionForceConfigNode::HandleSetNullSpaceObjectives(
    const std::shared_ptr<SetNullSpaceObjectives::Request> request,
    std::shared_ptr<SetNullSpaceObjectives::Response> response)
{
    std::string error;
    for (const auto& [values, field, lower] :
        {std::make_tuple(&request->linear_manipulability, "linear_manipulability", 0.0),
            std::make_tuple(&request->angular_manipulability, "angular_manipulability", 0.0),
            std::make_tuple(&request->ref_positions_tracking, "ref_positions_tracking", 0.1)}) {
        if (error.empty()) {
            error = CheckLength(values->size(), num_robots(), field);
        }
        if (error.empty()) {
            error = CheckRange(*values, lower, 1.0, field);
        }
    }

    std::vector<NullSpaceObjectives> objectives;
    for (size_t i = 0; error.empty() && i < num_robots(); ++i) {
        objectives.push_back({request->linear_manipulability[i], request->angular_manipulability[i],
            request->ref_positions_tracking[i]});
    }
    Serve(
        "null-space objectives", error, false,
        [&]() { setters_.set_null_space_objectives(objectives); }, response->success,
        response->message);
}

void CartesianMotionForceConfigNode::HandleSetNullSpacePosture(
    const std::shared_ptr<SetNullSpacePosture::Request> request,
    std::shared_ptr<SetNullSpacePosture::Response> response)
{
    auto error = CheckLength(request->ref_positions.size(), joint_names_.size(), "ref_positions");
    for (size_t i = 0; error.empty() && i < request->ref_positions.size(); ++i) {
        const double value = request->ref_positions[i];
        if (!std::isfinite(value) || value < bounds_.q_min[i] || value > bounds_.q_max[i]) {
            std::ostringstream stream;
            stream << "'ref_positions' for joint '" << joint_names_[i] << "' is " << value
                   << ", outside the valid range [" << bounds_.q_min[i] << ", " << bounds_.q_max[i]
                   << "]";
            error = stream.str();
        }
    }

    const auto& ref_positions = request->ref_positions;
    Serve(
        "null-space posture", error, false,
        [&]() { setters_.set_null_space_posture(ref_positions); }, response->success,
        response->message);
}

void CartesianMotionForceConfigNode::HandleSetPassiveForceControl(
    const std::shared_ptr<SetPassiveForceControl::Request> request,
    std::shared_ptr<SetPassiveForceControl::Response> response)
{
    const auto error = CheckLength(request->enabled.size(), num_robots(), "enabled");
    const std::vector<bool> enabled(request->enabled.begin(), request->enabled.end());
    Serve(
        "passive force control", error, true,
        [&]() {
            setters_.set_passive_force_control(enabled);
            passive_force_control_enabled_.store(
                std::find(enabled.begin(), enabled.end(), true) != enabled.end());
        },
        response->success, response->message);
}

} /* namespace flexiv_hardware */

/**
 * @file joint_impedance_config_node.cpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

#include "flexiv_hardware/joint_impedance_config_node.hpp"

namespace {

/** @brief Format a vector for a log line, trimmed to keep the message readable. */
std::string Vec2Str(const std::vector<double>& values)
{
    std::ostringstream stream;
    stream.precision(1);
    stream << std::fixed << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        stream << (i > 0 ? ", " : "") << values[i];
    }
    stream << "]";
    return stream.str();
}

/** @brief Pick out the entries [indices] names, in that order. */
std::vector<double> GatherByIndex(
    const std::vector<double>& values, const std::vector<size_t>& indices)
{
    std::vector<double> gathered;
    gathered.reserve(indices.size());
    for (const auto index : indices) {
        gathered.push_back(values[index]);
    }
    return gathered;
}

/** @brief Pick out the names [indices] names, in that order. */
std::vector<std::string> GatherNamesByIndex(
    const std::vector<std::string>& names, const std::vector<size_t>& indices)
{
    std::vector<std::string> gathered;
    gathered.reserve(indices.size());
    for (const auto index : indices) {
        gathered.push_back(names[index]);
    }
    return gathered;
}

}

namespace flexiv_hardware {

//======================================== JOINT ORDERING ==========================================

std::vector<double> ConvertROSToRDKOrder(
    const std::vector<double>& ros_values, const std::vector<size_t>& rdk_to_ros_map)
{
    std::vector<double> rdk_values(rdk_to_ros_map.size());
    for (size_t rdk_idx = 0; rdk_idx < rdk_to_ros_map.size(); ++rdk_idx) {
        const size_t ros_idx = rdk_to_ros_map[rdk_idx];
        if (ros_idx >= ros_values.size()) {
            throw std::invalid_argument(
                "ConvertROSToRDKOrder: joint index out of range for the provided values");
        }
        rdk_values[rdk_idx] = ros_values[ros_idx];
    }
    return rdk_values;
}

std::vector<double> ConvertRDKToROSOrder(
    const std::vector<double>& rdk_values, const std::vector<size_t>& rdk_to_ros_map)
{
    std::vector<double> ros_values(rdk_to_ros_map.size(), 0.0);
    for (size_t rdk_idx = 0; rdk_idx < rdk_to_ros_map.size(); ++rdk_idx) {
        if (rdk_idx >= rdk_values.size()) {
            throw std::invalid_argument(
                "ConvertRDKToROSOrder: RDK index out of range for the provided values");
        }
        const size_t ros_idx = rdk_to_ros_map[rdk_idx];
        if (ros_idx >= ros_values.size()) {
            throw std::invalid_argument("ConvertRDKToROSOrder: joint index out of range");
        }
        ros_values[ros_idx] = rdk_values[rdk_idx];
    }
    return ros_values;
}

//========================================= JOINT SELECTION ========================================

bool ResolveJointSelection(const std::vector<std::string>& requested_names,
    const std::vector<std::string>& joint_names, std::vector<size_t>& indices,
    std::string& message)
{
    indices.clear();

    // An empty request addresses every covered joint, which is the whole-robot form the interface
    // started with and stays the default.
    if (requested_names.empty()) {
        indices.resize(joint_names.size());
        std::iota(indices.begin(), indices.end(), size_t {0});
        return true;
    }

    std::set<size_t> already_named;
    indices.reserve(requested_names.size());
    for (const auto& requested : requested_names) {
        const auto it = std::find(joint_names.begin(), joint_names.end(), requested);
        if (it == joint_names.end()) {
            message = "'" + requested
                      + "' is not a joint this interface covers. The covered joints are published "
                        "as 'joint_names' on the matching topic";
            return false;
        }
        const auto index = static_cast<size_t>(std::distance(joint_names.begin(), it));
        if (!already_named.insert(index).second) {
            message = "joint '" + requested + "' is named more than once";
            return false;
        }
        indices.push_back(index);
    }
    return true;
}

//========================================== VALIDATION ============================================

namespace {

/** @brief Shared body of the two ValidateJointValues() overloads. */
bool ValidateAgainstBounds(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower,
    const std::vector<double>& upper_per_joint, const std::string& property, std::string& message)
{
    if (values.size() != joint_names.size()) {
        message = "'" + property + "' has " + std::to_string(values.size()) + " values, expected "
                  + std::to_string(joint_names.size()) + ", one per joint in URDF order";
        return false;
    }

    for (size_t i = 0; i < values.size(); ++i) {
        // Checked before the range comparison, which a NaN would pass by being false both ways.
        if (!std::isfinite(values[i])) {
            message = "'" + property + "' for joint '" + joint_names[i] + "' is not a finite value";
            return false;
        }
        if (values[i] < lower || values[i] > upper_per_joint[i]) {
            std::ostringstream stream;
            stream.precision(3);
            stream << std::fixed << "'" << property << "' for joint '" << joint_names[i] << "' is "
                   << values[i] << ", outside the valid range [" << lower << ", "
                   << upper_per_joint[i] << "]";
            message = stream.str();
            return false;
        }
    }
    return true;
}

}

bool ValidateJointValues(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower, const std::vector<double>& upper,
    const std::string& property, std::string& message)
{
    if (upper.size() != joint_names.size()) {
        message = "internal error: bounds for '" + property + "' do not cover every joint";
        return false;
    }
    return ValidateAgainstBounds(values, joint_names, lower, upper, property, message);
}

bool ValidateJointValues(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower, double upper,
    const std::string& property, std::string& message)
{
    return ValidateAgainstBounds(values, joint_names, lower,
        std::vector<double>(joint_names.size(), upper), property, message);
}

//========================================== THE NODE ==============================================

JointImpedanceConfigNode::JointImpedanceConfigNode(const std::string& robot_sn,
    std::vector<std::string> joint_names, JointImpedanceBounds bounds,
    bool impedance_mode_configured, std::shared_ptr<DriverStatus> status,
    JointImpedanceSetters setters)
: rclcpp::Node("flexiv_joint_impedance_config_node", SanitizeNamespace(robot_sn))
, joint_names_(std::move(joint_names))
, bounds_(std::move(bounds))
, impedance_mode_configured_(impedance_mode_configured)
, status_(std::move(status))
, setters_(std::move(setters))
{
    // Start from what the robot itself boots with, so the held configuration is complete and legal
    // from the first request onwards.
    k_q_ = bounds_.k_q_nom;
    z_q_.assign(joint_names_.size(), kNominalDampingRatio);
    max_contact_torques_ = bounds_.tau_max;
    inertia_scales_.assign(joint_names_.size(), kNominalInertiaScale);

    // One group for all three services: each issues a blocking RDK call, and two of those must
    // never be in flight at the same time.
    service_callback_group_
        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    set_joint_impedance_service_ = this->create_service<SetJointImpedance>(
        "~/set_joint_impedance",
        [this](const std::shared_ptr<SetJointImpedance::Request> request,
            std::shared_ptr<SetJointImpedance::Response> response) {
            this->HandleSetJointImpedance(request, response);
        },
        rmw_qos_profile_services_default, service_callback_group_);

    set_max_contact_torque_service_ = this->create_service<SetMaxContactTorque>(
        "~/set_max_contact_torque",
        [this](const std::shared_ptr<SetMaxContactTorque::Request> request,
            std::shared_ptr<SetMaxContactTorque::Response> response) {
            this->HandleSetMaxContactTorque(request, response);
        },
        rmw_qos_profile_services_default, service_callback_group_);

    set_joint_inertia_scale_service_ = this->create_service<SetJointInertiaScale>(
        "~/set_joint_inertia_scale",
        [this](const std::shared_ptr<SetJointInertiaScale::Request> request,
            std::shared_ptr<SetJointInertiaScale::Response> response) {
            this->HandleSetJointInertiaScale(request, response);
        },
        rmw_qos_profile_services_default, service_callback_group_);

    // Latched, so that a late subscriber immediately sees the joint order, the bounds and what is
    // currently set. These change only when something changes them, so there is no republish timer.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    joint_impedance_publisher_
        = this->create_publisher<flexiv_msgs::msg::JointImpedance>("~/joint_impedance", latched);
    max_contact_torque_publisher_ = this->create_publisher<flexiv_msgs::msg::MaxContactTorque>(
        "~/max_contact_torque", latched);
    joint_inertia_scale_publisher_ = this->create_publisher<flexiv_msgs::msg::JointInertiaScale>(
        "~/joint_inertia_scale", latched);

    PublishAll();

    if (impedance_mode_configured_) {
        RCLCPP_INFO(this->get_logger(),
            "Joint impedance interface ready: services '%s/set_joint_impedance', "
            "'%s/set_max_contact_torque', '%s/set_joint_inertia_scale'",
            this->get_fully_qualified_name(), this->get_fully_qualified_name(),
            this->get_fully_qualified_name());
    } else {
        RCLCPP_INFO(this->get_logger(),
            "Joint impedance interface is advertised but inactive: the driver runs in "
            "'joint_position' mode. Relaunch with 'rdk_control_mode:=joint_impedance' to use it.");
    }
}

JointImpedanceConfigNode::~JointImpedanceConfigNode() = default;

bool JointImpedanceConfigNode::InImpedanceMode() const
{
    const auto mode = status_->control_mode.load();
    return mode == flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE
           || mode == flexiv::rdk::Mode::RT_JOINT_IMPEDANCE;
}

bool JointImpedanceConfigNode::CheckPreconditions(std::string& message, bool& deliverable) const
{
    deliverable = false;

    if (!impedance_mode_configured_) {
        message
            = "The driver is running in 'joint_position' mode, so the joint impedance "
              "properties cannot be set. Relaunch with 'rdk_control_mode:=joint_impedance'.";
        return false;
    }

    const auto condition = status_->condition();
    if (!condition.connected) {
        message = "Not connected to the robot.";
        return false;
    }

    // Refuse while the robot is unhealthy rather than holding the values for later. Holding them
    // would apply them silently on the controller restart that a recovery requires, long after the
    // operator has stopped watching.
    const auto driver_state = status_->driver_state.load();
    if (driver_state != DriverState::READY) {
        message = "The robot is not ready. " + DescribeRobotCondition(condition)
                  + " Set the impedance properties once it is ready again.";
        return false;
    }

    // Both can be true while the driver state is still READY, and either means the safety system is
    // actively clamping the robot -- the wrong moment to change compliance or a torque limit.
    if (status_->reduced.load() || status_->recovery_state.load()) {
        message = "The robot is in a reduced or recovery state. "
                  + DescribeRobotCondition(condition)
                  + " Set the impedance properties once it has left that state.";
        return false;
    }

    // Advisory only: the mode can change between this check and the RDK call, which is why the
    // std::logic_error from the setter is also handled as "held, not applied".
    if (!InImpedanceMode()) {
        message = "Accepted and held: the robot is in "
                  + ControlModeName(status_->control_mode.load())
                  + " control mode, so the properties take effect when the controllers are "
                    "(re)started in a joint impedance control mode.";
        return true;
    }

    deliverable = true;
    return true;
}

flexiv_msgs::msg::JointImpedance JointImpedanceConfigNode::BuildJointImpedanceMessage() const
{
    flexiv_msgs::msg::JointImpedance message;
    message.header.stamp = this->now();
    message.joint_names = joint_names_;
    message.k_q = k_q_;
    message.z_q = z_q_;
    message.k_q_nom = bounds_.k_q_nom;
    message.in_effect = in_effect_;
    return message;
}

flexiv_msgs::msg::MaxContactTorque JointImpedanceConfigNode::BuildMaxContactTorqueMessage() const
{
    flexiv_msgs::msg::MaxContactTorque message;
    message.header.stamp = this->now();
    message.joint_names = joint_names_;
    message.max_contact_torques = max_contact_torques_;
    message.tau_max = bounds_.tau_max;
    message.in_effect = in_effect_;
    return message;
}

flexiv_msgs::msg::JointInertiaScale JointImpedanceConfigNode::BuildJointInertiaScaleMessage() const
{
    flexiv_msgs::msg::JointInertiaScale message;
    message.header.stamp = this->now();
    message.joint_names = joint_names_;
    message.inertia_scales = inertia_scales_;
    message.in_effect = in_effect_;
    return message;
}

void JointImpedanceConfigNode::PublishAll()
{
    std::lock_guard<std::mutex> lock(setting_mutex_);
    joint_impedance_publisher_->publish(BuildJointImpedanceMessage());
    max_contact_torque_publisher_->publish(BuildMaxContactTorqueMessage());
    joint_inertia_scale_publisher_->publish(BuildJointInertiaScaleMessage());
}

JointMask JointImpedanceConfigNode::MaskFor(const std::vector<size_t>& selection) const
{
    JointMask touched(joint_names_.size(), false);
    for (const auto index : selection) {
        touched[index] = true;
    }
    return touched;
}

void JointImpedanceConfigNode::ApplySelection(std::vector<double>& held,
    const std::vector<size_t>& selection, const std::vector<double>& values)
{
    for (size_t i = 0; i < selection.size(); ++i) {
        held[selection[i]] = values[i];
    }
}

void JointImpedanceConfigNode::MarkNotInEffect()
{
    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        if (!in_effect_) {
            return;
        }
        in_effect_ = false;
    }
    PublishAll();
}

bool JointImpedanceConfigNode::Reapply()
{
    bool apply_max_contact_torque = false;
    bool apply_inertia_scale = false;
    bool apply_joint_impedance = false;
    std::vector<double> k_q;
    std::vector<double> z_q;
    std::vector<double> max_contact_torques;
    std::vector<double> inertia_scales;
    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        apply_max_contact_torque = customized_max_contact_torque_;
        apply_inertia_scale = customized_inertia_scale_;
        apply_joint_impedance = customized_joint_impedance_;
        if (!apply_max_contact_torque && !apply_inertia_scale && !apply_joint_impedance) {
            // Nothing has ever been set, so the nominal values the mode entry reset the robot to
            // are already what the driver holds.
            return true;
        }
        k_q = k_q_;
        z_q = z_q_;
        max_contact_torques = max_contact_torques_;
        inertia_scales = inertia_scales_;
    }

    // Mode entry reset every joint group, so a re-apply covers all of them regardless of which
    // joints the requests that built up this configuration happened to name.
    const JointMask all_joints(joint_names_.size(), true);

    // Only the properties a request changed, but all of them: mode entry reset every property to
    // nominal, so a property nobody touched needs nothing, while every property the operator has
    // ever set has to be re-sent, not just the one they set most recently.
    const char* property = "";
    try {
        if (apply_max_contact_torque) {
            // Contact torque first, so the clamp is tightened before the stiffness is lowered.
            property = "maximum contact torque";
            setters_.set_max_contact_torque(max_contact_torques, all_joints);
        }
        if (apply_inertia_scale) {
            property = "inertia shaping scale";
            setters_.set_joint_inertia_scale(inertia_scales, all_joints);
        }
        if (apply_joint_impedance) {
            property = "stiffness and damping ratio";
            setters_.set_joint_impedance(k_q, z_q, all_joints);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not re-apply the joint %s: %s", property, e.what());
        {
            std::lock_guard<std::mutex> lock(setting_mutex_);
            in_effect_ = false;
        }
        PublishAll();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        in_effect_ = true;
    }
    PublishAll();

    if (apply_joint_impedance) {
        RCLCPP_WARN(this->get_logger(), "Re-applied joint stiffness %s and damping ratio %s",
            Vec2Str(k_q).c_str(), Vec2Str(z_q).c_str());
    }
    if (apply_max_contact_torque) {
        RCLCPP_WARN(this->get_logger(), "Re-applied maximum contact torque %s",
            Vec2Str(max_contact_torques).c_str());
    }
    if (apply_inertia_scale) {
        RCLCPP_WARN(this->get_logger(), "Re-applied inertia shaping scale %s",
            Vec2Str(inertia_scales).c_str());
    }

    return true;
}

void JointImpedanceConfigNode::HandleSetJointImpedance(
    const std::shared_ptr<SetJointImpedance::Request> request,
    std::shared_ptr<SetJointImpedance::Response> response)
{
    std::string message;
    bool deliverable = false;

    std::vector<size_t> selection;
    bool valid = ResolveJointSelection(request->joint_names, joint_names_, selection, message);

    // An empty z_q means "use the nominal damping ratio", matching the RDK argument default.
    std::vector<double> z_q = request->z_q.empty()
                                  ? std::vector<double>(selection.size(), kNominalDampingRatio)
                                  : request->z_q;

    if (valid) {
        const auto names = GatherNamesByIndex(joint_names_, selection);
        valid = ValidateJointValues(request->k_q, names, 0.0,
                    GatherByIndex(bounds_.k_q_nom, selection), "k_q", message)
                && ValidateJointValues(
                    z_q, names, kMinDampingRatio, kMaxDampingRatio, "z_q", message);
    }

    if (!valid || !CheckPreconditions(message, deliverable)) {
        response->success = false;
        response->message = message;
        std::lock_guard<std::mutex> lock(setting_mutex_);
        response->setting = BuildJointImpedanceMessage();
        RCLCPP_WARN(this->get_logger(), "Rejected joint impedance request: %s", message.c_str());
        return;
    }

    // Merged before delivery, because the RDK sets a whole joint group at a time: a joint the
    // request did not name still has to be sent with the value it already holds.
    std::vector<double> merged_k_q;
    std::vector<double> merged_z_q;
    JointMask touched = MaskFor(selection);
    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        merged_k_q = k_q_;
        merged_z_q = z_q_;
        ApplySelection(merged_k_q, selection, request->k_q);
        ApplySelection(merged_z_q, selection, z_q);
        // Nothing held has reached the robot yet, so this delivery has to carry every group
        // rather than only the named one.
        if (!in_effect_) {
            touched.assign(joint_names_.size(), true);
        }
    }

    if (deliverable) {
        try {
            setters_.set_joint_impedance(merged_k_q, merged_z_q, touched);
        } catch (const std::exception& e) {
            response->success = false;
            response->message
                = "The robot rejected the stiffness and damping ratio: " + std::string(e.what());
            {
                std::lock_guard<std::mutex> lock(setting_mutex_);
                in_effect_ = false;
                response->setting = BuildJointImpedanceMessage();
            }
            // Published, so a subscriber does not keep reading in_effect as true after a delivery
            // the robot refused.
            PublishAll();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        k_q_ = merged_k_q;
        z_q_ = merged_z_q;
        customized_joint_impedance_ = true;
        in_effect_ = deliverable;
        response->setting = BuildJointImpedanceMessage();
    }

    response->success = true;
    response->message = deliverable ? "Joint stiffness and damping ratio applied." : message;

    RCLCPP_WARN(this->get_logger(), "Joint stiffness of %zu joint(s) set to %s, damping ratio %s%s",
        selection.size(), Vec2Str(request->k_q).c_str(), Vec2Str(z_q).c_str(),
        deliverable ? "" : " (held until the controllers are started)");
    for (size_t i = 0; i < request->k_q.size(); ++i) {
        if (request->k_q[i] == 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "Joint '%s' has a stiffness of 0 and is now free-floating. This driver streams "
                "position commands, so that joint will sag under gravity.",
                joint_names_[selection[i]].c_str());
        }
    }
    for (const auto& value : z_q) {
        if (value != kNominalDampingRatio) {
            RCLCPP_WARN(this->get_logger(),
                "A damping ratio away from the nominal %.1f may lead to performance and stability "
                "issues.",
                kNominalDampingRatio);
            break;
        }
    }

    PublishAll();
}

void JointImpedanceConfigNode::HandleSetMaxContactTorque(
    const std::shared_ptr<SetMaxContactTorque::Request> request,
    std::shared_ptr<SetMaxContactTorque::Response> response)
{
    std::string message;
    bool deliverable = false;

    std::vector<size_t> selection;
    bool valid = ResolveJointSelection(request->joint_names, joint_names_, selection, message);
    if (valid) {
        valid = ValidateJointValues(request->max_contact_torques,
            GatherNamesByIndex(joint_names_, selection), 0.0,
            GatherByIndex(bounds_.tau_max, selection), "max_contact_torques", message);
    }

    if (!valid || !CheckPreconditions(message, deliverable)) {
        response->success = false;
        response->message = message;
        std::lock_guard<std::mutex> lock(setting_mutex_);
        response->setting = BuildMaxContactTorqueMessage();
        RCLCPP_WARN(this->get_logger(), "Rejected max contact torque request: %s", message.c_str());
        return;
    }

    // Merged before delivery, because the RDK sets a whole joint group at a time: a joint the
    // request did not name still has to be sent with the value it already holds.
    std::vector<double> merged;
    JointMask touched = MaskFor(selection);
    bool raised = false;
    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        merged = max_contact_torques_;
        for (size_t i = 0; i < selection.size(); ++i) {
            if (request->max_contact_torques[i] > merged[selection[i]]) {
                raised = true;
            }
        }
        ApplySelection(merged, selection, request->max_contact_torques);
        // Nothing held has reached the robot yet, so this delivery has to carry every group
        // rather than only the named one.
        if (!in_effect_) {
            touched.assign(joint_names_.size(), true);
        }
    }

    if (deliverable) {
        try {
            setters_.set_max_contact_torque(merged, touched);
        } catch (const std::exception& e) {
            response->success = false;
            response->message
                = "The robot rejected the maximum contact torque: " + std::string(e.what());
            {
                std::lock_guard<std::mutex> lock(setting_mutex_);
                in_effect_ = false;
                response->setting = BuildMaxContactTorqueMessage();
            }
            // Published, so a subscriber does not keep reading in_effect as true after a delivery
            // the robot refused.
            PublishAll();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        max_contact_torques_ = merged;
        customized_max_contact_torque_ = true;
        in_effect_ = deliverable;
        response->setting = BuildMaxContactTorqueMessage();
    }

    response->success = true;
    response->message = deliverable ? "Maximum contact torque applied." : message;

    RCLCPP_WARN(this->get_logger(), "Maximum contact torque of %zu joint(s) set to %s%s",
        selection.size(), Vec2Str(request->max_contact_torques).c_str(),
        deliverable ? "" : " (held until the controllers are started)");
    if (raised) {
        RCLCPP_WARN(this->get_logger(),
            "The maximum contact torque was raised, which increases the force the robot may exert "
            "on its surroundings.");
    }

    PublishAll();
}

void JointImpedanceConfigNode::HandleSetJointInertiaScale(
    const std::shared_ptr<SetJointInertiaScale::Request> request,
    std::shared_ptr<SetJointInertiaScale::Response> response)
{
    std::string message;
    bool deliverable = false;

    std::vector<size_t> selection;
    bool valid = ResolveJointSelection(request->joint_names, joint_names_, selection, message);
    if (valid) {
        valid = ValidateJointValues(request->inertia_scales,
            GatherNamesByIndex(joint_names_, selection), kMinInertiaScale, kMaxInertiaScale,
            "inertia_scales", message);
    }

    if (!valid || !CheckPreconditions(message, deliverable)) {
        response->success = false;
        response->message = message;
        std::lock_guard<std::mutex> lock(setting_mutex_);
        response->setting = BuildJointInertiaScaleMessage();
        RCLCPP_WARN(this->get_logger(), "Rejected inertia scale request: %s", message.c_str());
        return;
    }

    // Merged before delivery, because the RDK sets a whole joint group at a time: a joint the
    // request did not name still has to be sent with the value it already holds.
    std::vector<double> merged;
    JointMask touched = MaskFor(selection);
    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        merged = inertia_scales_;
        ApplySelection(merged, selection, request->inertia_scales);
        // Nothing held has reached the robot yet, so this delivery has to carry every group
        // rather than only the named one.
        if (!in_effect_) {
            touched.assign(joint_names_.size(), true);
        }
    }

    if (deliverable) {
        try {
            setters_.set_joint_inertia_scale(merged, touched);
        } catch (const std::exception& e) {
            response->success = false;
            response->message
                = "The robot rejected the inertia shaping scale: " + std::string(e.what());
            {
                std::lock_guard<std::mutex> lock(setting_mutex_);
                in_effect_ = false;
                response->setting = BuildJointInertiaScaleMessage();
            }
            // Published, so a subscriber does not keep reading in_effect as true after a delivery
            // the robot refused.
            PublishAll();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(setting_mutex_);
        inertia_scales_ = merged;
        customized_inertia_scale_ = true;
        in_effect_ = deliverable;
        response->setting = BuildJointInertiaScaleMessage();
    }

    response->success = true;
    response->message = deliverable ? "Inertia shaping scale applied." : message;

    RCLCPP_WARN(this->get_logger(), "Inertia shaping scale of %zu joint(s) set to %s%s",
        selection.size(), Vec2Str(request->inertia_scales).c_str(),
        deliverable ? "" : " (held until the controllers are started)");

    PublishAll();
}

} /* namespace flexiv_hardware */

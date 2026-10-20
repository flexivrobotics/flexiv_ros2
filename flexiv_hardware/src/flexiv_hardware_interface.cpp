/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv/rdk/utility.hpp"
#include "flexiv_hardware/fault_recovery.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"
#include "flexiv_hardware/flexiv_robot_states_handle.hpp"
#include "flexiv_hardware/rdk_compat.hpp"

namespace {

constexpr uint64_t kMaxExactRobotStatesHandle = 1ULL << 53;

// Bounded wait for the robot to become operational during activation. Brake release dominates the
// duration; a robot that is not ready within this window needs operator attention.
constexpr std::chrono::seconds kActivationOperationalTimeout {30};
constexpr std::chrono::milliseconds kOperationalPollPeriod {200};

// Bounded wait for the ZeroFTSensor primitive, which takes a few seconds.
constexpr std::chrono::seconds kZeroFTSensorTimeout {10};
constexpr std::chrono::milliseconds kPrimitivePollPeriod {100};

// Furthest a velocity target may lead the measured position [rad], so that a blocked or
// lagging joint cannot build up a large step
constexpr double kMaxVelocityTargetLead = 0.1;

using GroupDofList = std::vector<std::pair<flexiv::rdk::JointGroup, size_t>>;

std::atomic<uint64_t> g_next_robot_states_handle {1};
std::mutex g_robot_states_handle_mutex;
std::unordered_map<uint64_t, flexiv::rdk::RobotStates*> g_robot_states_handle_registry;

bool decode_robot_states_handle(double encoded_handle, uint64_t& handle)
{
    if (!std::isfinite(encoded_handle) || encoded_handle < 1.0
        || encoded_handle > static_cast<double>(kMaxExactRobotStatesHandle)) {
        return false;
    }

    handle = static_cast<uint64_t>(encoded_handle);
    return encoded_handle == static_cast<double>(handle);
}

std::string get_optional_hardware_parameter(
    const hardware_interface::HardwareInfo& info, const std::string& key)
{
    const auto it = info.hardware_parameters.find(key);
    return it != info.hardware_parameters.end() ? it->second : "";
}

std::string joint_group_name_string(flexiv::rdk::JointGroup group)
{
    const auto& group_names = flexiv_hardware::rdk_compat::JointGroupNames();
    const auto name_it = group_names.find(group);
    if (name_it != group_names.end()) {
        return name_it->second;
    }
    return "GROUP_" + std::to_string(static_cast<int>(group));
}

/**
 * @brief Whether a request touched any joint of the group occupying [begin, begin + dof) of the
 * RDK-ordered impedance vector. The RDK sets a whole joint group at a time, so a group nothing
 * touched is skipped rather than re-sent with the values it already holds.
 *
 * @param touched_ros Per-joint mask in ROS (URDF) order.
 * @param rdk_to_ros_map Index is the RDK index, value is the index into [touched_ros].
 */
bool group_is_touched(const std::vector<bool>& touched_ros,
    const std::vector<size_t>& rdk_to_ros_map, size_t begin, size_t dof)
{
    for (size_t rdk_index = begin; rdk_index < begin + dof; ++rdk_index) {
        const size_t ros_index = rdk_to_ros_map[rdk_index];
        if (ros_index < touched_ros.size() && touched_ros[ros_index]) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Map the joints of [groups], taken from [rdk_to_ros_map] in RDK order, onto a joint list of
 * just those joints in URDF order.
 * @param[in] group_slices Begin index into [rdk_to_ros_map] and DoF of each group, in RDK order.
 * @param[out] joint_list_indices ROS joint index of each entry of the joint list, in URDF order.
 * @return Index is the RDK index over the groups' joints, value is the index into the joint list.
 */
std::vector<size_t> map_groups_to_joint_list(
    const std::vector<std::pair<size_t, size_t>>& group_slices,
    const std::vector<size_t>& rdk_to_ros_map, std::vector<size_t>& joint_list_indices)
{
    std::vector<size_t> ros_indices;
    for (const auto& [begin, dof] : group_slices) {
        ros_indices.insert(ros_indices.end(),
            rdk_to_ros_map.begin() + static_cast<std::ptrdiff_t>(begin),
            rdk_to_ros_map.begin() + static_cast<std::ptrdiff_t>(begin + dof));
    }

    joint_list_indices = ros_indices;
    std::sort(joint_list_indices.begin(), joint_list_indices.end());

    std::vector<size_t> map;
    map.reserve(ros_indices.size());
    for (const auto ros_index : ros_indices) {
        const auto it
            = std::lower_bound(joint_list_indices.begin(), joint_list_indices.end(), ros_index);
        map.push_back(static_cast<size_t>(std::distance(joint_list_indices.begin(), it)));
    }
    return map;
}

/** Interface a joint group ends up claimed with, given what a mode switch starts and stops. */
uint8_t next_claimed_interface(uint8_t current, uint8_t starting, uint8_t stopping)
{
    if (starting != flexiv_hardware::kInterfaceNone) {
        return starting;
    }
    if (stopping != flexiv_hardware::kInterfaceNone) {
        return flexiv_hardware::kInterfaceNone;
    }
    return current;
}

template <size_t N>
bool all_finite(const std::array<double, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

}

namespace flexiv_hardware {

double register_robot_states_handle(flexiv::rdk::RobotStates* robot_states)
{
    if (robot_states == nullptr) {
        return 0.0;
    }

    const auto handle = g_next_robot_states_handle.fetch_add(1);
    if (handle > kMaxExactRobotStatesHandle) {
        throw std::overflow_error("Exhausted exact robot-state handles");
    }

    std::lock_guard<std::mutex> lock(g_robot_states_handle_mutex);
    g_robot_states_handle_registry[handle] = robot_states;
    return static_cast<double>(handle);
}

void unregister_robot_states_handle(double encoded_handle)
{
    uint64_t handle = 0;
    if (!decode_robot_states_handle(encoded_handle, handle)) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_robot_states_handle_mutex);
    g_robot_states_handle_registry.erase(handle);
}

flexiv::rdk::RobotStates* resolve_robot_states_handle(double encoded_handle)
{
    uint64_t handle = 0;
    if (!decode_robot_states_handle(encoded_handle, handle)) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(g_robot_states_handle_mutex);
    const auto it = g_robot_states_handle_registry.find(handle);
    return it != g_robot_states_handle_registry.end() ? it->second : nullptr;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (hardware_interface::SystemInterface::on_init(params)
        != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        getLogger(), "Parsed %zu joints from ros2_control hardware info", info_.joints.size());

    hw_states_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_targets_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    target_pos_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    target_vel_.assign(info_.joints.size(), 0.0);
    target_torque_.assign(info_.joints.size(), 0.0);
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    claimed_interfaces_.fill(kInterfaceNone);
    cartesian_claimed_ = false;

    if (info_.joints.size() < 7) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected at least 7.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto prefix = get_optional_hardware_parameter(info_, "prefix");
    const auto prefix_left = get_optional_hardware_parameter(info_, "prefix_left");
    const auto prefix_right = get_optional_hardware_parameter(info_, "prefix_right");

    arm_prefixes_.clear();
    if (!prefix.empty()) {
        if (!prefix_left.empty() || !prefix_right.empty()) {
            RCLCPP_FATAL(getLogger(),
                "Parameters 'prefix' and 'prefix_left'/'prefix_right' are mutually exclusive");
            return hardware_interface::CallbackReturn::ERROR;
        }
        arm_prefixes_.push_back(prefix);
    } else if (!prefix_left.empty() && !prefix_right.empty()) {
        arm_prefixes_.push_back(prefix_left);
        arm_prefixes_.push_back(prefix_right);
    } else {
        RCLCPP_FATAL(getLogger(),
            "Expected hardware parameter 'prefix' for a single-arm setup (generated from "
            "robot_sn by the single-arm xacro), or both 'prefix_left' and 'prefix_right' for "
            "a dual-arm setup");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Build RDK to ROS joint mapping
    std::vector<size_t> arm_indices;
    std::vector<size_t> ext_indices;

    std::set<size_t> arm_joint_index_set;
    for (const auto& arm_prefix : arm_prefixes_) {
        std::vector<std::pair<int, size_t>> indexed_arm_joints;
        const std::string arm_joint_prefix = arm_prefix + "joint";
        indexed_arm_joints.reserve(info_.joints.size());

        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (arm_joint_index_set.count(i) > 0) {
                continue;
            }

            const std::string& joint_name = info_.joints[i].name;
            if (joint_name.rfind(arm_joint_prefix, 0) != 0) {
                continue;
            }

            const std::string suffix = joint_name.substr(arm_joint_prefix.size());
            if (suffix.empty()) {
                continue;
            }

            try {
                size_t parsed_chars = 0;
                int joint_index = std::stoi(suffix, &parsed_chars);
                if (parsed_chars == suffix.size() && joint_index > 0) {
                    indexed_arm_joints.emplace_back(joint_index, i);
                }
            } catch (const std::exception&) {
                continue;
            }
        }

        if (indexed_arm_joints.size() != flexiv::rdk::kSerialJointDoF) {
            RCLCPP_FATAL(getLogger(),
                "Arm prefix '%s' resolved to %ld joints. Expected exactly %zu", arm_prefix.c_str(),
                indexed_arm_joints.size(), flexiv::rdk::kSerialJointDoF);
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::sort(indexed_arm_joints.begin(), indexed_arm_joints.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        if (indexed_arm_joints.front().first != 1) {
            RCLCPP_FATAL(getLogger(), "Arm joint numbering must start at 1 ('%sjoint1')",
                arm_prefix.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (size_t i = 1; i < indexed_arm_joints.size(); ++i) {
            if (indexed_arm_joints[i].first != indexed_arm_joints[i - 1].first + 1) {
                RCLCPP_FATAL(getLogger(),
                    "Arm joints must be contiguous '%sjoint1' ... '%sjointN' without "
                    "gaps/duplicates",
                    arm_prefix.c_str(), arm_prefix.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        for (const auto& [joint_number, joint_pos] : indexed_arm_joints) {
            (void)joint_number;
            arm_indices.push_back(joint_pos);
            arm_joint_index_set.insert(joint_pos);
        }
    }

    // Find external axis joints (any joint that is not an arm joint)
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (arm_joint_index_set.count(i) == 0) {
            ext_indices.push_back(i);
        }
    }

    // Construct map: external joints first, then arm joints (RDK order)
    rdk_to_ros_map_.clear();
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), ext_indices.begin(), ext_indices.end());
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), arm_indices.begin(), arm_indices.end());

    // Partition the joints into RDK joint groups. Every ROS joint is either an arm joint or an
    // external axis, so rdk_to_ros_map_ is a permutation of [0, n) and each group owns one
    // contiguous slice of it: external axes first, then arm 1, then arm 2. The groups are named
    // once the robot reports them, see ResolveJointGroups().
    active_groups_.clear();
    arm_groups_.clear();
    if (!ext_indices.empty()) {
        active_groups_.emplace_back(flexiv::rdk::JointGroup::UNKNOWN, ext_indices.size());
    }
    for (size_t i = 0; i < arm_prefixes_.size(); ++i) {
        arm_groups_.push_back(active_groups_.size());
        active_groups_.emplace_back(flexiv::rdk::JointGroup::UNKNOWN, flexiv::rdk::kSerialJointDoF);
    }
    if (active_groups_.size() > kMaxJointGroups) {
        RCLCPP_FATAL(getLogger(), "Resolved %zu joint groups. Expected at most %zu.",
            active_groups_.size(), kMaxJointGroups);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Cartesian buffers, one set per arm
    const size_t num_arms = arm_prefixes_.size();
    std::array<double, flexiv::rdk::kPoseSize> nan_pose;
    std::array<double, flexiv::rdk::kCartDoF> nan_cart;
    nan_pose.fill(std::numeric_limits<double>::quiet_NaN());
    nan_cart.fill(std::numeric_limits<double>::quiet_NaN());
    hw_commands_cartesian_pose_.assign(num_arms, nan_pose);
    hw_commands_cartesian_wrench_.assign(num_arms, nan_cart);
    hw_commands_cartesian_velocity_.assign(num_arms, nan_cart);
    hw_states_cartesian_pose_.assign(num_arms, nan_pose);
    last_cartesian_target_.assign(num_arms, nan_pose);
    cartesian_command_interface_names_.clear();
    for (const auto& arm_prefix : arm_prefixes_) {
        for (const auto& name : CartesianCommandInterfaceNames(arm_prefix)) {
            cartesian_command_interface_names_.push_back(name);
        }
    }

    try {
        info_.hardware_parameters.at("robot_sn");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_sn' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld state interfaces found. 3 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Optional, so that a description without it keeps the real-time modes v2.x has always used
    const auto realtime_it = info_.hardware_parameters.find("rdk_realtime_mode");
    try {
        rdk_realtime_ = realtime_it == info_.hardware_parameters.end()
                        || hardware_interface::parse_bool(realtime_it->second);
    } catch (const std::invalid_argument& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'rdk_realtime_mode': %s", ex.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto withhold_it = info_.hardware_parameters.find("withhold_on_timeliness_failure");
    try {
        withhold_on_timeliness_failure_ = withhold_it == info_.hardware_parameters.end()
                                          || hardware_interface::parse_bool(withhold_it->second);
    } catch (const std::invalid_argument& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'withhold_on_timeliness_failure': %s", ex.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        auto rdk_control_mode_str = info_.hardware_parameters.at("rdk_control_mode");
        if (rdk_control_mode_str == "joint_position") {
            rdk_control_mode_ = rdk_realtime_ ? flexiv::rdk::Mode::RT_JOINT_POSITION
                                              : flexiv::rdk::Mode::NRT_JOINT_POSITION;
        } else if (rdk_control_mode_str == "joint_impedance") {
            rdk_control_mode_ = rdk_realtime_ ? flexiv::rdk::Mode::RT_JOINT_IMPEDANCE
                                              : flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE;
        } else {
            RCLCPP_FATAL(getLogger(),
                "Parameter 'rdk_control_mode' has invalid value '%s'. Options: joint_position, "
                "joint_impedance",
                rdk_control_mode_str.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'rdk_control_mode' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }
    rdk_cartesian_mode_ = rdk_realtime_ ? flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE
                                        : flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE;
    RCLCPP_INFO(getLogger(),
        "RDK control modes: %s (joint), %s (Cartesian), RT_JOINT_TORQUE (effort)",
        ControlModeName(rdk_control_mode_).c_str(), ControlModeName(rdk_cartesian_mode_).c_str());
    if (rdk_realtime_) {
        RCLCPP_INFO(getLogger(),
            "Real-time modes stream every cycle without the robot's motion generator, so the joint "
            "and Cartesian motion limits do not apply");
    }

    // Limits for the robot's joint motion generator, in RDK order
    const auto max_velocities = JointVelocityLimits(info_);
    try {
        max_joint_acc_ = std::vector<double>(info_.joints.size(), MaxJointAcceleration(info_));
    } catch (const std::invalid_argument& ex) {
        RCLCPP_FATAL(getLogger(), "%s", ex.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    max_joint_vel_ = ConvertROSToRDKOrder(max_velocities, rdk_to_ros_map_);
    if (!rdk_realtime_) {
        std::string max_vel_str;
        for (double v : max_joint_vel_) {
            max_vel_str += std::to_string(v) + " ";
        }
        RCLCPP_INFO(getLogger(),
            "Joint motion limits (RDK order): max_vel [ %s] rad/s, max_acc %f rad/s^2",
            max_vel_str.c_str(), max_joint_acc_.front());
    }

    for (size_t i = 0; i < kMaxJointGroups; ++i) {
        const CartesianMotionLimits defaults;
        cartesian_max_linear_vel_[i].store(defaults.max_linear_vel);
        cartesian_max_angular_vel_[i].store(defaults.max_angular_vel);
        cartesian_max_linear_acc_[i].store(defaults.max_linear_acc);
        cartesian_max_angular_acc_[i].store(defaults.max_angular_acc);
    }

    // The connection is opened here because the state interface export, which the resource manager
    // runs right after on_init(), needs the robot's joint groups to name the per-group robot-state
    // interfaces. on_configure() re-opens it after an on_cleanup(), which is what lets a lost
    // connection be recovered without restarting the whole process.
    driver_status_ = std::make_shared<DriverStatus>();
    executor_ = params.executor;

    if (!Connect()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

bool FlexivHardwareInterface::Connect()
{
    if (robot_) {
        return true;
    }

    const std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), "%s", e.what());
        return false;
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return true;
}

bool FlexivHardwareInterface::ResolveJointGroups()
{
    const auto robot_info = robot_->info();

    // The commandable groups in RDK order: external axes first, then the single arms in ascending
    // JointGroup order (ARM_1, then ARM_2), matching rdk_to_ros_map_.
    GroupDofList robot_groups;
    const auto ext_dof_it = robot_info.DoF.find(flexiv::rdk::JointGroup::EXT_AXIS);
    if (ext_dof_it != robot_info.DoF.end() && ext_dof_it->second > 0) {
        robot_groups.emplace_back(flexiv::rdk::JointGroup::EXT_AXIS, ext_dof_it->second);
    }
    for (const auto& [group, name] : robot_info.single_arm_groups) {
        (void)name;
        const auto dof_it = robot_info.DoF.find(group);
        if (dof_it != robot_info.DoF.end() && dof_it->second > 0) {
            robot_groups.emplace_back(group, dof_it->second);
        }
    }

    if (robot_groups.size() != active_groups_.size()) {
        RCLCPP_FATAL(getLogger(),
            "Robot reports %zu commandable joint groups, but the URDF resolved %zu",
            robot_groups.size(), active_groups_.size());
        return false;
    }

    size_t offset = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        if (robot_groups[g].second != active_groups_[g].second) {
            RCLCPP_FATAL(getLogger(),
                "Joint group %s has %zu joints on the robot, but %zu in the URDF",
                joint_group_name_string(robot_groups[g].first).c_str(), robot_groups[g].second,
                active_groups_[g].second);
            return false;
        }
        active_groups_[g].first = robot_groups[g].first;
        RCLCPP_INFO(getLogger(), "Joint group %s drives %zu joints, starting at '%s'",
            joint_group_name_string(active_groups_[g].first).c_str(), active_groups_[g].second,
            info_.joints[rdk_to_ros_map_[offset]].name.c_str());
        offset += active_groups_[g].second;
    }
    for (size_t i = 0; i < arm_groups_.size(); ++i) {
        if (active_groups_[arm_groups_[i]].first == flexiv::rdk::JointGroup::EXT_AXIS) {
            RCLCPP_FATAL(getLogger(), "Arm '%s' resolved to the external axis group",
                arm_prefixes_[i].c_str());
            return false;
        }
    }

    // Preallocate every per-group command, so that write() does not allocate
    rt_joint_position_cmds_.clear();
    rt_joint_torque_cmds_.clear();
    nrt_joint_position_cmds_.clear();
    offset = 0;
    for (const auto& [group, group_dof] : active_groups_) {
        const auto begin = max_joint_vel_.begin() + static_cast<std::ptrdiff_t>(offset);
        auto& pos_cmd = rt_joint_position_cmds_[group];
        pos_cmd.q_d.assign(group_dof, 0.0);
        pos_cmd.dq_d.assign(group_dof, 0.0);
        pos_cmd.ddq_d.assign(group_dof, 0.0);

        auto& torque_cmd = rt_joint_torque_cmds_[group];
        torque_cmd.tau_d.assign(group_dof, 0.0);
        torque_cmd.enable_gravity_comp = true;
        torque_cmd.enable_soft_limits = true;

        auto& nrt_cmd = nrt_joint_position_cmds_[group];
        nrt_cmd.q_d.assign(group_dof, 0.0);
        nrt_cmd.dq_d.assign(group_dof, 0.0);
        nrt_cmd.dq_max.assign(begin, begin + static_cast<std::ptrdiff_t>(group_dof));
        nrt_cmd.ddq_max.assign(group_dof, max_joint_acc_.front());
        offset += group_dof;
    }

    rt_cartesian_cmds_.clear();
    nrt_cartesian_cmds_.clear();
    for (const auto g : arm_groups_) {
        rt_cartesian_cmds_[active_groups_[g].first] = flexiv::rdk::RtCartesianCmd {};
        nrt_cartesian_cmds_[active_groups_[g].first] = flexiv::rdk::NrtCartesianCmd {};
    }

    return true;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    const std::string robot_sn = info_.hardware_parameters.at("robot_sn");

    if (!Connect()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Check the commandable joint groups of the robot against the URDF before anything is enabled.
    if (!ResolveJointGroups()) {
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    robot_system_control_ = std::make_unique<RdkRobotSystemControl>(*robot_);
    driver_status_->driver_state.store(DriverState::FAULT);

    if (!StartSupervisoryNodes(robot_sn)) {
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

bool FlexivHardwareInterface::StartSupervisoryNodes(const std::string& robot_sn)
{
    // Host the nodes on the controller manager's executor, so that all blocking system control
    // calls happen off the real-time control loop without needing a thread here.
    auto executor = executor_.lock();
    if (!executor) {
        RCLCPP_FATAL(getLogger(),
            "No executor available to host the recovery interface. The controller manager must "
            "provide one through HardwareComponentInterfaceParams.");
        return false;
    }

    recovery_node_
        = std::make_shared<RecoveryNode>(robot_sn, *robot_system_control_, driver_status_);
    executor->add_node(recovery_node_->get_node_base_interface());

    // The impedance properties are set one joint group at a time in RDK v2.x, and only the
    // single-arm groups report a nominal joint stiffness to validate against, so the interface
    // covers the arm joints. External axes (the MICO pan-tilt torso) are left out.
    const auto robot_info = robot_->info();

    // Begin index into rdk_to_ros_map_ of each arm group, parallel to arm_groups_
    std::vector<size_t> arm_begins;
    {
        size_t offset = 0;
        for (size_t g = 0, arm = 0; g < active_groups_.size(); ++g) {
            if (arm < arm_groups_.size() && arm_groups_[arm] == g) {
                arm_begins.push_back(offset);
                ++arm;
            }
            offset += active_groups_[g].second;
        }
    }

    impedance_groups_.clear();
    std::vector<std::pair<size_t, size_t>> impedance_slices;
    for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
        const auto& [group, group_dof] = active_groups_[arm_groups_[arm]];
        const auto k_q_nom_it = robot_info.K_q_nom.find(group);
        if (k_q_nom_it == robot_info.K_q_nom.end() || k_q_nom_it->second.size() != group_dof) {
            RCLCPP_WARN(getLogger(),
                "Joint group '%s' reports no usable nominal joint stiffness; it is left out of the "
                "joint impedance interface",
                joint_group_name_string(group).c_str());
            continue;
        }
        impedance_groups_.emplace_back(group, group_dof);
        impedance_slices.emplace_back(arm_begins[arm], group_dof);
    }

    // The impedance joint list is in URDF order, matching the "one entry per arm joint in URDF
    // order" contract of the services.
    std::vector<size_t> impedance_ros_indices;
    impedance_rdk_to_ros_map_
        = map_groups_to_joint_list(impedance_slices, rdk_to_ros_map_, impedance_ros_indices);

    std::vector<std::string> joint_names;
    joint_names.reserve(impedance_ros_indices.size());
    for (const auto ros_index : impedance_ros_indices) {
        joint_names.push_back(info_.joints[ros_index].name);
    }

    // Per-joint bounds, gathered per group in RDK order and then permuted into the joint list's
    // order.
    std::vector<double> k_q_nom_rdk;
    std::vector<double> tau_max_rdk;
    for (const auto& [group, group_dof] : impedance_groups_) {
        const auto& k_q_nom = robot_info.K_q_nom.at(group);
        k_q_nom_rdk.insert(k_q_nom_rdk.end(), k_q_nom.begin(), k_q_nom.end());

        const auto tau_max_it = robot_info.tau_max.find(group);
        if (tau_max_it != robot_info.tau_max.end() && tau_max_it->second.size() == group_dof) {
            tau_max_rdk.insert(
                tau_max_rdk.end(), tau_max_it->second.begin(), tau_max_it->second.end());
        } else {
            // No reported limit means no headroom can be granted: a request above 0 is refused
            // rather than sent to the robot on a guess.
            tau_max_rdk.insert(tau_max_rdk.end(), group_dof, 0.0);
        }
    }

    JointImpedanceBounds bounds;
    bounds.k_q_nom = ConvertRDKToROSOrder(k_q_nom_rdk, impedance_rdk_to_ros_map_);
    bounds.tau_max = ConvertRDKToROSOrder(tau_max_rdk, impedance_rdk_to_ros_map_);

    // The node works in ROS joint order and knows nothing about the RDK; these three closures are
    // where the order is translated, the vector is split per joint group and the RDK is called.
    JointImpedanceSetters setters;
    setters.set_joint_impedance = [this](const std::vector<double>& k_q,
                                      const std::vector<double>& z_q, const JointMask& touched) {
        const auto k_q_rdk = ConvertROSToRDKOrder(k_q, impedance_rdk_to_ros_map_);
        const auto z_q_rdk = ConvertROSToRDKOrder(z_q, impedance_rdk_to_ros_map_);
        size_t offset = 0;
        for (const auto& [group, group_dof] : impedance_groups_) {
            if (group_is_touched(touched, impedance_rdk_to_ros_map_, offset, group_dof)) {
                const auto begin = static_cast<std::ptrdiff_t>(offset);
                const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
                robot_->SetJointImpedance(group, {k_q_rdk.begin() + begin, k_q_rdk.begin() + end},
                    {z_q_rdk.begin() + begin, z_q_rdk.begin() + end});
            }
            offset += group_dof;
        }
    };
    setters.set_max_contact_torque
        = [this](const std::vector<double>& max_torques, const JointMask& touched) {
              const auto rdk = ConvertROSToRDKOrder(max_torques, impedance_rdk_to_ros_map_);
              size_t offset = 0;
              for (const auto& [group, group_dof] : impedance_groups_) {
                  if (group_is_touched(touched, impedance_rdk_to_ros_map_, offset, group_dof)) {
                      const auto begin = static_cast<std::ptrdiff_t>(offset);
                      const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
                      robot_->SetMaxContactTorque(group, {rdk.begin() + begin, rdk.begin() + end});
                  }
                  offset += group_dof;
              }
          };
    setters.set_joint_inertia_scale
        = [this](const std::vector<double>& inertia_scales, const JointMask& touched) {
              const auto rdk = ConvertROSToRDKOrder(inertia_scales, impedance_rdk_to_ros_map_);
              size_t offset = 0;
              for (const auto& [group, group_dof] : impedance_groups_) {
                  if (group_is_touched(touched, impedance_rdk_to_ros_map_, offset, group_dof)) {
                      const auto begin = static_cast<std::ptrdiff_t>(offset);
                      const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
                      robot_->SetJointInertiaScale(group, {rdk.begin() + begin, rdk.begin() + end});
                  }
                  offset += group_dof;
              }
          };

    joint_impedance_config_node_ = std::make_shared<JointImpedanceConfigNode>(robot_sn,
        std::move(joint_names), std::move(bounds),
        rdk_control_mode_ == flexiv::rdk::Mode::RT_JOINT_IMPEDANCE
            || rdk_control_mode_ == flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE,
        driver_status_, std::move(setters));
    executor->add_node(joint_impedance_config_node_->get_node_base_interface());

    // The Cartesian motion-force settings are also per joint group, one entry per arm. The
    // null-space posture covers every arm joint, in URDF order.
    std::vector<std::pair<size_t, size_t>> arm_slices;
    for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
        arm_slices.emplace_back(arm_begins[arm], active_groups_[arm_groups_[arm]].second);
    }
    std::vector<size_t> arm_ros_indices;
    cartesian_rdk_to_ros_map_
        = map_groups_to_joint_list(arm_slices, rdk_to_ros_map_, arm_ros_indices);

    std::vector<std::string> arm_joint_names;
    arm_joint_names.reserve(arm_ros_indices.size());
    for (const auto ros_index : arm_ros_indices) {
        arm_joint_names.push_back(info_.joints[ros_index].name);
    }

    CartesianMotionForceBounds cartesian_bounds;
    std::vector<double> q_min_rdk;
    std::vector<double> q_max_rdk;
    for (const auto g : arm_groups_) {
        const auto& [group, group_dof] = active_groups_[g];
        const auto k_x_nom_it = robot_info.K_x_nom.find(group);
        if (k_x_nom_it != robot_info.K_x_nom.end()) {
            cartesian_bounds.k_x_nom.push_back(k_x_nom_it->second);
        } else {
            // No reported stiffness means no stiffness can be granted, as for the joint impedance
            RCLCPP_WARN(getLogger(), "Joint group '%s' reports no nominal Cartesian stiffness",
                joint_group_name_string(group).c_str());
            cartesian_bounds.k_x_nom.push_back(CartesianArray {});
        }

        const auto q_min_it = robot_info.q_min.find(group);
        const auto q_max_it = robot_info.q_max.find(group);
        const bool has_q_limits
            = q_min_it != robot_info.q_min.end() && q_max_it != robot_info.q_max.end()
              && q_min_it->second.size() == group_dof && q_max_it->second.size() == group_dof;
        if (has_q_limits) {
            q_min_rdk.insert(q_min_rdk.end(), q_min_it->second.begin(), q_min_it->second.end());
            q_max_rdk.insert(q_max_rdk.end(), q_max_it->second.begin(), q_max_it->second.end());
        } else {
            // An empty range refuses every posture rather than sending one on a guess
            q_min_rdk.insert(q_min_rdk.end(), group_dof, 0.0);
            q_max_rdk.insert(q_max_rdk.end(), group_dof, 0.0);
        }
    }
    cartesian_bounds.q_min = ConvertRDKToROSOrder(q_min_rdk, cartesian_rdk_to_ros_map_);
    cartesian_bounds.q_max = ConvertRDKToROSOrder(q_max_rdk, cartesian_rdk_to_ros_map_);

    // Every per-arm vector holds one entry per arm, parallel to arm_groups_.
    const auto arm_group = [this](size_t arm) { return active_groups_[arm_groups_[arm]].first; };
    CartesianMotionForceSetters cartesian_setters;
    cartesian_setters.set_cartesian_impedance
        = [this, arm_group](
              const std::vector<CartesianArray>& k_x, const std::vector<CartesianArray>& z_x) {
              for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
                  robot_->SetCartesianImpedance(arm_group(arm), k_x[arm], z_x[arm]);
              }
          };
    cartesian_setters.set_max_contact_wrench
        = [this, arm_group](const std::vector<CartesianArray>& wrench) {
              for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
                  robot_->SetMaxContactWrench(arm_group(arm), wrench[arm]);
              }
          };
    cartesian_setters.set_null_space_posture = [this, arm_group](
                                                   const std::vector<double>& ref_positions) {
        const auto rdk = ConvertROSToRDKOrder(ref_positions, cartesian_rdk_to_ros_map_);
        size_t offset = 0;
        for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
            const size_t group_dof = active_groups_[arm_groups_[arm]].second;
            const auto begin = static_cast<std::ptrdiff_t>(offset);
            const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
            robot_->SetNullSpacePosture(arm_group(arm), {rdk.begin() + begin, rdk.begin() + end});
            offset += group_dof;
        }
    };
    cartesian_setters.set_null_space_objectives
        = [this, arm_group](const std::vector<NullSpaceObjectives>& objectives) {
              for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
                  robot_->SetNullSpaceObjectives(arm_group(arm),
                      objectives[arm].linear_manipulability, objectives[arm].angular_manipulability,
                      objectives[arm].ref_positions_tracking);
              }
          };
    cartesian_setters.set_force_control_axis = [this, arm_group](
                                                   const std::vector<CartesianFlags>& enabled_axes,
                                                   const std::vector<LinearArray>& max_linear_vel) {
        for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
            robot_->SetForceControlAxis(arm_group(arm), enabled_axes[arm], max_linear_vel[arm]);
        }
    };
    cartesian_setters.set_force_control_frame
        = [this, arm_group](const std::vector<flexiv::rdk::CoordType>& root_coord,
              const std::vector<PoseArray>& t_in_root) {
              for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
                  robot_->SetForceControlFrame(arm_group(arm), root_coord[arm], t_in_root[arm]);
              }
          };
    cartesian_setters.set_passive_force_control
        = [this, arm_group](const std::vector<bool>& enabled) {
              for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
                  robot_->SetPassiveForceControl(arm_group(arm), enabled[arm]);
              }
          };
    cartesian_setters.set_motion_limits = [this](const std::vector<CartesianMotionLimits>& limits) {
        if (rdk_realtime_) {
            throw std::logic_error(
                "the motion limits apply only to NRT_CARTESIAN_MOTION_FORCE, "
                "relaunch with 'rdk_realtime_mode:=false' to use them");
        }
        for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
            cartesian_max_linear_vel_[arm].store(limits[arm].max_linear_vel);
            cartesian_max_angular_vel_[arm].store(limits[arm].max_angular_vel);
            cartesian_max_linear_acc_[arm].store(limits[arm].max_linear_acc);
            cartesian_max_angular_acc_[arm].store(limits[arm].max_angular_acc);
        }
    };

    cartesian_config_node_
        = std::make_shared<CartesianMotionForceConfigNode>(robot_sn, std::move(arm_joint_names),
            std::move(cartesian_bounds), driver_status_, std::move(cartesian_setters));
    executor->add_node(cartesian_config_node_->get_node_base_interface());

    return true;
}

void FlexivHardwareInterface::TrackPositionChangeAcrossInterruption()
{
    const bool ready = driver_status_->driver_state.load() == DriverState::READY;

    if (was_ready_ && !ready) {
        // Joint states stop being refreshed once the robot is no longer operational, so this still
        // holds the last position the robot was known to be at before it stopped.
        positions_before_interruption_ = hw_states_joint_positions_;
    } else if (!was_ready_ && ready && !positions_before_interruption_.empty()) {
        if (driver_status_->RequiresControllerRestart()) {
            const double deviation
                = MaxJointDeviation(positions_before_interruption_, hw_states_joint_positions_);

            RCLCPP_WARN(getLogger(),
                "The robot is ready again, %.3f rad from the last commanded "
                "position. Motion stays withheld until the controllers are restarted.",
                deviation);
        }
        positions_before_interruption_.clear();
    }

    was_ready_ = ready;
}

void FlexivHardwareInterface::StopIfOperational()
{
    if (robot_ && robot_->connected() && robot_->operational()) {
        robot_->Stop();
    }
}

bool FlexivHardwareInterface::ZeroForceTorqueSensors()
{
    if (driver_status_->driver_state.load() != DriverState::READY) {
        RCLCPP_ERROR(getLogger(), "Cannot zero the force/torque sensors: the robot is not ready");
        return false;
    }

    RCLCPP_WARN(getLogger(),
        "Zeroing the force/torque sensors, make sure nothing is in contact with the robot");
    try {
        std::map<flexiv::rdk::JointGroup, flexiv::rdk::PrimitiveArgs> pt_args;
        for (const auto g : arm_groups_) {
            pt_args[active_groups_[g].first] = flexiv::rdk::PrimitiveArgs("ZeroFTSensor", {});
        }
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive(pt_args);

        const auto deadline = std::chrono::steady_clock::now() + kZeroFTSensorTimeout;
        while (!flexiv::rdk::utility::PrimitiveStateTrueForGroups(
            robot_->primitive_states(), "terminated")) {
            if (robot_->fault()) {
                throw std::runtime_error("a fault occurred on the robot");
            }
            if (std::chrono::steady_clock::now() > deadline) {
                throw std::runtime_error("the ZeroFTSensor primitive did not finish in time");
            }
            std::this_thread::sleep_for(kPrimitivePollPeriod);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not zero the force/torque sensors: %s", e.what());
        StopIfOperational();
        return false;
    }

    // Back to IDLE, as before the zeroing
    StopIfOperational();
    RCLCPP_INFO(getLogger(), "Force/torque sensors zeroed");
    return true;
}

void FlexivHardwareInterface::Disconnect()
{
    if (recovery_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(recovery_node_->get_node_base_interface());
        }
        recovery_node_.reset();
    }
    // Torn down before robot_ below, since their closures capture this and call through it.
    if (joint_impedance_config_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(joint_impedance_config_node_->get_node_base_interface());
        }
        joint_impedance_config_node_.reset();
    }
    if (cartesian_config_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(cartesian_config_node_->get_node_base_interface());
        }
        cartesian_config_node_.reset();
    }
    robot_system_control_.reset();
    robot_.reset();
    claimed_interfaces_.fill(kInterfaceNone);
    cartesian_claimed_ = false;
    if (driver_status_) {
        driver_status_->driver_state.store(DriverState::UNINITIALIZED);
        driver_status_->commands_synchronized.store(false);
    }
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Cleaning up, closing the connection to the robot ...");
    Disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    Disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_ERROR(getLogger(), "Hardware component entered the error state, stopping the robot");

    try {
        StopIfOperational();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not stop the robot: %s", e.what());
    }

    Disconnect();

    // Returning SUCCESS puts the component in UNCONFIGURED, from which it can be configured and
    // activated again. Returning FAILURE or ERROR here would finalize it, and the whole process
    // would have to be restarted to recover.
    return hardware_interface::CallbackReturn::SUCCESS;
}

rclcpp::Logger FlexivHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivHardwareInterface");
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
FlexivHardwareInterface::on_export_state_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.push_back(interfaces_.BindState(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.push_back(interfaces_.BindState(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.push_back(interfaces_.BindState(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_joint_efforts_[i]));
    }

    for (const auto& [group, encoded_handle] : hw_flexiv_robot_state_handles_by_group_) {
        (void)group;
        unregister_robot_states_handle(encoded_handle);
    }

    hw_flexiv_robot_states_by_group_.clear();
    hw_flexiv_robot_state_handles_by_group_.clear();
    std::string robot_sn = info_.hardware_parameters.at("robot_sn");

    std::vector<flexiv::rdk::JointGroup> groups;
    if (robot_) {
        for (const auto& [group, name] : robot_->info().all_groups) {
            if (group == flexiv::rdk::JointGroup::ALL
                || group == flexiv::rdk::JointGroup::UNKNOWN) {
                continue;
            }
            groups.push_back(group);
        }
    }
    if (groups.empty()) {
        groups.push_back(flexiv::rdk::JointGroup::ARMS);
        RCLCPP_WARN(getLogger(),
            "Robot reported no joint groups during state interface export; defaulting to ARMS");
    }

    // The RDK populates the Cartesian states (TCP and flange poses, wrenches) of the single-arm
    // groups only, so robot_sn names the arm on a single-arm robot, not the ARMS aggregate.
    const bool single_arm = arm_prefixes_.size() == 1;
    for (const auto& group : groups) {
        std::string robot_state_name = robot_sn + "_" + joint_group_name_string(group);
        if (group == flexiv::rdk::JointGroup::ARM_1) {
            robot_state_name = single_arm ? robot_sn : "left_" + robot_sn;
        } else if (group == flexiv::rdk::JointGroup::ARM_2) {
            robot_state_name = "right_" + robot_sn;
        } else if (group == flexiv::rdk::JointGroup::ARMS && !single_arm) {
            robot_state_name = robot_sn;
        }

        auto state_storage
            = hw_flexiv_robot_states_by_group_.emplace(group, flexiv::rdk::RobotStates {}).first;
        auto handle_storage
            = hw_flexiv_robot_state_handles_by_group_
                  .emplace(group, register_robot_states_handle(&state_storage->second))
                  .first;
        state_interfaces.push_back(interfaces_.BindState(
            robot_state_name, "flexiv_robot_states", &handle_storage->second));
    }

    const std::string gpio_interface_name = robot_sn + "_gpio";
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.push_back(interfaces_.BindState(
            gpio_interface_name, "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    // The TCP belongs to the arm, so it is named like the arm's joints
    for (size_t arm = 0; arm < arm_prefixes_.size(); ++arm) {
        for (size_t i = 0; i < flexiv::rdk::kPoseSize; i++) {
            state_interfaces.push_back(interfaces_.BindState(arm_prefixes_[arm] + "tcp",
                kCartesianPoseInterfaces[i], &hw_states_cartesian_pose_[arm][i]));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
FlexivHardwareInterface::on_export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.push_back(interfaces_.BindCommand(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.push_back(interfaces_.BindCommand(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.push_back(interfaces_.BindCommand(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    const std::string gpio_interface_name = robot_sn + "_gpio";
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.push_back(interfaces_.BindCommand(
            gpio_interface_name, "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
    }

    for (size_t arm = 0; arm < arm_prefixes_.size(); ++arm) {
        const std::string name = arm_prefixes_[arm] + "tcp";
        for (size_t i = 0; i < flexiv::rdk::kPoseSize; i++) {
            command_interfaces.push_back(interfaces_.BindCommand(
                name, kCartesianPoseInterfaces[i], &hw_commands_cartesian_pose_[arm][i]));
        }
        for (size_t i = 0; i < flexiv::rdk::kCartDoF; i++) {
            command_interfaces.push_back(interfaces_.BindCommand(
                name, kCartesianWrenchInterfaces[i], &hw_commands_cartesian_wrench_[arm][i]));
            command_interfaces.push_back(interfaces_.BindCommand(
                name, kCartesianVelocityInterfaces[i], &hw_commands_cartesian_velocity_[arm][i]));
        }
    }

    return command_interfaces;
}

bool FlexivHardwareInterface::WaitUntilOperational(std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_log = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        if (robot_->operational()) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= next_log) {
            RCLCPP_INFO(getLogger(), "Waiting for the robot to become operational: %s",
                OperationalStatusName(robot_->operational_status()).c_str());
            next_log = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        }
        std::this_thread::sleep_for(kOperationalPollPeriod);
    }
    return robot_->operational();
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
        // Report the condition through the classifier, so that a pressed E-stop or a robot left in
        // Manual mode names the operator action instead of surfacing as an RDK exception.
        const auto condition = robot_system_control_->condition();
        const auto policy = ClassifyRecoveryPolicy(condition);
        if (policy != RecoveryPolicy::NONE) {
            RCLCPP_WARN(getLogger(), "%s", DescribeRobotCondition(condition).c_str());
        }
        if (policy == RecoveryPolicy::SAFETY_LOCKOUT || policy == RecoveryPolicy::WAIT_OPERATOR) {
            RCLCPP_FATAL(
                getLogger(), "Cannot start: %s", DescribeRobotCondition(condition).c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Clear fault on robot server if any
        if (robot_->fault()) {
            RCLCPP_WARN(getLogger(), "Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            if (!robot_->ClearFault()) {
                RCLCPP_FATAL(getLogger(), "Fault cannot be cleared, exiting ...");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(getLogger(), "Fault on robot server is cleared");
        }

        // Servo on the robot (release brakes and become operational)
        RCLCPP_INFO(getLogger(), "Servoing on robot ...");
        robot_->ServoOn();

        // Wait for the robot to become operational, bounded so that a robot that never becomes
        // ready fails the activation instead of hanging the controller manager forever.
        if (!WaitUntilOperational(kActivationOperationalTimeout)) {
            RCLCPP_FATAL(getLogger(), "Robot did not become operational within %ld s. %s",
                static_cast<long>(kActivationOperationalTimeout.count()),
                DescribeRobotCondition(robot_system_control_->condition()).c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable robot.");
        RCLCPP_FATAL(getLogger(), "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // The robot is servoed on but in IDLE with no joint group claimed: a controller start has to
    // establish the control mode and synchronize the command buffers before any motion may be
    // streamed.
    claimed_interfaces_.fill(kInterfaceNone);
    cartesian_claimed_ = false;
    std::fill(target_pos_.begin(), target_pos_.end(), std::numeric_limits<double>::quiet_NaN());
    driver_status_->commands_synchronized.store(false);
    driver_status_->driver_state.store(DriverState::READY);

    RCLCPP_INFO(getLogger(), "System successfully started!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    // Hold off write() before stopping, so the real-time loop cannot stream a command into a robot
    // that is being brought to a halt.
    driver_status_->driver_state.store(DriverState::FAULT);
    claimed_interfaces_.fill(kInterfaceNone);
    cartesian_claimed_ = false;

    try {
        StopIfOperational();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not stop the robot: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!robot_system_control_) {
        return hardware_interface::return_type::ERROR;
    }

    // Latch the robot condition for the recovery node. These are all non-blocking accessors over
    // cached state, so they are safe to poll from the real-time loop.
    driver_status_->Latch(*robot_system_control_);

    // Recovery owns the driver state while it runs, so this is a no-op for its duration.
    driver_status_->TryApplyDerivedDriverState();

    // A lost connection is the only condition the driver cannot report or recover from in place,
    // so it is the only one escalated to the controller manager. Every other fault keeps the
    // component ACTIVE, which keeps the status topic and the recovery action reachable.
    if (!driver_status_->connected.load()) {
        RCLCPP_ERROR(getLogger(), "Lost connection with the robot");
        return hardware_interface::return_type::ERROR;
    }

    if (driver_status_->operational.load()) {
        const auto states_by_group = robot_->states();
        if (states_by_group.empty()) {
            interfaces_.WriteStates();
            return hardware_interface::return_type::OK;
        }

        // Read joint states one joint group at a time, mapping RDK order onto ROS order.
        size_t offset = 0;
        for (const auto& [group, group_dof] : active_groups_) {
            const auto it = states_by_group.find(group);
            if (it == states_by_group.end()) {
                RCLCPP_ERROR_THROTTLE(getLogger(), log_clock_, 1000,
                    "Joint group %s is missing from the robot states",
                    joint_group_name_string(group).c_str());
                return hardware_interface::return_type::ERROR;
            }

            const auto& group_states = it->second;
            if (group_states.q.size() < group_dof || group_states.dtheta.size() < group_dof
                || group_states.tau.size() < group_dof) {
                RCLCPP_ERROR_THROTTLE(getLogger(), log_clock_, 1000,
                    "Joint group %s state vector size mismatch (q=%ld dtheta=%ld tau=%ld "
                    "expected=%ld)",
                    joint_group_name_string(group).c_str(), group_states.q.size(),
                    group_states.dtheta.size(), group_states.tau.size(), group_dof);
                return hardware_interface::return_type::ERROR;
            }

            for (size_t k = 0; k < group_dof; ++k) {
                const size_t ros_idx = rdk_to_ros_map_[offset + k];
                hw_states_joint_positions_[ros_idx] = group_states.q[k];
                hw_states_joint_velocities_[ros_idx] = group_states.dtheta[k];
                hw_states_joint_efforts_[ros_idx] = group_states.tau[k];
            }
            offset += group_dof;
        }

        for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
            hw_states_cartesian_pose_[arm]
                = states_by_group.at(active_groups_[arm_groups_[arm]].first).tcp_pose;
        }

        // Refresh every exported per-group robot-states handle the robot reports, so the
        // robot-states broadcaster(s) publish fresh data regardless of which group name they read:
        // a single-arm broadcaster reads the ARM_1 (robot_sn) handle, and dual-arm broadcasters
        // read the ARM_1/ARM_2 (left_/right_) handles.
        for (auto& [group, group_states] : hw_flexiv_robot_states_by_group_) {
            const auto it = states_by_group.find(group);
            if (it != states_by_group.end()) {
                group_states = it->second;
            }
        }

        // Read GPIO input states
        auto gpio_in = robot_->digital_inputs();
        for (size_t i = 0; i < hw_states_gpio_in_.size(); i++) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_in[i]);
        }
    }

    TrackPositionChangeAcrossInterruption();

    interfaces_.WriteStates();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    // Issue no RDK call unless the robot is ready. While recovery runs it changes the control mode
    // and the fault state, and this early return is what guarantees the real-time loop is
    // quiescent for the duration without needing a lock on the hot path.
    if (!robot_ || driver_status_->driver_state.load() != DriverState::READY) {
        return hardware_interface::return_type::OK;
    }

    interfaces_.ReadCommands();

    // Withhold motion until a controller restart has re-synchronized the command buffers.
    if (driver_status_->commands_synchronized.load()) {
        // A fault is picked up by read() and recovered as usual. Anything else stops streaming
        // until the controller is restarted; the robot holds its position meanwhile.
        const auto withhold_motion = [this](const char* reason) {
            driver_status_->commands_synchronized.store(false);
            RCLCPP_ERROR(
                getLogger(), "Motion is withheld until the controller is restarted: %s", reason);
        };
        try {
            // The RDK only warns once real-time commands arrive late too often, it does not throw.
            // Act on the flag only once it was seen clear, since it is still raised from before a
            // restart until about a second of commands has arrived on time.
            if (SendMotionCommands(period.seconds())) {
                if (!robot_->reached_timeliness_failure_limit()) {
                    timeliness_clear_since_sync_ = true;
                } else if (timeliness_clear_since_sync_ && withhold_on_timeliness_failure_) {
                    withhold_motion(
                        "too many real-time commands arrived late. The host cannot keep up "
                        "with the control rate, run the controller manager at real-time "
                        "priority.");
                } else if (timeliness_clear_since_sync_) {
                    // Warned once until the flag clears again
                    timeliness_clear_since_sync_ = false;
                    RCLCPP_WARN(getLogger(),
                        "Too many real-time commands arrived late. Streaming continues as "
                        "withhold_on_timeliness_failure is false, run the controller manager at "
                        "real-time priority.");
                }
            }
        } catch (const std::exception& e) {
            withhold_motion(e.what());
        }
    }

    // Write digital output
    std::map<unsigned int, bool> digital_outputs;
    for (size_t i = 0; i < hw_commands_gpio_out_.size(); i++) {
        if (!std::isfinite(hw_commands_gpio_out_[i])) {
            continue;
        }
        digital_outputs[i] = static_cast<bool>(hw_commands_gpio_out_[i]);
    }
    // Check if there are changes in the digital output values. A port never sent counts as low.
    bool digital_outputs_changed = false;
    for (const auto& [index, value] : digital_outputs) {
        if (current_digital_outputs_[index] != value) {
            current_digital_outputs_[index] = value;
            digital_outputs_changed = true;
        }
    }

    // Set digital outputs
    if (digital_outputs_changed) {
        try {
            robot_->SetDigitalOutputs(digital_outputs);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                getLogger(), log_clock_, 5000, "Failed to set digital outputs: %s", e.what());
        }
    }

    return hardware_interface::return_type::OK;
}

bool FlexivHardwareInterface::SendMotionCommands(double dt)
{
    const auto required_mode = RequiredRdkMode();
    if (required_mode == flexiv::rdk::Mode::UNKNOWN || robot_->mode() != required_mode) {
        return false;
    }
    return cartesian_claimed_ ? SendCartesianCommands() : SendJointCommands(dt);
}

bool FlexivHardwareInterface::SendJointCommands(double dt)
{
    const bool torque_control = RequiredRdkMode() == flexiv::rdk::Mode::RT_JOINT_TORQUE;

    if (torque_control) {
        // Every joint group is claimed with the effort interface, see
        // prepare_command_mode_switch(). A robot cannot hold without torque commands, so nothing
        // is streamed until every joint has one.
        if (!std::all_of(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
                [](double v) { return std::isfinite(v); })) {
            return false;
        }
        size_t offset = 0;
        for (const auto& [group, group_dof] : active_groups_) {
            auto& tau_d = rt_joint_torque_cmds_.at(group).tau_d;
            for (size_t k = 0; k < group_dof; ++k) {
                tau_d[k] = hw_commands_joint_efforts_[rdk_to_ros_map_[offset + k]];
            }
            offset += group_dof;
        }
        robot_->StreamJointTorque(rt_joint_torque_cmds_);
        return true;
    }

    // Motion control: each joint group follows its own controller's command. A group that is not
    // claimed, or whose command is not finite, holds the last target it was given, which a
    // real-time mode has to keep streaming.
    std::array<bool, kMaxJointGroups> commanded {};
    size_t offset = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        const size_t group_dof = active_groups_[g].second;
        const auto claim = claimed_interfaces_[g];

        bool finite = claim == kInterfacePosition || claim == kInterfaceVelocity;
        for (size_t k = 0; k < group_dof && finite; ++k) {
            const size_t ros_idx = rdk_to_ros_map_[offset + k];
            finite = claim == kInterfacePosition
                         ? std::isfinite(hw_commands_joint_positions_[ros_idx])
                         : std::isfinite(hw_commands_joint_velocities_[ros_idx])
                               && std::isfinite(hw_states_joint_positions_[ros_idx]);
        }

        if (finite && claim == kInterfaceVelocity) {
            // The target moves ahead of the robot at the commanded velocity. The velocity is the
            // one to have on arriving at the target, or the feed-forward in a real-time mode.
            AdvanceVelocityTargets(g, dt);
        } else if (claim == kInterfaceVelocity) {
            // Velocity control resumes from the measured position after NaN commands
            for (size_t k = 0; k < group_dof; ++k) {
                const size_t ros_idx = rdk_to_ros_map_[offset + k];
                velocity_targets_[ros_idx] = hw_states_joint_positions_[ros_idx];
            }
        }

        for (size_t k = 0; k < group_dof; ++k) {
            const size_t rdk_idx = offset + k;
            const size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            if (finite && claim == kInterfacePosition) {
                target_pos_[rdk_idx] = hw_commands_joint_positions_[ros_idx];
                // The target velocity stays zero. In the non-real-time mode it is the velocity to
                // have on arriving at the target, and passing on that of a 1 kHz stream makes the
                // robot overshoot.
                target_vel_[rdk_idx] = 0.0;
            } else if (finite) {
                target_pos_[rdk_idx] = velocity_targets_[ros_idx];
                target_vel_[rdk_idx] = hw_commands_joint_velocities_[ros_idx];
            } else {
                if (!std::isfinite(target_pos_[rdk_idx])) {
                    // Seed the hold target from the measured position, on the first cycle and after
                    // a Stop()/SwitchMode() invalidated it.
                    target_pos_[rdk_idx] = hw_states_joint_positions_[ros_idx];
                    if (!std::isfinite(target_pos_[rdk_idx])) {
                        return false;
                    }
                }
                target_vel_[rdk_idx] = 0.0;
            }
        }
        commanded[g] = finite;
        offset += group_dof;
    }

    if (rdk_realtime_) {
        offset = 0;
        for (const auto& [group, group_dof] : active_groups_) {
            auto& cmd = rt_joint_position_cmds_.at(group);
            const auto begin = static_cast<std::ptrdiff_t>(offset);
            std::copy_n(target_pos_.begin() + begin, group_dof, cmd.q_d.begin());
            std::copy_n(target_vel_.begin() + begin, group_dof, cmd.dq_d.begin());
            offset += group_dof;
        }
        robot_->StreamJointPosition(rt_joint_position_cmds_);
        return true;
    }

    // The non-real-time mode's motion generator holds a group that is sent nothing, so only the
    // groups with a fresh command are sent.
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtJointPositionCmd> nrt_cmds;
    offset = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        const auto& [group, group_dof] = active_groups_[g];
        if (commanded[g]) {
            auto& cmd = nrt_joint_position_cmds_.at(group);
            const auto begin = static_cast<std::ptrdiff_t>(offset);
            std::copy_n(target_pos_.begin() + begin, group_dof, cmd.q_d.begin());
            std::copy_n(target_vel_.begin() + begin, group_dof, cmd.dq_d.begin());
            nrt_cmds.emplace(group, cmd);
        }
        offset += group_dof;
    }
    if (!nrt_cmds.empty()) {
        robot_->SendJointPosition(nrt_cmds);
    }
    return false;
}

bool FlexivHardwareInterface::SendCartesianCommands()
{
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtCartesianCmd> nrt_cmds;
    for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
        const auto group = active_groups_[arm_groups_[arm]].first;
        const bool finite = all_finite(hw_commands_cartesian_pose_[arm])
                            && all_finite(hw_commands_cartesian_wrench_[arm])
                            && all_finite(hw_commands_cartesian_velocity_[arm]);
        if (rdk_realtime_) {
            // A real-time mode needs a command every cycle, so hold the last pose without force
            auto& cmd = rt_cartesian_cmds_.at(group);
            if (finite) {
                last_cartesian_target_[arm] = hw_commands_cartesian_pose_[arm];
                cmd.pose_d = hw_commands_cartesian_pose_[arm];
                cmd.wrench_d = hw_commands_cartesian_wrench_[arm];
                cmd.twist_d = hw_commands_cartesian_velocity_[arm];
            } else {
                cmd.pose_d = last_cartesian_target_[arm];
                cmd.wrench_d.fill(0.0);
                cmd.twist_d.fill(0.0);
            }
        } else if (finite) {
            auto& cmd = nrt_cartesian_cmds_.at(group);
            cmd.pose_d = hw_commands_cartesian_pose_[arm];
            cmd.wrench_d = hw_commands_cartesian_wrench_[arm];
            cmd.twist_d = hw_commands_cartesian_velocity_[arm];
            cmd.max_linear_vel = cartesian_max_linear_vel_[arm].load();
            cmd.max_angular_vel = cartesian_max_angular_vel_[arm].load();
            cmd.max_linear_acc = cartesian_max_linear_acc_[arm].load();
            cmd.max_angular_acc = cartesian_max_angular_acc_[arm].load();
            nrt_cmds.emplace(group, cmd);
        }
    }

    if (rdk_realtime_) {
        robot_->StreamCartesianMotionForce(rt_cartesian_cmds_);
        return true;
    }
    if (!nrt_cmds.empty()) {
        robot_->SendCartesianMotionForce(nrt_cmds);
    }
    return false;
}

void FlexivHardwareInterface::AdvanceVelocityTargets(size_t g, double dt)
{
    size_t offset = 0;
    for (size_t i = 0; i < g; ++i) {
        offset += active_groups_[i].second;
    }
    for (size_t k = 0; k < active_groups_[g].second; ++k) {
        const size_t i = rdk_to_ros_map_[offset + k];
        const double measured = hw_states_joint_positions_[i];
        velocity_targets_[i]
            = std::clamp(velocity_targets_[i] + hw_commands_joint_velocities_[i] * dt,
                measured - kMaxVelocityTargetLead, measured + kMaxVelocityTargetLead);
    }
}

void FlexivHardwareInterface::SynchronizeCommandsWithState(
    const std::array<bool, kMaxJointGroups>& groups, bool cartesian)
{
    // Called from perform_command_mode_switch(), which is the controller restart the driver
    // requires after a fault. Once the buffers hold the measured state, motion may stream again.
    driver_status_->commands_synchronized.store(true);
    // Refreshed first, so that writing the buffers back below changes only what is set here
    interfaces_.ReadCommands();

    size_t offset = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        if (groups[g]) {
            for (size_t k = 0; k < active_groups_[g].second; ++k) {
                const size_t rdk_idx = offset + k;
                const size_t ros_idx = rdk_to_ros_map_[rdk_idx];
                // Position commands start from where the robot actually is, so the first write()
                // after a mode switch commands a hold instead of whatever setpoint was left over.
                hw_commands_joint_positions_[ros_idx] = hw_states_joint_positions_[ros_idx];
                hw_commands_joint_velocities_[ros_idx] = 0.0;
                velocity_targets_[ros_idx] = hw_states_joint_positions_[ros_idx];
                target_pos_[rdk_idx] = hw_states_joint_positions_[ros_idx];
                // Effort commands are deliberately left as NaN rather than zeroed. write() skips
                // streaming while they are NaN, whereas a zero torque command is streamed with
                // gravity compensation enabled and would leave the arm floating freely.
                hw_commands_joint_efforts_[ros_idx] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        offset += active_groups_[g].second;
    }

    // The Cartesian commands likewise start as a hold at the measured TCP poses, without force.
    if (cartesian) {
        for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
            hw_commands_cartesian_pose_[arm] = hw_states_cartesian_pose_[arm];
            hw_commands_cartesian_wrench_[arm].fill(0.0);
            hw_commands_cartesian_velocity_[arm].fill(0.0);
            last_cartesian_target_[arm] = hw_states_cartesian_pose_[arm];
        }
    }

    timeliness_clear_since_sync_ = false;

    interfaces_.WriteCommands();
}

bool FlexivHardwareInterface::ResolveClaimedGroups(
    const std::vector<std::string>& keys, JointGroupClaims& claimed) const
{
    claimed.fill(kInterfaceNone);
    std::array<size_t, kMaxJointGroups> claimed_counts {};

    size_t offset = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        for (size_t k = 0; k < active_groups_[g].second; ++k) {
            const std::string& joint_name = info_.joints[rdk_to_ros_map_[offset + k]].name;
            for (const auto& key : keys) {
                uint8_t interface_type = kInterfaceNone;
                if (key == joint_name + "/" + hardware_interface::HW_IF_POSITION) {
                    interface_type = kInterfacePosition;
                } else if (key == joint_name + "/" + hardware_interface::HW_IF_VELOCITY) {
                    interface_type = kInterfaceVelocity;
                } else if (key == joint_name + "/" + hardware_interface::HW_IF_EFFORT) {
                    interface_type = kInterfaceEffort;
                } else {
                    continue;
                }

                if (claimed[g] == kInterfaceNone) {
                    claimed[g] = interface_type;
                } else if (claimed[g] != interface_type) {
                    RCLCPP_ERROR(getLogger(),
                        "Joint group %s would be claimed with more than one command interface type "
                        "at once. All joints of one arm or external axis group must use the same "
                        "interface type.",
                        joint_group_name_string(active_groups_[g].first).c_str());
                    return false;
                }
                claimed_counts[g]++;
            }
        }
        offset += active_groups_[g].second;
    }

    // A joint group is the smallest unit RDK accepts a command for, so it must be claimed whole.
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        if (claimed_counts[g] != 0 && claimed_counts[g] != active_groups_[g].second) {
            RCLCPP_ERROR(getLogger(),
                "Joint group %s would have %zu of its %zu joints claimed. Claim all joints of a "
                "group, or none of them.",
                joint_group_name_string(active_groups_[g].first).c_str(), claimed_counts[g],
                active_groups_[g].second);
            return false;
        }
    }

    return true;
}

size_t FlexivHardwareInterface::CountCartesianInterfaces(const std::vector<std::string>& keys) const
{
    return static_cast<size_t>(std::count_if(keys.begin(), keys.end(), [this](const auto& key) {
        return std::find(cartesian_command_interface_names_.begin(),
                   cartesian_command_interface_names_.end(), key)
               != cartesian_command_interface_names_.end();
    }));
}

flexiv::rdk::Mode FlexivHardwareInterface::RequiredRdkMode() const
{
    if (cartesian_claimed_) {
        return rdk_cartesian_mode_;
    }

    size_t effort_groups = 0;
    size_t motion_groups = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        if (claimed_interfaces_[g] == kInterfaceEffort) {
            effort_groups++;
        } else if (claimed_interfaces_[g] != kInterfaceNone) {
            motion_groups++;
        }
    }

    if (effort_groups == 0 && motion_groups == 0) {
        return flexiv::rdk::Mode::UNKNOWN;
    }
    return effort_groups != 0 ? flexiv::rdk::Mode::RT_JOINT_TORQUE : rdk_control_mode_;
}

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    // Joints are claimed per RDK joint group rather than all at once, so that each arm of a
    // dual-arm robot can be driven by its own controller. A group must still be claimed whole and
    // with a single interface type.
    JointGroupClaims starting {};
    JointGroupClaims stopping {};
    if (!ResolveClaimedGroups(start_interfaces, starting)
        || !ResolveClaimedGroups(stop_interfaces, stopping)) {
        return hardware_interface::return_type::ERROR;
    }

    // The Cartesian command interfaces of all arms must be claimed together
    const size_t cartesian_start_count = CountCartesianInterfaces(start_interfaces);
    const size_t cartesian_stop_count = CountCartesianInterfaces(stop_interfaces);
    if ((cartesian_start_count != 0
            && cartesian_start_count != cartesian_command_interface_names_.size())
        || (cartesian_stop_count != 0
            && cartesian_stop_count != cartesian_command_interface_names_.size())) {
        RCLCPP_ERROR(getLogger(),
            "A controller must claim all %zu Cartesian command interfaces of the robot's arms",
            cartesian_command_interface_names_.size());
        return hardware_interface::return_type::ERROR;
    }
    const bool cartesian_starting = cartesian_start_count != 0;
    const bool next_cartesian
        = cartesian_starting || (cartesian_claimed_ && cartesian_stop_count == 0);

    size_t effort_groups = 0;
    size_t motion_groups = 0;
    size_t idle_groups = 0;
    bool any_joint_starting = false;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        any_joint_starting |= starting[g] != kInterfaceNone;
        switch (next_claimed_interface(claimed_interfaces_[g], starting[g], stopping[g])) {
            case kInterfaceEffort:
                effort_groups++;
                break;
            case kInterfaceNone:
                idle_groups++;
                break;
            default:
                motion_groups++;
                break;
        }
    }

    // One RDK control mode applies to the whole robot, so the Cartesian, effort and
    // position/velocity interfaces cannot be in use at the same time. Position on one arm and
    // velocity on the other is fine: both are served by the same RDK control mode.
    if (next_cartesian && (effort_groups != 0 || motion_groups != 0)) {
        RCLCPP_ERROR(getLogger(),
            "A joint controller and a Cartesian controller cannot run at the same time. Stop one "
            "before starting the other.");
        return hardware_interface::return_type::ERROR;
    }
    if (effort_groups != 0 && motion_groups != 0) {
        RCLCPP_ERROR(getLogger(),
            "The effort interface needs RDK control mode RT_JOINT_TORQUE while the "
            "position/velocity interfaces need a motion control mode, and the RDK control mode "
            "applies to the whole robot.");
        return hardware_interface::return_type::ERROR;
    }

    // An idle joint group in RT_JOINT_TORQUE can only be sent zero torque with gravity
    // compensation, i.e. left free-floating.
    if (effort_groups != 0 && idle_groups != 0) {
        RCLCPP_ERROR(getLogger(),
            "Effort control requires every joint group to be claimed, otherwise the unclaimed "
            "group(s) would be left free-floating under zero torque.");
        return hardware_interface::return_type::ERROR;
    }

    // Once the robot stopped following the controllers, after a fault or withheld motion, a start
    // resumes streaming for every joint group. A running controller not restarted with it would
    // resume from its stale setpoint, so the controllers of all groups must be restarted together.
    if ((any_joint_starting || cartesian_starting)
        && !driver_status_->commands_synchronized.load()) {
        for (size_t g = 0; g < active_groups_.size(); ++g) {
            if (claimed_interfaces_[g] != kInterfaceNone && starting[g] == kInterfaceNone
                && stopping[g] == kInterfaceNone) {
                RCLCPP_ERROR(getLogger(),
                    "The robot is not following the controllers. Restart the controllers of all "
                    "arms together, joint group %s is claimed by one that is not restarted.",
                    joint_group_name_string(active_groups_[g].first).c_str());
                return hardware_interface::return_type::ERROR;
            }
        }
    }

    // Zeroed right before the Cartesian mode is entered, off the real-time loop
    if (cartesian_starting && !ZeroForceTorqueSensors()) {
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    JointGroupClaims starting {};
    JointGroupClaims stopping {};
    if (!ResolveClaimedGroups(start_interfaces, starting)
        || !ResolveClaimedGroups(stop_interfaces, stopping)) {
        return hardware_interface::return_type::ERROR;
    }
    const bool cartesian_starting = CountCartesianInterfaces(start_interfaces) != 0;
    const bool cartesian_stopping = CountCartesianInterfaces(stop_interfaces) != 0;

    const auto previous_mode = RequiredRdkMode();
    const bool was_cartesian = cartesian_claimed_;

    std::array<bool, kMaxJointGroups> groups_to_sync {};
    bool any_change = cartesian_starting || cartesian_stopping;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        const uint8_t next_interface
            = next_claimed_interface(claimed_interfaces_[g], starting[g], stopping[g]);
        any_change |= next_interface != claimed_interfaces_[g] || starting[g] != kInterfaceNone;
        claimed_interfaces_[g] = next_interface;
        groups_to_sync[g] = starting[g] != kInterfaceNone;
    }
    cartesian_claimed_ = cartesian_starting || (cartesian_claimed_ && !cartesian_stopping);

    if (!any_change) {
        return hardware_interface::return_type::OK;
    }

    try {
        // The robot keeps passive force control across mode entries, so it is disabled once the
        // Cartesian controller is gone, which the RDK only accepts in IDLE.
        if (was_cartesian && !cartesian_claimed_) {
            StopIfOperational();
            if (cartesian_config_node_) {
                cartesian_config_node_->DisablePassiveForceControl();
            }
        }

        // Hold before user commands arrive
        if (cartesian_starting
            || std::find(groups_to_sync.begin(), groups_to_sync.end(), true)
                   != groups_to_sync.end()) {
            SynchronizeCommandsWithState(groups_to_sync, cartesian_starting);
        }

        const auto required_mode = RequiredRdkMode();
        const auto current_mode = robot_->mode();

        if (required_mode == flexiv::rdk::Mode::UNKNOWN) {
            // Nothing is claimed any more
            if (current_mode != flexiv::rdk::Mode::IDLE) {
                StopIfOperational();
            }
            std::fill(
                target_pos_.begin(), target_pos_.end(), std::numeric_limits<double>::quiet_NaN());
        } else if (current_mode != required_mode || previous_mode != required_mode) {
            // The joint groups already running in the required mode keep running: switching again
            // would interrupt them and reset the mode's settings.
            if (required_mode == rdk_cartesian_mode_) {
                // Every start begins from the defaults: the robot resets its settings on mode
                // entry, and the motion limits are reset here
                const CartesianMotionLimits defaults;
                for (size_t arm = 0; arm < arm_groups_.size(); ++arm) {
                    cartesian_max_linear_vel_[arm].store(defaults.max_linear_vel);
                    cartesian_max_angular_vel_[arm].store(defaults.max_angular_vel);
                    cartesian_max_linear_acc_[arm].store(defaults.max_linear_acc);
                    cartesian_max_angular_acc_[arm].store(defaults.max_angular_acc);
                }
            }

            // SwitchMode() stops the robot before transiting. This is also the step that brings
            // the robot back from IDLE after a fault: recovery leaves the robot operational in
            // IDLE, and restarting the controllers lands here with freshly synchronized buffers.
            robot_->SwitchMode(required_mode);

            if (required_mode == rdk_control_mode_) {
                // The robot resets its joint impedance properties on mode entry, so whatever was
                // set has to be re-applied before any motion is streamed.
                if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
                    RCLCPP_FATAL(getLogger(),
                        "Could not re-apply the joint impedance properties. The robot would run at "
                        "nominal stiffness instead of the requested one, so the controller start "
                        "is refused.");
                    driver_status_->commands_synchronized.store(false);
                    StopIfOperational();
                    return hardware_interface::return_type::ERROR;
                }
            } else if (joint_impedance_config_node_) {
                // The joint impedance properties do not govern RT_JOINT_TORQUE or the Cartesian
                // mode, so what the driver holds is no longer in effect.
                joint_impedance_config_node_->MarkNotInEffect();
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to apply the RDK control mode: %s", e.what());
        driver_status_->commands_synchronized.store(false);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)

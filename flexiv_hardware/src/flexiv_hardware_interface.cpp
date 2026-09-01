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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"
#include "flexiv_hardware/flexiv_robot_states_handle.hpp"

namespace {

constexpr uint64_t kMaxExactRobotStatesHandle = 1ULL << 53;

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
    const auto name_it = flexiv::rdk::kJointGroupNames.find(group);
    if (name_it != flexiv::rdk::kJointGroupNames.end()) {
        return name_it->second;
    }
    return "GROUP_" + std::to_string(static_cast<int>(group));
}

std::string describe_group_layout(
    const std::map<flexiv::rdk::JointGroup, flexiv::rdk::RobotStates>& states_by_group)
{
    std::ostringstream stream;
    bool first = true;
    for (const auto& [group, states] : states_by_group) {
        if (!first) {
            stream << ", ";
        }
        first = false;
        stream << joint_group_name_string(group) << "(q=" << states.q.size()
               << ", dtheta=" << states.dtheta.size() << ", tau=" << states.tau.size() << ")";
    }
    return stream.str();
}

/**
 * Resolve active joint groups from Robot::states() for this interface.
 *
 * Supported arm layouts, each optionally combined with an EXT_AXIS group (e.g. MICO-Plus,
 * MICO-Ultra torso), are: 1) Single-arm: one group ARMS. 2) Dual-arm: one group each for ARM_1 and
 * ARM_2.
 *
 * When an EXT_AXIS group is present it is placed first in the returned list, matching the
 * rdk_to_ros_map_ ordering convention (external axes precede arm joints). The DoFs of all
 * returned groups must sum to expected_dof.
 *
 * @param states_by_group Joint-group keyed robot states from RDK.
 * @param expected_dof Total DoF expected by this hardware interface mapping.
 * @return Ordered list of active groups and each group's DoF.
 *         Returns empty when the layout is unsupported or DoF does not match.
 */
GroupDofList determine_active_groups(
    const std::map<flexiv::rdk::JointGroup, flexiv::rdk::RobotStates>& states_by_group,
    size_t expected_dof, const rclcpp::Logger& logger)
{
    GroupDofList active_groups;

    // Optional external-axis group. Commanded first to match the
    // rdk_to_ros_map_ ordering [ext_axis..., arm_joint...].
    auto ext_it = states_by_group.find(flexiv::rdk::JointGroup::EXT_AXIS);
    const bool has_ext = ext_it != states_by_group.end();

    // Arm layout. A real robot's states() exposes overlapping views: the commandable per-arm
    // single-arm groups ARM_1 (and ARM_2 on a dual-arm robot) appear *alongside* the aggregate
    // views ALL and ARMS. The joint command APIs (SendJointPosition/StreamJointPosition) only
    // accept single-arm and external-axis groups and reject ALL/ARMS, so select ARM_1[/ARM_2] and
    // never the aggregates. ARMS is used only as a last-resort fallback for a robot that exposes
    // no per-arm group at all.
    auto arm1_it = states_by_group.find(flexiv::rdk::JointGroup::ARM_1);
    auto arm2_it = states_by_group.find(flexiv::rdk::JointGroup::ARM_2);
    const bool has_arm1 = arm1_it != states_by_group.end();
    const bool has_arm2 = arm2_it != states_by_group.end();

    // Assemble in RDK order: external axes first, then arm(s).
    GroupDofList candidate_groups;
    if (has_ext) {
        candidate_groups.emplace_back(flexiv::rdk::JointGroup::EXT_AXIS, ext_it->second.q.size());
    }
    if (has_arm1 && has_arm2) {
        // Dual-arm: two commandable single-arm groups.
        candidate_groups.emplace_back(flexiv::rdk::JointGroup::ARM_1, arm1_it->second.q.size());
        candidate_groups.emplace_back(flexiv::rdk::JointGroup::ARM_2, arm2_it->second.q.size());
    } else if (has_arm1) {
        // Single-arm: ARM_1 is the commandable group (ARMS is only an aggregate view).
        candidate_groups.emplace_back(flexiv::rdk::JointGroup::ARM_1, arm1_it->second.q.size());
    } else {
        RCLCPP_ERROR(logger,
            "Unsupported joint-group combination returned by robot states: %s. Expected a "
            "commandable ARM_1[/ARM_2] group (optionally with EXT_AXIS); the aggregate ALL/ARMS "
            "views are not commandable. Expected total DoF %zu.",
            describe_group_layout(states_by_group).c_str(), expected_dof);
        return active_groups;
    }

    size_t total_dof = 0;
    for (const auto& [group, group_dof] : candidate_groups) {
        if (group_dof == 0) {
            RCLCPP_ERROR(logger, "Joint group '%s' reported 0 joints. Layout: %s",
                joint_group_name_string(group).c_str(),
                describe_group_layout(states_by_group).c_str());
            return active_groups;
        }
        total_dof += group_dof;
    }

    if (total_dof != expected_dof) {
        RCLCPP_ERROR(logger,
            "Robot joint groups report %zu total joints, but hardware interface expects %zu. "
            "Layout: %s",
            total_dof, expected_dof, describe_group_layout(states_by_group).c_str());
        return active_groups;
    }

    active_groups = std::move(candidate_groups);
    return active_groups;
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
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info)
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
    target_pos_buffer_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    target_vel_buffer_.resize(info_.joints.size(), 0.0);
    target_torque_buffer_.resize(info_.joints.size(), 0.0);
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    rt_joint_position_cmds_.clear();
    rt_joint_torque_cmds_.clear();
    active_groups_.clear();
    claimed_interfaces_.fill(kInterfaceNone);
    stop_modes_ = {};
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;
    controllers_initialized_ = false;

    if (info_.joints.size() < 7) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected at least 7.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto prefix = get_optional_hardware_parameter(info_, "prefix");
    const auto prefix_left = get_optional_hardware_parameter(info_, "prefix_left");
    const auto prefix_right = get_optional_hardware_parameter(info_, "prefix_right");

    std::vector<std::string> arm_prefixes;
    if (!prefix.empty()) {
        if (!prefix_left.empty() || !prefix_right.empty()) {
            RCLCPP_FATAL(getLogger(),
                "Parameters 'prefix' and 'prefix_left'/'prefix_right' are mutually exclusive");
            return hardware_interface::CallbackReturn::ERROR;
        }
        arm_prefixes.push_back(prefix);
    } else if (!prefix_left.empty() && !prefix_right.empty()) {
        arm_prefixes.push_back(prefix_left);
        arm_prefixes.push_back(prefix_right);
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
    for (const auto& arm_prefix : arm_prefixes) {
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
    // contiguous slice of it.
    // Order must match determine_active_groups(): external axes first, then arm 1, then arm 2.
    std::vector<size_t> group_dofs;
    if (!ext_indices.empty()) {
        group_dofs.push_back(ext_indices.size());
    }
    for (size_t i = 0; i < arm_prefixes.size(); ++i) {
        group_dofs.push_back(flexiv::rdk::kSerialJointDoF);
    }

    if (group_dofs.size() > kMaxJointGroups) {
        RCLCPP_FATAL(getLogger(), "Resolved %zu joint groups. Expected at most %zu.",
            group_dofs.size(), kMaxJointGroups);
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t total_group_dof = 0;
    for (const auto group_dof : group_dofs) {
        active_groups_.emplace_back(flexiv::rdk::JointGroup::UNKNOWN, group_dof);
        total_group_dof += group_dof;
    }

    if (total_group_dof != rdk_to_ros_map_.size()) {
        RCLCPP_FATAL(getLogger(), "Joint groups cover %zu of %zu joints", total_group_dof,
            rdk_to_ros_map_.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string robot_sn;
    try {
        robot_sn = info_.hardware_parameters["robot_sn"];
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

    try {
        auto rdk_control_mode_str = info_.hardware_parameters.at("rdk_control_mode");
        if (rdk_control_mode_str == "joint_position") {
            rdk_control_mode_ = flexiv::rdk::Mode::RT_JOINT_POSITION;
        } else if (rdk_control_mode_str == "joint_impedance") {
            rdk_control_mode_ = flexiv::rdk::Mode::RT_JOINT_IMPEDANCE;
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

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return hardware_interface::CallbackReturn::SUCCESS;
}

const rclcpp::Logger& FlexivHardwareInterface::getLogger()
{
    static const rclcpp::Logger logger = rclcpp::get_logger("FlexivHardwareInterface");
    return logger;
}

std::vector<hardware_interface::StateInterface> FlexivHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
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

    for (const auto& group : groups) {
        std::string robot_state_name = robot_sn;
        if (group == flexiv::rdk::JointGroup::ARM_1) {
            robot_state_name = "left_" + robot_sn;
        } else if (group == flexiv::rdk::JointGroup::ARM_2) {
            robot_state_name = "right_" + robot_sn;
        } else if (group != flexiv::rdk::JointGroup::ARMS) {
            robot_state_name = robot_sn + "_" + joint_group_name_string(group);
        }

        auto state_storage
            = hw_flexiv_robot_states_by_group_.emplace(group, flexiv::rdk::RobotStates {}).first;
        auto handle_storage
            = hw_flexiv_robot_state_handles_by_group_
                  .emplace(group, register_robot_states_handle(&state_storage->second))
                  .first;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            robot_state_name, "flexiv_robot_states", &handle_storage->second));
    }

    const std::string gpio_interface_name = robot_sn + "_gpio";
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio_interface_name, "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    const std::string gpio_interface_name = robot_sn + "_gpio";
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio_interface_name, "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
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

        const auto robot_info = robot_->info();
        size_t robot_total_dof = 0;
        for (const auto& [group, name] : robot_info.single_arm_groups) {
            const auto dof_it = robot_info.DoF.find(group);
            if (dof_it != robot_info.DoF.end()) {
                robot_total_dof += dof_it->second;
            }
        }
        const auto ext_dof_it = robot_info.DoF.find(flexiv::rdk::JointGroup::EXT_AXIS);
        if (ext_dof_it != robot_info.DoF.end()) {
            robot_total_dof += ext_dof_it->second;
        }
        if (robot_total_dof != info_.joints.size()) {
            RCLCPP_FATAL(getLogger(), "Robot has %zu commandable DoF. Expected %zu (from URDF).",
                robot_total_dof, info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Servo on the robot (release brakes and become operational)
        RCLCPP_INFO(getLogger(), "Servoing on robot ...");
        robot_->ServoOn();

        // Wait for the robot to become operational
        while (!robot_->operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");

        GroupDofList robot_groups;
        if (ext_dof_it != robot_info.DoF.end() && ext_dof_it->second > 0) {
            robot_groups.emplace_back(flexiv::rdk::JointGroup::EXT_AXIS, ext_dof_it->second);
        }
        for (const auto& [group, name] : robot_info.single_arm_groups) {
            const auto dof_it = robot_info.DoF.find(group);
            if (dof_it != robot_info.DoF.end() && dof_it->second > 0) {
                robot_groups.emplace_back(group, dof_it->second);
            }
        }
        if (robot_groups.empty()) {
            for (int attempt = 0; attempt < 100 && robot_groups.empty(); ++attempt) {
                robot_groups = determine_active_groups(
                    robot_->states(), rdk_to_ros_map_.size(), getLogger());
                if (robot_groups.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }

        if (robot_groups.size() != active_groups_.size()) {
            RCLCPP_FATAL(getLogger(),
                "Robot reports %zu commandable joint groups, but the URDF resolved %zu",
                robot_groups.size(), active_groups_.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        size_t group_offset = 0;
        for (size_t g = 0; g < active_groups_.size(); ++g) {
            if (robot_groups[g].second != active_groups_[g].second) {
                RCLCPP_FATAL(getLogger(),
                    "Joint group %s has %zu joints on the robot, but %zu in the URDF",
                    joint_group_name_string(robot_groups[g].first).c_str(), robot_groups[g].second,
                    active_groups_[g].second);
                return hardware_interface::CallbackReturn::ERROR;
            }
            active_groups_[g].first = robot_groups[g].first;
            RCLCPP_INFO(getLogger(), "Joint group %s drives %zu joints, starting at '%s'",
                joint_group_name_string(active_groups_[g].first).c_str(), active_groups_[g].second,
                info_.joints[rdk_to_ros_map_[group_offset]].name.c_str());
            group_offset += active_groups_[g].second;
        }

        rt_joint_position_cmds_.clear();
        rt_joint_torque_cmds_.clear();
        for (const auto& [group, group_dof] : active_groups_) {
            auto& pos_cmd = rt_joint_position_cmds_[group];
            pos_cmd.q_d.assign(group_dof, 0.0);
            pos_cmd.dq_d.assign(group_dof, 0.0);
            pos_cmd.ddq_d.assign(group_dof, 0.0);

            auto& torque_cmd = rt_joint_torque_cmds_[group];
            torque_cmd.tau_d.assign(group_dof, 0.0);
            torque_cmd.enable_gravity_comp = true;
            torque_cmd.enable_soft_limits = true;
        }

        // Start from a known state: no joint group claimed, robot idle, every hold target latched.
        if (robot_->mode() != flexiv::rdk::Mode::IDLE) {
            robot_->Stop();
        }
        claimed_interfaces_.fill(kInterfaceNone);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable robot.");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully started!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    robot_->Stop();
    claimed_interfaces_.fill(kInterfaceNone);

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_ERROR(getLogger(), "Hardware interface entered the error state, stopping the robot");

    claimed_interfaces_.fill(kInterfaceNone);

    if (robot_) {
        try {
            robot_->Stop();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(getLogger(), "Failed to stop the robot: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

bool FlexivHardwareInterface::resolve_claimed_groups(
    const std::vector<std::string>& keys, std::array<uint8_t, kMaxJointGroups>& claimed) const
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

flexiv::rdk::Mode FlexivHardwareInterface::required_rdk_mode() const
{
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

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!robot_->operational()) {
        return hardware_interface::return_type::OK;
    }

    const auto states_by_group = robot_->states();
    if (states_by_group.empty()) {
        return hardware_interface::return_type::OK;
    }

    // Read joint states straight into the exported state interfaces, one joint group at a time,
    // mapping RDK order onto ROS order as we go.
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
                "Joint group %s state vector size mismatch (q=%ld dtheta=%ld tau=%ld expected=%ld)",
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

    // a single-arm broadcaster reads the ARMS-named (robot_sn) handle while commands use ARM_1,
    // and dual-arm broadcasters read the ARM_1/ARM_2 (left_/right_) handles.
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

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    const auto required_mode = required_rdk_mode();
    if (required_mode != flexiv::rdk::Mode::UNKNOWN && robot_->mode() != required_mode) {
        RCLCPP_ERROR_THROTTLE(getLogger(), log_clock_, 1000,
            "Robot is no longer in the expected RDK control mode, skipping joint commands");
        return hardware_interface::return_type::ERROR;
    }

    const bool torque_control = required_mode == flexiv::rdk::Mode::RT_JOINT_TORQUE;

    auto& target_pos = target_pos_buffer_;
    auto& target_vel = target_vel_buffer_;
    auto& target_torque = target_torque_buffer_;

    size_t commanded_groups = 0;
    bool targets_valid = true;
    size_t offset = 0;
    for (size_t g = 0; required_mode != flexiv::rdk::Mode::UNKNOWN && g < active_groups_.size();
         ++g) {
        const auto& [group, group_dof] = active_groups_[g];
        const auto begin = static_cast<std::ptrdiff_t>(offset);

        if (torque_control) {
            bool commanded = claimed_interfaces_[g] == kInterfaceEffort;
            for (size_t k = 0; k < group_dof && commanded; ++k) {
                const double tau = hw_commands_joint_efforts_[rdk_to_ros_map_[offset + k]];
                commanded = std::isfinite(tau);
                target_torque[offset + k] = tau;
            }
            if (!commanded) {
                targets_valid = false;
                break;
            }
            std::copy_n(target_torque.begin() + begin, group_dof,
                rt_joint_torque_cmds_.at(group).tau_d.begin());
            commanded_groups++;
            offset += group_dof;
            continue;
        }

        // Check the whole group before writing any target, so a group that is not fully commanded
        // keeps the previous targets it is holding at.
        bool commanded = claimed_interfaces_[g] == kInterfacePosition
                         || claimed_interfaces_[g] == kInterfaceVelocity;
        for (size_t k = 0; k < group_dof && commanded; ++k) {
            const size_t ros_idx = rdk_to_ros_map_[offset + k];
            commanded = claimed_interfaces_[g] == kInterfacePosition
                            ? std::isfinite(hw_commands_joint_positions_[ros_idx])
                            : std::isfinite(hw_commands_joint_velocities_[ros_idx])
                                  && std::isfinite(hw_states_joint_positions_[ros_idx]);
        }

        if (commanded) {
            for (size_t k = 0; k < group_dof; ++k) {
                const size_t ros_idx = rdk_to_ros_map_[offset + k];
                if (claimed_interfaces_[g] == kInterfacePosition) {
                    target_pos[offset + k] = hw_commands_joint_positions_[ros_idx];
                    target_vel[offset + k] = 0.0;
                } else {
                    // Velocity control feeds the measured position forward with the commanded
                    // velocity, as the position interface has no velocity to track.
                    target_pos[offset + k] = hw_states_joint_positions_[ros_idx];
                    target_vel[offset + k] = hw_commands_joint_velocities_[ros_idx];
                }
            }
            commanded_groups++;
        } else {
            for (size_t k = 0; k < group_dof && targets_valid; ++k) {
                if (!std::isfinite(target_pos[offset + k])) {
                    // Seed the hold target from the measured position, both on the first cycle and
                    // after a Stop()/SwitchMode() invalidated it.
                    const double q = hw_states_joint_positions_[rdk_to_ros_map_[offset + k]];
                    targets_valid = std::isfinite(q);
                    target_pos[offset + k] = q;
                }
                target_vel[offset + k] = 0.0;
            }
            if (!targets_valid) {
                break;
            }
        }

        auto& cmd = rt_joint_position_cmds_.at(group);
        std::copy_n(target_pos.begin() + begin, group_dof, cmd.q_d.begin());
        std::copy_n(target_vel.begin() + begin, group_dof, cmd.dq_d.begin());
        offset += group_dof;
    }

    // Stream every joint group in a single call
    if (commanded_groups > 0 && targets_valid) {
        try {
            if (torque_control) {
                robot_->StreamJointTorque(rt_joint_torque_cmds_);
            } else {
                robot_->StreamJointPosition(rt_joint_position_cmds_);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                getLogger(), log_clock_, 1000, "Failed to stream joint commands: %s", e.what());
            return hardware_interface::return_type::ERROR;
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
    const bool digital_outputs_changed = digital_outputs != current_digital_outputs_;
    current_digital_outputs_ = digital_outputs;

    // Set digital outputs
    if (digital_outputs_changed && !digital_outputs.empty()) {
        try {
            robot_->SetDigitalOutputs(digital_outputs);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                getLogger(), log_clock_, 5000, "Failed to set digital outputs: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    start_modes_.clear();
    stop_modes_.clear();

    // Starting interfaces
    for (const auto& key : start_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                start_modes_.push_back(hardware_interface::HW_IF_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                start_modes_.push_back(hardware_interface::HW_IF_EFFORT);
            }
        }
    }

    // Stop motion on all relevant joints that are stopping
    for (const auto& key : stop_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                stop_modes_.push_back(StoppingInterface::STOP_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                stop_modes_.push_back(StoppingInterface::STOP_EFFORT);
            }
        }
    }

    // Joints are claimed per RDK joint group rather than all at once, so that each arm of a
    // dual-arm robot can be driven by its own controller. A group must still be claimed whole and
    // with a single interface type.
    std::array<uint8_t, kMaxJointGroups> starting {};
    std::array<uint8_t, kMaxJointGroups> stopping {};
    if (!resolve_claimed_groups(start_interfaces, starting)
        || !resolve_claimed_groups(stop_interfaces, stopping)) {
        return hardware_interface::return_type::ERROR;
    }

    size_t effort_groups = 0;
    size_t motion_groups = 0;
    size_t idle_groups = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
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

    // One RDK control mode applies to the whole robot, so the effort interface cannot be in use at
    // the same time as the position/velocity interfaces. Position on one arm and velocity on the
    // other is fine: both are served by the same RDK control mode.
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

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    std::array<uint8_t, kMaxJointGroups> starting {};
    std::array<uint8_t, kMaxJointGroups> stopping {};
    if (!resolve_claimed_groups(start_interfaces, starting)
        || !resolve_claimed_groups(stop_interfaces, stopping)) {
        return hardware_interface::return_type::ERROR;
    }

    bool any_change = false;
    size_t offset = 0;
    for (size_t g = 0; g < active_groups_.size(); ++g) {
        const uint8_t next_interface
            = next_claimed_interface(claimed_interfaces_[g], starting[g], stopping[g]);
        if (next_interface != claimed_interfaces_[g]) {
            any_change = true;
            claimed_interfaces_[g] = next_interface;

            for (size_t k = 0; k < active_groups_[g].second; ++k) {
                const size_t ros_idx = rdk_to_ros_map_[offset + k];
                hw_commands_joint_positions_[ros_idx] = std::numeric_limits<double>::quiet_NaN();
                hw_commands_joint_velocities_[ros_idx] = std::numeric_limits<double>::quiet_NaN();
                hw_commands_joint_efforts_[ros_idx] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        offset += active_groups_[g].second;
    }

    position_controller_running_
        = std::find(claimed_interfaces_.begin(), claimed_interfaces_.end(), kInterfacePosition)
          != claimed_interfaces_.end();
    velocity_controller_running_
        = std::find(claimed_interfaces_.begin(), claimed_interfaces_.end(), kInterfaceVelocity)
          != claimed_interfaces_.end();
    torque_controller_running_
        = std::find(claimed_interfaces_.begin(), claimed_interfaces_.end(), kInterfaceEffort)
          != claimed_interfaces_.end();

    start_modes_.clear();
    stop_modes_.clear();

    if (!any_change) {
        return hardware_interface::return_type::OK;
    }

    try {
        const auto required_mode = required_rdk_mode();
        const auto current_mode = robot_->mode();

        if (required_mode == flexiv::rdk::Mode::UNKNOWN) {
            // No joint group is claimed any more.
            if (current_mode != flexiv::rdk::Mode::IDLE) {
                robot_->Stop();
                std::fill(target_pos_buffer_.begin(), target_pos_buffer_.end(),
                    std::numeric_limits<double>::quiet_NaN());
            }
        } else if (current_mode != required_mode) {
            if (!robot_->operational() || robot_->fault()) {
                RCLCPP_ERROR(getLogger(),
                    "Cannot switch RDK control mode: the robot is not operational or has faulted");
                claimed_interfaces_.fill(kInterfaceNone);
                return hardware_interface::return_type::ERROR;
            }
            // SwitchMode() stops the robot before transiting, so this is only reached when no
            // joint group is being commanded in the required mode yet.
            robot_->SwitchMode(required_mode);
            std::fill(target_pos_buffer_.begin(), target_pos_buffer_.end(),
                std::numeric_limits<double>::quiet_NaN());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to apply the RDK control mode: %s", e.what());
        claimed_interfaces_.fill(kInterfaceNone);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)

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
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/fault_recovery.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"
#include "flexiv_hardware/flexiv_robot_states_handle.hpp"

namespace {

constexpr uint64_t kMaxExactRobotStatesHandle = 1ULL << 53;

// Bounded wait for the robot to become operational during activation. Brake release dominates the
// duration; a robot that is not ready within this window needs operator attention.
constexpr std::chrono::seconds kActivationOperationalTimeout {30};
constexpr std::chrono::milliseconds kOperationalPollPeriod {200};

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
    const auto& group_names = flexiv::rdk::JointGroupNames();
    const auto name_it = group_names.find(group);
    if (name_it != group_names.end()) {
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
    target_pos_buffer_.resize(info_.joints.size(), 0.0);
    target_vel_buffer_.resize(info_.joints.size(), 0.0);
    target_torque_buffer_.resize(info_.joints.size(), 0.0);
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    rt_joint_position_cmds_.clear();
    rt_joint_torque_cmds_.clear();
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

    // The connection is opened here because export_state_interfaces(), which the resource manager
    // calls right after on_init(), needs the robot's joint groups to name the per-group robot-state
    // interfaces. on_configure() re-opens it after an on_cleanup(), which is what lets a lost
    // connection be recovered without restarting the whole process.
    driver_status_ = std::make_shared<DriverStatus>();

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
        RCLCPP_FATAL(getLogger(), e.what());
        return false;
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return true;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    const std::string robot_sn = info_.hardware_parameters.at("robot_sn");

    if (!Connect()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Check the commandable DoF of the robot against the URDF before anything is enabled.
    const auto robot_info = robot_->info();
    size_t robot_total_dof = 0;
    for (const auto& [group, name] : robot_info.single_arm_groups) {
        (void)name;
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
    // Host both nodes on an executor owned here with its own spin thread, so that all blocking
    // system control calls happen off the real-time control loop.
    recovery_node_
        = std::make_shared<RecoveryNode>(robot_sn, *robot_system_control_, driver_status_);
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(recovery_node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // The impedance properties are set one joint group at a time in RDK v2.x, and only the
    // single-arm groups report a nominal joint stiffness to validate against, so the interface
    // covers the arm joints. External axes (the MICO pan-tilt torso) are left out.
    const auto robot_info = robot_->info();

    // rdk_to_ros_map_ is ordered [external axes..., ARM_1 joints..., ARM_2 joints...], matching
    // the ascending JointGroup order that single_arm_groups iterates in, so walking the arm
    // groups walks the tail of the map in step.
    const auto ext_dof_it = robot_info.DoF.find(flexiv::rdk::JointGroup::EXT_AXIS);
    size_t rdk_index
        = ext_dof_it != robot_info.DoF.end() ? ext_dof_it->second : static_cast<size_t>(0);

    impedance_groups_.clear();
    std::vector<size_t> arm_ros_indices;
    for (const auto& [group, name] : robot_info.single_arm_groups) {
        (void)name;
        const auto dof_it = robot_info.DoF.find(group);
        if (dof_it == robot_info.DoF.end()) {
            continue;
        }
        const size_t group_dof = dof_it->second;
        const size_t group_begin = rdk_index;
        rdk_index += group_dof;
        if (rdk_index > rdk_to_ros_map_.size()) {
            RCLCPP_FATAL(getLogger(),
                "Joint group '%s' runs past the %zu joints of the hardware component",
                joint_group_name_string(group).c_str(), rdk_to_ros_map_.size());
            return false;
        }

        const auto k_q_nom_it = robot_info.K_q_nom.find(group);
        if (k_q_nom_it == robot_info.K_q_nom.end() || k_q_nom_it->second.size() != group_dof) {
            RCLCPP_WARN(getLogger(),
                "Joint group '%s' reports no usable nominal joint stiffness; it is left out of the "
                "joint impedance interface",
                joint_group_name_string(group).c_str());
            continue;
        }

        impedance_groups_.emplace_back(group, group_dof);
        arm_ros_indices.insert(arm_ros_indices.end(),
            rdk_to_ros_map_.begin() + static_cast<std::ptrdiff_t>(group_begin),
            rdk_to_ros_map_.begin() + static_cast<std::ptrdiff_t>(rdk_index));
    }

    // The impedance joint list is in URDF order, matching the "one entry per arm joint in URDF
    // order" contract of the services.
    std::vector<size_t> impedance_ros_indices = arm_ros_indices;
    std::sort(impedance_ros_indices.begin(), impedance_ros_indices.end());

    impedance_rdk_to_ros_map_.clear();
    impedance_rdk_to_ros_map_.reserve(arm_ros_indices.size());
    for (const auto ros_index : arm_ros_indices) {
        const auto it = std::lower_bound(
            impedance_ros_indices.begin(), impedance_ros_indices.end(), ros_index);
        impedance_rdk_to_ros_map_.push_back(
            static_cast<size_t>(std::distance(impedance_ros_indices.begin(), it)));
    }

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

    joint_impedance_config_node_
        = std::make_shared<JointImpedanceConfigNode>(robot_sn, std::move(joint_names),
            std::move(bounds), rdk_control_mode_ == flexiv::rdk::Mode::RT_JOINT_IMPEDANCE,
            driver_status_, std::move(setters));
    executor_->add_node(joint_impedance_config_node_->get_node_base_interface());

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

void FlexivHardwareInterface::Disconnect()
{
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    if (recovery_node_) {
        if (executor_) {
            executor_->remove_node(recovery_node_->get_node_base_interface());
        }
        recovery_node_.reset();
    }
    // Torn down before robot_ below, since its closures capture this and call through it.
    if (joint_impedance_config_node_) {
        if (executor_) {
            executor_->remove_node(joint_impedance_config_node_->get_node_base_interface());
        }
        joint_impedance_config_node_.reset();
    }
    executor_.reset();
    robot_system_control_.reset();
    robot_.reset();
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
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // The robot is servoed on but in IDLE: a controller start has to establish the control mode
    // and synchronize the command buffers before any motion may be streamed.
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
        const size_t dof = rdk_to_ros_map_.size();
        auto states_by_group = robot_->states();
        if (states_by_group.empty()) {
            return hardware_interface::return_type::OK;
        }

        auto active_groups = determine_active_groups(states_by_group, dof, getLogger());
        if (active_groups.empty()) {
            return hardware_interface::return_type::ERROR;
        }

        std::vector<double> q;
        std::vector<double> dtheta;
        std::vector<double> tau;
        q.reserve(dof);
        dtheta.reserve(dof);
        tau.reserve(dof);

        for (const auto& [group, group_dof] : active_groups) {
            const auto& group_states = states_by_group.at(group);
            if (group_states.q.size() < group_dof || group_states.dtheta.size() < group_dof
                || group_states.tau.size() < group_dof) {
                RCLCPP_ERROR(getLogger(),
                    "Group state vector size mismatch for group %d (q=%ld dtheta=%ld tau=%ld "
                    "expected=%ld)",
                    static_cast<int>(group), group_states.q.size(), group_states.dtheta.size(),
                    group_states.tau.size(), group_dof);
                return hardware_interface::return_type::ERROR;
            }
            q.insert(q.end(), group_states.q.begin(), group_states.q.begin() + group_dof);
            dtheta.insert(
                dtheta.end(), group_states.dtheta.begin(), group_states.dtheta.begin() + group_dof);
            tau.insert(tau.end(), group_states.tau.begin(), group_states.tau.begin() + group_dof);
        }

        if (q.size() != dof || dtheta.size() != dof || tau.size() != dof) {
            RCLCPP_ERROR(getLogger(),
                "Resolved joint state size mismatch (q=%ld dtheta=%ld tau=%ld expected=%ld)",
                q.size(), dtheta.size(), tau.size(), dof);
            return hardware_interface::return_type::ERROR;
        }

        // Refresh every exported per-group robot-states handle the robot reports, so the
        // robot-states broadcaster(s) publish fresh data regardless of which group name they read:
        // a single-arm broadcaster reads the ARMS-named (robot_sn) handle while commands use ARM_1,
        // and dual-arm broadcasters read the ARM_1/ARM_2 (left_/right_) handles.
        for (auto& [group, group_states] : hw_flexiv_robot_states_by_group_) {
            const auto it = states_by_group.find(group);
            if (it != states_by_group.end()) {
                group_states = it->second;
            }
        }

        // Read joint states
        // Map RDK states (RDK order) to Hardware Interface states (ROS order)
        for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            if (ros_idx < info_.joints.size()) {
                hw_states_joint_positions_[ros_idx] = q[rdk_idx];
                hw_states_joint_velocities_[ros_idx] = dtheta[rdk_idx];
                hw_states_joint_efforts_[ros_idx] = tau[rdk_idx];
            }
        }

        // Read GPIO input states
        auto gpio_in = robot_->digital_inputs();
        for (size_t i = 0; i < hw_states_gpio_in_.size(); i++) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_in[i]);
        }
    }

    TrackPositionChangeAcrossInterruption();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Issue no RDK call unless the robot is ready. While recovery runs it changes the control mode
    // and the fault state, and this early return is what guarantees the real-time loop is
    // quiescent for the duration without needing a lock on the hot path.
    if (!robot_ || driver_status_->driver_state.load() != DriverState::READY) {
        return hardware_interface::return_type::OK;
    }

    const size_t dof = rdk_to_ros_map_.size();

    // Reuse preallocated target buffers to keep the control loop allocation-free.
    auto& target_pos = target_pos_buffer_;
    auto& target_vel = target_vel_buffer_;
    auto& target_torque = target_torque_buffer_;

    bool is_pos_nan = false;
    bool is_vel_nan = false;
    bool is_eff_nan = false;
    for (std::size_t i = 0; i < dof; i++) {
        if (hw_commands_joint_positions_[i] != hw_commands_joint_positions_[i]) {
            is_pos_nan = true;
        }
        if (hw_commands_joint_velocities_[i] != hw_commands_joint_velocities_[i]) {
            is_vel_nan = true;
        }
        if (hw_commands_joint_efforts_[i] != hw_commands_joint_efforts_[i]) {
            is_eff_nan = true;
        }
    }

    // Withhold motion until a controller restart has re-synchronized the command buffers.
    const bool stream_motion = driver_status_->commands_synchronized.load();

    if (stream_motion && robot_->mode() == rdk_control_mode_
        && ((position_controller_running_ && !is_pos_nan)
            || (velocity_controller_running_ && !is_vel_nan))) {
        auto states_by_group = robot_->states();
        auto active_groups = determine_active_groups(states_by_group, dof, getLogger());
        if (active_groups.empty()) {
            return hardware_interface::return_type::ERROR;
        }

        if (position_controller_running_ && !is_pos_nan) {
            std::fill(target_vel.begin(), target_vel.end(), 0.0);

            // Map ROS commands to RDK targets.
            for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
                size_t ros_idx = rdk_to_ros_map_[rdk_idx];
                target_pos[rdk_idx] = hw_commands_joint_positions_[ros_idx];
            }
        } else {
            // Map ROS commands/states to RDK targets.
            for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
                size_t ros_idx = rdk_to_ros_map_[rdk_idx];
                target_pos[rdk_idx] = hw_states_joint_positions_[ros_idx];
                target_vel[rdk_idx] = hw_commands_joint_velocities_[ros_idx];
            }
        }

        // Stream real-time joint position commands.
        bool rebuild_rt_joint_position_cmds
            = rt_joint_position_cmds_.size() != active_groups.size();
        if (!rebuild_rt_joint_position_cmds) {
            auto cmd_it = rt_joint_position_cmds_.begin();
            for (const auto& [group, group_dof] : active_groups) {
                if (cmd_it == rt_joint_position_cmds_.end() || cmd_it->first != group
                    || cmd_it->second.q_d.size() != group_dof
                    || cmd_it->second.dq_d.size() != group_dof
                    || cmd_it->second.ddq_d.size() != group_dof) {
                    rebuild_rt_joint_position_cmds = true;
                    break;
                }
                ++cmd_it;
            }
        }

        if (rebuild_rt_joint_position_cmds) {
            rt_joint_position_cmds_.clear();
            for (const auto& [group, group_dof] : active_groups) {
                auto& cmd = rt_joint_position_cmds_[group];
                cmd.q_d.resize(group_dof);
                cmd.dq_d.resize(group_dof);
                cmd.ddq_d.assign(group_dof, 0.0);
            }
        }

        if (active_groups.size() == 1
            && active_groups.front().first == flexiv::rdk::JointGroup::ARMS) {
            auto& cmd = rt_joint_position_cmds_.at(flexiv::rdk::JointGroup::ARMS);
            std::copy(target_pos.begin(), target_pos.end(), cmd.q_d.begin());
            std::copy(target_vel.begin(), target_vel.end(), cmd.dq_d.begin());
        } else {
            size_t offset = 0;
            for (const auto& [group, group_dof] : active_groups) {
                auto& cmd = rt_joint_position_cmds_.at(group);
                const auto begin = static_cast<std::vector<double>::difference_type>(offset);
                std::copy_n(target_pos.begin() + begin, group_dof, cmd.q_d.begin());
                std::copy_n(target_vel.begin() + begin, group_dof, cmd.dq_d.begin());
                offset += group_dof;
            }
        }

        robot_->StreamJointPosition(rt_joint_position_cmds_);
    } else if (stream_motion && torque_controller_running_
               && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE && !is_eff_nan) {
        auto states_by_group = robot_->states();
        auto active_groups = determine_active_groups(states_by_group, dof, getLogger());
        if (active_groups.empty()) {
            return hardware_interface::return_type::ERROR;
        }

        // Map ROS commands to RDK targets.
        for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_torque[rdk_idx] = hw_commands_joint_efforts_[ros_idx];
        }

        bool rebuild_rt_joint_torque_cmds = rt_joint_torque_cmds_.size() != active_groups.size();
        if (!rebuild_rt_joint_torque_cmds) {
            auto cmd_it = rt_joint_torque_cmds_.begin();
            for (const auto& [group, group_dof] : active_groups) {
                if (cmd_it == rt_joint_torque_cmds_.end() || cmd_it->first != group
                    || cmd_it->second.tau_d.size() != group_dof) {
                    rebuild_rt_joint_torque_cmds = true;
                    break;
                }
                ++cmd_it;
            }
        }

        if (rebuild_rt_joint_torque_cmds) {
            rt_joint_torque_cmds_.clear();
            for (const auto& [group, group_dof] : active_groups) {
                auto& cmd = rt_joint_torque_cmds_[group];
                cmd.tau_d.resize(group_dof);
                cmd.enable_gravity_comp = true;
                cmd.enable_soft_limits = true;
            }
        }

        if (active_groups.size() == 1
            && active_groups.front().first == flexiv::rdk::JointGroup::ARMS) {
            auto& cmd = rt_joint_torque_cmds_.at(flexiv::rdk::JointGroup::ARMS);
            std::copy(target_torque.begin(), target_torque.end(), cmd.tau_d.begin());
        } else {
            size_t offset = 0;
            for (const auto& [group, group_dof] : active_groups) {
                auto& cmd = rt_joint_torque_cmds_.at(group);
                const auto begin = static_cast<std::vector<double>::difference_type>(offset);
                std::copy_n(target_torque.begin() + begin, group_dof, cmd.tau_d.begin());
                offset += group_dof;
            }
        }

        robot_->StreamJointTorque(rt_joint_torque_cmds_);
    }

    // Write digital output
    std::map<unsigned int, bool> digital_outputs;
    for (size_t i = 0; i < hw_commands_gpio_out_.size(); i++) {
        if (hw_commands_gpio_out_[i] != hw_commands_gpio_out_[i]) {
            continue;
        }
        digital_outputs[i] = static_cast<bool>(hw_commands_gpio_out_[i]);
    }
    const bool digital_outputs_changed = digital_outputs != current_digital_outputs_;
    current_digital_outputs_ = digital_outputs;

    // Set digital outputs
    if (digital_outputs_changed && !digital_outputs.empty()) {
        robot_->SetDigitalOutputs(digital_outputs);
    }

    return hardware_interface::return_type::OK;
}

void FlexivHardwareInterface::SynchronizeCommandsWithState()
{
    // Called from perform_command_mode_switch(), which is the controller restart the driver
    // requires after a fault. Once the buffers hold the measured position, motion may stream again.
    driver_status_->commands_synchronized.store(true);
    // Position commands start from where the robot actually is, so the first write() after a mode
    // switch commands a hold instead of whatever setpoint was left over from before.
    hw_commands_joint_positions_ = hw_states_joint_positions_;
    std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(), 0.0);

    // Effort commands are deliberately left as NaN rather than zeroed. write() skips streaming
    // while they are NaN, whereas a zero torque command is streamed with gravity compensation
    // enabled and would leave the arm floating freely instead of holding.
    std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
        std::numeric_limits<double>::quiet_NaN());
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
    // All joints must be given new command mode at the same time
    if (start_modes_.size() != 0 && start_modes_.size() != info_.joints.size()) {
        return hardware_interface::return_type::ERROR;
    }
    // All joints must have the same command mode
    if (start_modes_.size() != 0
        && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
        return hardware_interface::return_type::ERROR;
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
    // stop all interfaces at the same time
    if (stop_modes_.size() != 0
        && (stop_modes_.size() != info_.joints.size()
            || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
        return hardware_interface::return_type::ERROR;
    }

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    if (stop_modes_.size() != 0
        && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION)
               != stop_modes_.end()) {
        position_controller_running_ = false;
        StopIfOperational();
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        StopIfOperational();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        StopIfOperational();
    }

    if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        SynchronizeCommandsWithState();

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        // The robot resets its joint impedance properties on mode entry, so whatever was set has to
        // be re-applied before any motion is streamed.
        if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the joint impedance properties. The robot would run at nominal "
                "stiffness instead of the requested one, so the controller start is refused.");
            driver_status_->commands_synchronized.store(false);
            StopIfOperational();
            return hardware_interface::return_type::ERROR;
        }

        position_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        SynchronizeCommandsWithState();

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        // The robot resets its joint impedance properties on mode entry, so whatever was set has to
        // be re-applied before any motion is streamed.
        if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the joint impedance properties. The robot would run at nominal "
                "stiffness instead of the requested one, so the controller start is refused.");
            driver_status_->commands_synchronized.store(false);
            StopIfOperational();
            return hardware_interface::return_type::ERROR;
        }

        velocity_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_EFFORT)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        SynchronizeCommandsWithState();

        // Set to joint torque mode. This is also the step that brings the robot back from IDLE to
        // RT_JOINT_TORQUE after a fault: recovery leaves the robot operational in IDLE, and
        // restarting the effort controller lands here with a freshly synchronized command buffer.
        robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

        // The joint impedance properties do not govern RT_JOINT_TORQUE, so what the driver holds is
        // no longer in effect while the effort controller runs.
        if (joint_impedance_config_node_) {
            joint_impedance_config_node_->MarkNotInEffect();
        }

        torque_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)

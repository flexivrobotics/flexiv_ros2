/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"

namespace {

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

using GroupDofList = std::vector<std::pair<flexiv::rdk::JointGroup, size_t>>;

/**
 * Resolve active joint groups from Robot::states() for this interface.
 *
 * Supported layouts:
 * 1) Single-arm: exactly one group ARMS with DoF equal to expected_dof.
 * 2) Dual-arm: exactly two groups ARM_1 and ARM_2 whose DoFs sum to expected_dof.
 *
 * @param states_by_group Joint-group keyed robot states from RDK.
 * @param expected_dof Total DoF expected by this hardware interface mapping.
 * @return Ordered list of active groups and each group's DoF.
 *         Returns empty when the layout is unsupported or DoF does not match.
 */
GroupDofList determine_active_groups(
    const std::map<flexiv::rdk::JointGroup, flexiv::rdk::RobotStates>& states_by_group,
    size_t expected_dof)
{
    GroupDofList active_groups;

    // Single-arm robot: one and only one group ARMS.
    auto arms_it = states_by_group.find(flexiv::rdk::JointGroup::ARMS);
    if (states_by_group.size() == 1 && arms_it != states_by_group.end()
        && arms_it->second.q.size() == expected_dof) {
        active_groups.emplace_back(flexiv::rdk::JointGroup::ARMS, expected_dof);
        return active_groups;
    }

    // Dual-arm robot: one and only one group for each arm.
    auto arm1_it = states_by_group.find(flexiv::rdk::JointGroup::ARM_1);
    auto arm2_it = states_by_group.find(flexiv::rdk::JointGroup::ARM_2);
    if (states_by_group.size() == 2 && arm1_it != states_by_group.end()
        && arm2_it != states_by_group.end()) {
        const size_t arm1_dof = arm1_it->second.q.size();
        const size_t arm2_dof = arm2_it->second.q.size();
        if (arm1_dof > 0 && arm2_dof > 0 && arm1_dof + arm2_dof == expected_dof) {
            active_groups.emplace_back(flexiv::rdk::JointGroup::ARM_1, arm1_dof);
            active_groups.emplace_back(flexiv::rdk::JointGroup::ARM_2, arm2_dof);
            return active_groups;
        }
    }

    // Any other group combination is unsupported in this interface.
    active_groups.clear();
    return active_groups;
}

}

namespace flexiv_hardware {

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
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
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

    // Get prefix for joint mapping
    std::string prefix;
    try {
        prefix = info_.hardware_parameters.at("prefix");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'prefix' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Build RDK to ROS joint mapping
    std::vector<size_t> arm_indices;
    std::vector<size_t> ext_indices;

    // Find arm joints in standard numeric order: <prefix>joint1 ... <prefix>jointN.
    std::vector<std::pair<int, size_t>> indexed_arm_joints;
    const std::string arm_joint_prefix = prefix + "joint";
    indexed_arm_joints.reserve(info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); ++i) {
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

    if (indexed_arm_joints.size() < 7) {
        RCLCPP_FATAL(getLogger(), "Expected at least 7 arm joints named '%sjointN', found %ld",
            prefix.c_str(), indexed_arm_joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::sort(indexed_arm_joints.begin(), indexed_arm_joints.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    if (indexed_arm_joints.front().first != 1) {
        RCLCPP_FATAL(getLogger(), "Arm joint numbering must start at 1 ('%sjoint1')",
            prefix.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 1; i < indexed_arm_joints.size(); ++i) {
        if (indexed_arm_joints[i].first != indexed_arm_joints[i - 1].first + 1) {
            RCLCPP_FATAL(getLogger(),
                "Arm joints must be contiguous '%sjoint1' ... '%sjointN' without gaps/duplicates",
                prefix.c_str(), prefix.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    for (const auto& [joint_number, joint_pos] : indexed_arm_joints) {
        (void)joint_number;
        arm_indices.push_back(joint_pos);
    }

    if (arm_indices.size() % 7 != 0) {
        RCLCPP_FATAL(getLogger(),
            "Arm joint count (%ld) is invalid: each joint group must contain 7 arm joints",
            arm_indices.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Find external axis joints (any joint that is not an arm joint)
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        bool is_arm = false;
        for (size_t arm_idx : arm_indices) {
            if (i == arm_idx) {
                is_arm = true;
                break;
            }
        }
        if (!is_arm) {
            ext_indices.push_back(i);
        }
    }

    // Construct map: external joints first, then arm joints (RDK order)
    rdk_to_ros_map_.clear();
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), ext_indices.begin(), ext_indices.end());
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), arm_indices.begin(), arm_indices.end());

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

    std::string robot_sn;
    try {
        robot_sn = info_.hardware_parameters["robot_sn"];
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_sn' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        auto rdk_control_mode_str = info_.hardware_parameters.at("rdk_control_mode");
        if (rdk_control_mode_str == "joint_position") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_POSITION;
        } else if (rdk_control_mode_str == "joint_impedance") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE;
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

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        robot_sn, "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_)));

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
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

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "gpio", "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
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

        // Check the DoF of the robot
        if (robot_->info().DoF != info_.joints.size()) {
            RCLCPP_FATAL(getLogger(), "Robot has %ld DoF. Expected %ld (from URDF).",
                robot_->info().DoF, info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Enable the robot
        RCLCPP_INFO(getLogger(), "Enabling robot ...");
        robot_->Enable();

        // Wait for the robot to become operational
        while (!robot_->operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");

        // Unlock external axes if any
        if (robot_->info().DoF_e > 0) {
            robot_->LockExternalAxes(false);
        }
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

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (robot_->operational()) {
        const size_t dof = rdk_to_ros_map_.size();
        auto states_by_group = robot_->states();
        if (states_by_group.empty()) {
            return hardware_interface::return_type::OK;
        }

        auto active_groups = determine_active_groups(states_by_group, dof);
        if (active_groups.empty()) {
            RCLCPP_ERROR(getLogger(),
                "Cannot resolve joint groups from robot states (groups=%ld, expected_dof=%ld)",
                states_by_group.size(), dof);
            return hardware_interface::return_type::ERROR;
        }

        std::vector<double> q;
        std::vector<double> dtheta;
        std::vector<double> tau;
        q.reserve(dof);
        dtheta.reserve(dof);
        tau.reserve(dof);

        const auto& first_group_states = states_by_group.at(active_groups.front().first);
        hw_flexiv_robot_states_ = first_group_states;

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

        hw_flexiv_robot_states_.q = q;
        hw_flexiv_robot_states_.dtheta = dtheta;
        hw_flexiv_robot_states_.tau = tau;

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

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    const size_t dof = rdk_to_ros_map_.size();

    // Initialize target vectors to hold position
    std::vector<double> target_pos(dof);
    std::vector<double> target_vel(dof);

    std::vector<double> max_vel(dof, kMaxJointVelocity);
    std::vector<double> max_acc(dof, kMaxJointAcceleration);

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

    if (position_controller_running_ && robot_->mode() == rdk_control_mode_ && !is_pos_nan) {
        auto states_by_group = robot_->states();
        auto active_groups = determine_active_groups(states_by_group, dof);
        if (active_groups.empty()) {
            RCLCPP_ERROR(getLogger(),
                "Cannot resolve joint groups for position command (groups=%ld, expected_dof=%ld)",
                states_by_group.size(), dof);
            return hardware_interface::return_type::ERROR;
        }

        // Map ROS commands to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_pos[rdk_idx] = hw_commands_joint_positions_[ros_idx];
        }

        std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtJointPositionCmd> cmds;
        if (active_groups.size() == 1 && active_groups.front().first == flexiv::rdk::JointGroup::ARMS) {
            cmds[flexiv::rdk::JointGroup::ARMS]
                = flexiv::rdk::NrtJointPositionCmd(target_pos, target_vel, max_vel, max_acc);
        } else {
            size_t offset = 0;
            for (const auto& [group, group_dof] : active_groups) {
                const auto begin = static_cast<std::ptrdiff_t>(offset);
                const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
                cmds[group] = flexiv::rdk::NrtJointPositionCmd(
                    std::vector<double>(target_pos.begin() + begin, target_pos.begin() + end),
                    std::vector<double>(target_vel.begin() + begin, target_vel.begin() + end),
                    std::vector<double>(max_vel.begin() + begin, max_vel.begin() + end),
                    std::vector<double>(max_acc.begin() + begin, max_acc.begin() + end));
                offset += group_dof;
            }
        }

        robot_->SendJointPosition(cmds);
    } else if (velocity_controller_running_ && robot_->mode() == rdk_control_mode_ && !is_vel_nan) {
        auto states_by_group = robot_->states();
        auto active_groups = determine_active_groups(states_by_group, dof);
        if (active_groups.empty()) {
            RCLCPP_ERROR(getLogger(),
                "Cannot resolve joint groups for velocity command (groups=%ld, expected_dof=%ld)",
                states_by_group.size(), dof);
            return hardware_interface::return_type::ERROR;
        }

        // Map ROS commands/states to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_pos[rdk_idx] = hw_states_joint_positions_[ros_idx];
            target_vel[rdk_idx] = hw_commands_joint_velocities_[ros_idx];
        }

        std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtJointPositionCmd> cmds;
        if (active_groups.size() == 1 && active_groups.front().first == flexiv::rdk::JointGroup::ARMS) {
            cmds[flexiv::rdk::JointGroup::ARMS]
                = flexiv::rdk::NrtJointPositionCmd(target_pos, target_vel, max_vel, max_acc);
        } else {
            size_t offset = 0;
            for (const auto& [group, group_dof] : active_groups) {
                const auto begin = static_cast<std::ptrdiff_t>(offset);
                const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
                cmds[group] = flexiv::rdk::NrtJointPositionCmd(
                    std::vector<double>(target_pos.begin() + begin, target_pos.begin() + end),
                    std::vector<double>(target_vel.begin() + begin, target_vel.begin() + end),
                    std::vector<double>(max_vel.begin() + begin, max_vel.begin() + end),
                    std::vector<double>(max_acc.begin() + begin, max_acc.begin() + end));
                offset += group_dof;
            }
        }

        robot_->SendJointPosition(cmds);
    } else if (torque_controller_running_ && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE
               && !is_eff_nan) {
        auto states_by_group = robot_->states();
        auto active_groups = determine_active_groups(states_by_group, dof);
        if (active_groups.empty()) {
            RCLCPP_ERROR(getLogger(),
                "Cannot resolve joint groups for torque command (groups=%ld, expected_dof=%ld)",
                states_by_group.size(), dof);
            return hardware_interface::return_type::ERROR;
        }

        std::vector<double> target_torque(dof);
        // Map ROS commands to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < dof; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_torque[rdk_idx] = hw_commands_joint_efforts_[ros_idx];
        }

        std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointTorqueCmd> cmds;
        if (active_groups.size() == 1 && active_groups.front().first == flexiv::rdk::JointGroup::ARMS) {
            cmds[flexiv::rdk::JointGroup::ARMS]
                = flexiv::rdk::RtJointTorqueCmd(target_torque, true, true);
        } else {
            size_t offset = 0;
            for (const auto& [group, group_dof] : active_groups) {
                const auto begin = static_cast<std::ptrdiff_t>(offset);
                const auto end = static_cast<std::ptrdiff_t>(offset + group_dof);
                cmds[group] = flexiv::rdk::RtJointTorqueCmd(
                    std::vector<double>(target_torque.begin() + begin, target_torque.begin() + end),
                    true, true);
                offset += group_dof;
            }
        }

        robot_->StreamJointTorque(cmds);
    }

    // Write digital output
    std::map<unsigned int, bool> digital_outputs;
    for (size_t i = 0; i < hw_commands_gpio_out_.size(); i++) {
        if (hw_commands_gpio_out_[i] != hw_commands_gpio_out_[i]) {
            continue;
        }
        digital_outputs[i] = static_cast<bool>(hw_commands_gpio_out_[i]);
    }
    // Check if there are changes in the digital output values
    bool digital_outputs_changed = false;
    for (const auto& [index, value] : digital_outputs) {
        if (current_digital_outputs_[index] != value) {
            current_digital_outputs_[index] = value;
            digital_outputs_changed = true;
        }
    }
    current_digital_outputs_.clear();
    for (const auto& [index, value] : digital_outputs) {
        current_digital_outputs_[index] = value;
    }

    // Set digital outputs
    if (digital_outputs_changed && !digital_outputs.empty()) {
        robot_->SetDigitalOutputs(digital_outputs);
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
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        robot_->Stop();
    }

    if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_positions_.begin(), hw_commands_joint_positions_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        position_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        velocity_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_EFFORT)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint torque mode
        robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

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

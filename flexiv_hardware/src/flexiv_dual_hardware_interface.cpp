/**
 * @file flexiv_dual_hardware_interface.cpp
 * @brief Hardware interface to a pair of Flexiv robots for ROS 2 control.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <algorithm>
#include <cmath>
#include <map>
#include <variant>
#include <vector>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/drdk/robot_pair.hpp"
#include "flexiv_hardware/flexiv_dual_hardware_interface.hpp"
#include "flexiv_hardware/fault_recovery.hpp"

namespace {
constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;
// Furthest a velocity target may lead the measured position [rad], so that a blocked or
// lagging joint cannot build up a large step
constexpr double kMaxVelocityTargetLead = 0.1;

// Bounded wait for both robots to become operational during activation.
constexpr std::chrono::seconds kActivationOperationalTimeout {30};
constexpr std::chrono::milliseconds kOperationalPollPeriod {200};

// Bounded wait for the ZeroFTSensor primitive, which takes a few seconds.
constexpr std::chrono::seconds kZeroFTSensorTimeout {10};
constexpr std::chrono::milliseconds kPrimitivePollPeriod {100};

template <size_t N>
bool AllFinite(const std::array<std::array<double, N>, 2>& values)
{
    return std::all_of(values.begin(), values.end(), [](const auto& robot) {
        return std::all_of(robot.begin(), robot.end(), [](double v) { return std::isfinite(v); });
    });
}
}

namespace flexiv_hardware {

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info)
        != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_targets_.resize(info_.joints.size(), 0.0);
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // kIOPorts per robot, left first
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts * 2, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(
        flexiv::rdk::kIOPorts * 2, std::numeric_limits<double>::quiet_NaN());
    for (size_t robot = 0; robot < 2; robot++) {
        hw_commands_cartesian_pose_[robot].fill(std::numeric_limits<double>::quiet_NaN());
        hw_commands_cartesian_wrench_[robot].fill(std::numeric_limits<double>::quiet_NaN());
        hw_commands_cartesian_velocity_[robot].fill(std::numeric_limits<double>::quiet_NaN());
        hw_states_cartesian_pose_[robot].fill(std::numeric_limits<double>::quiet_NaN());
    }
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;

    if (info_.joints.size() < 14) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected at least 14.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
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
            RCLCPP_FATAL(getLogger(), "Joint '%s' have %s state interfaces found. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' have %s state interfaces found. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' have %s state interfaces found. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    std::string robot_sn_left;
    std::string robot_sn_right;
    try {
        robot_sn_left = info_.hardware_parameters.at("robot_sn_left");
        robot_sn_right = info_.hardware_parameters.at("robot_sn_right");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_sn_left' or 'robot_sn_right' not set");
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

    // Read translation parameters
    double left_x = 0.0, left_y = 0.0, left_z = 0.0;
    double right_x = 0.0, right_y = 0.0, right_z = 0.0;
    try {
        if (info_.hardware_parameters.count("translation_left_x")) {
            left_x = std::stod(info_.hardware_parameters.at("translation_left_x"));
            left_y = std::stod(info_.hardware_parameters.at("translation_left_y"));
            left_z = std::stod(info_.hardware_parameters.at("translation_left_z"));
        }
        if (info_.hardware_parameters.count("translation_right_x")) {
            right_x = std::stod(info_.hardware_parameters.at("translation_right_x"));
            right_y = std::stod(info_.hardware_parameters.at("translation_right_y"));
            right_z = std::stod(info_.hardware_parameters.at("translation_right_z"));
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN(getLogger(), "Failed to parse translation parameters, using default (0,0,0)");
    }

    std::pair<std::array<double, 3>, std::array<double, 3>> translations;
    translations.first = {left_x, left_y, left_z};
    translations.second = {right_x, right_y, right_z};

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robots %s and %s ...", robot_sn_left.c_str(),
            robot_sn_right.c_str());
        robot_pair_ = std::make_unique<flexiv::drdk::RobotPair>(
            std::make_pair(robot_sn_left, robot_sn_right), translations);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robots");
        RCLCPP_FATAL(getLogger(), "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // The joint map below is built from the connected robots, so unlike the single-robot
    // interface the connection has to stay in on_init. on_configure only brings up the recovery
    // interface on top of it.
    driver_status_ = std::make_shared<DriverStatus>();
    robot_system_control_ = std::make_unique<DualRobotSystemControl>(*robot_pair_);

    if (!CheckJointCount()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Build joint map
    joint_map_.resize(info_.joints.size());
    std::vector<size_t> unmapped_indices;
    const std::string prefix_left = info_.hardware_parameters.at("prefix_left");
    const std::string prefix_right = info_.hardware_parameters.at("prefix_right");

    cartesian_command_interface_names_ = CartesianCommandInterfaceNames(prefix_left);
    for (const auto& name : CartesianCommandInterfaceNames(prefix_right)) {
        cartesian_command_interface_names_.push_back(name);
    }

    // External axes come first in each robot's joint vector. On AICO2 they belong to the left
    // robot; the right robot has none.
    const size_t extra_dof_left = robot_pair_->info().first.DoF_e;
    const size_t extra_dof_right = robot_pair_->info().second.DoF_e;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        std::string name = info_.joints[i].name;
        bool mapped = false;
        // Left robot arm joints (ext_dof_left + 0...6)
        if (name.find(prefix_left + "joint") == 0) {
            std::string num_str = name.substr((prefix_left + "joint").length());
            try {
                int joint_num = std::stoi(num_str);
                if (joint_num >= 1 && joint_num <= 7) {
                    joint_map_[i] = {0, (int)extra_dof_left + joint_num - 1};
                    mapped = true;
                }
            } catch (...) {
            }
        }

        // Right robot arm joints (ext_dof_right + 0...6)
        if (!mapped && name.find(prefix_right + "joint") == 0) {
            std::string num_str = name.substr((prefix_right + "joint").length());
            try {
                int joint_num = std::stoi(num_str);
                if (joint_num >= 1 && joint_num <= 7) {
                    joint_map_[i] = {1, (int)extra_dof_right + joint_num - 1};
                    mapped = true;
                }
            } catch (...) {
            }
        }

        if (!mapped) {
            unmapped_indices.push_back(i);
        }
    }

    if (unmapped_indices.size() != extra_dof_left + extra_dof_right) {
        RCLCPP_FATAL(getLogger(), "Mismatch in extra joints count. Unmapped: %ld, Expected: %ld",
            unmapped_indices.size(), extra_dof_left + extra_dof_right);
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t unmapped_idx = 0;
    // Assign external joints to Left Robot (indices 0 to extra_dof_left-1)
    for (size_t k = 0; k < extra_dof_left; k++) {
        joint_map_[unmapped_indices[unmapped_idx++]] = {0, (int)k};
    }
    // Assign external joints to Right Robot (indices 0 to extra_dof_right-1)
    for (size_t k = 0; k < extra_dof_right; k++) {
        joint_map_[unmapped_indices[unmapped_idx++]] = {1, (int)k};
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robots");
    return hardware_interface::CallbackReturn::SUCCESS;
}

rclcpp::Logger FlexivDualHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivDualHardwareInterface");
}

bool FlexivDualHardwareInterface::CheckJointCount() const
{
    const auto info = robot_pair_->info();
    if (info.first.DoF + info.second.DoF == info_.joints.size()) {
        return true;
    }
    RCLCPP_FATAL(getLogger(),
        "Connected robots total DoF (%ld + %ld = %ld) do not match expected DoF (%ld)",
        info.first.DoF, info.second.DoF, info.first.DoF + info.second.DoF, info_.joints.size());
    return false;
}

std::pair<std::vector<double>, std::vector<double>> FlexivDualHardwareInterface::ToDRDKOrder(
    const std::vector<double>& ros_values,
    const std::pair<flexiv::rdk::RobotInfo, flexiv::rdk::RobotInfo>& info) const
{
    std::pair<std::vector<double>, std::vector<double>> values {
        std::vector<double>(info.first.DoF), std::vector<double>(info.second.DoF)};

    for (size_t i = 0; i < joint_map_.size(); i++) {
        auto& robot_values = joint_map_[i].robot_index == 0 ? values.first : values.second;
        robot_values[joint_map_[i].dof_index] = ros_values[i];
    }
    return values;
}

std::vector<hardware_interface::StateInterface>
FlexivDualHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_joint_efforts_[i]));
    }

    // GPIOs
    const std::string prefix_left = info_.hardware_parameters.at("prefix_left");
    const std::string prefix_right = info_.hardware_parameters.at("prefix_right");

    // Remove trailing underscore from prefix to get the robot name used in controllers
    std::string robot_name_left = prefix_left;
    if (!robot_name_left.empty() && robot_name_left.back() == '_') {
        robot_name_left.pop_back();
    }
    std::string robot_name_right = prefix_right;
    if (!robot_name_right.empty() && robot_name_right.back() == '_') {
        robot_name_right.pop_back();
    }

    // Export robot states for both robots
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_name_left,
        "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_left_)));

    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_name_right,
        "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_right_)));
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix_left + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_right + "gpio",
            "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i + flexiv::rdk::kIOPorts]));
    }

    const std::array<std::string, 2> prefixes = {prefix_left, prefix_right};
    for (size_t robot = 0; robot < 2; robot++) {
        for (size_t i = 0; i < flexiv::rdk::kPoseSize; i++) {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(prefixes[robot] + "tcp",
                    kCartesianPoseInterfaces[i], &hw_states_cartesian_pose_[robot][i]));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivDualHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    const std::string prefix_left = info_.hardware_parameters.at("prefix_left");
    const std::string prefix_right = info_.hardware_parameters.at("prefix_right");
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_left + "gpio",
            "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_right + "gpio",
            "digital_output_" + std::to_string(i),
            &hw_commands_gpio_out_[i + flexiv::rdk::kIOPorts]));
    }

    const std::array<std::string, 2> prefixes = {prefix_left, prefix_right};
    for (size_t robot = 0; robot < 2; robot++) {
        const std::string name = prefixes[robot] + "tcp";
        for (size_t i = 0; i < flexiv::rdk::kPoseSize; i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                name, kCartesianPoseInterfaces[i], &hw_commands_cartesian_pose_[robot][i]));
        }
        for (size_t i = 0; i < flexiv::rdk::kCartDoF; i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                name, kCartesianWrenchInterfaces[i], &hw_commands_cartesian_wrench_[robot][i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                name, kCartesianVelocityInterfaces[i], &hw_commands_cartesian_velocity_[robot][i]));
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    driver_status_->driver_state.store(DriverState::FAULT);

    // Namespaced by the left robot so that the pair has a single, predictable recovery interface.
    const std::string robot_sn_left = info_.hardware_parameters.at("robot_sn_left");
    recovery_node_
        = std::make_shared<RecoveryNode>(robot_sn_left, *robot_system_control_, driver_status_);

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(recovery_node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // The impedance setters only apply to the joint impedance control modes, but the interface is
    // advertised either way so that a request made against a joint_position driver is answered with
    // an explanation instead of a missing service.
    std::vector<std::string> joint_names;
    joint_names.reserve(info_.joints.size());
    for (const auto& joint : info_.joints) {
        joint_names.push_back(joint.name);
    }

    // DRDK takes one vector per robot, so the ROS-ordered vectors are split by this map. It mirrors
    // joint_map_, which is what read() and write() already use.
    std::vector<PairJointIndex> pair_joint_map;
    pair_joint_map.reserve(joint_map_.size());
    for (const auto& entry : joint_map_) {
        pair_joint_map.push_back({entry.robot_index, entry.dof_index});
    }

    // A joint of either robot that no ROS joint maps to keeps its nominal value rather than 0,
    // which would leave it free-floating.
    const auto info = robot_pair_->info();

    JointImpedanceBounds bounds;
    bounds.k_q_nom = ConvertDRDKToROSOrder(info.first.K_q_nom, info.second.K_q_nom, pair_joint_map);
    bounds.tau_max = ConvertDRDKToROSOrder(info.first.tau_max, info.second.tau_max, pair_joint_map);

    const std::vector<double> nominal_z_q_left(info.first.DoF, kNominalDampingRatio);
    const std::vector<double> nominal_z_q_right(info.second.DoF, kNominalDampingRatio);
    const std::vector<double> nominal_inertia_left(info.first.DoF, kNominalInertiaScale);
    const std::vector<double> nominal_inertia_right(info.second.DoF, kNominalInertiaScale);

    // The node works in ROS joint order and knows nothing about DRDK; these three closures are
    // where the order is split and the pair is actually called. Both halves are always sent in
    // full: an empty half means "nominal" to DRDK, which would silently reset the other arm.
    JointImpedanceSetters setters;
    setters.set_joint_impedance
        = [this, pair_joint_map, k_q_nom_left = info.first.K_q_nom,
              k_q_nom_right = info.second.K_q_nom, nominal_z_q_left,
              nominal_z_q_right](const std::vector<double>& k_q, const std::vector<double>& z_q) {
              robot_pair_->SetJointImpedance(
                  ConvertROSToDRDKOrder(k_q, pair_joint_map, k_q_nom_left, k_q_nom_right),
                  ConvertROSToDRDKOrder(z_q, pair_joint_map, nominal_z_q_left, nominal_z_q_right));
          };
    setters.set_max_contact_torque
        = [this, pair_joint_map, tau_max_left = info.first.tau_max,
              tau_max_right = info.second.tau_max](const std::vector<double>& max_torques) {
              robot_pair_->SetMaxContactTorque(
                  ConvertROSToDRDKOrder(max_torques, pair_joint_map, tau_max_left, tau_max_right));
          };
    setters.set_joint_inertia_scale
        = [this, pair_joint_map, nominal_inertia_left, nominal_inertia_right](
              const std::vector<double>& inertia_scales) {
              robot_pair_->SetJointInertiaScale(ConvertROSToDRDKOrder(
                  inertia_scales, pair_joint_map, nominal_inertia_left, nominal_inertia_right));
          };

    joint_impedance_config_node_ = std::make_shared<JointImpedanceConfigNode>(robot_sn_left,
        joint_names, std::move(bounds), rdk_control_mode_ == flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE,
        driver_status_, std::move(setters));
    executor_->add_node(joint_impedance_config_node_->get_node_base_interface());

    CartesianMotionForceBounds cartesian_bounds;
    cartesian_bounds.k_x_nom = {info.first.K_x_nom, info.second.K_x_nom};
    cartesian_bounds.q_min
        = ConvertDRDKToROSOrder(info.first.q_min, info.second.q_min, pair_joint_map);
    cartesian_bounds.q_max
        = ConvertDRDKToROSOrder(info.first.q_max, info.second.q_max, pair_joint_map);

    // Every per-robot vector holds the left robot first, which is the order DRDK pairs take.
    CartesianMotionForceSetters cartesian_setters;
    cartesian_setters.set_cartesian_impedance
        = [this](const std::vector<CartesianArray>& k_x, const std::vector<CartesianArray>& z_x) {
              robot_pair_->SetCartesianImpedance({k_x[0], k_x[1]}, {z_x[0], z_x[1]});
          };
    cartesian_setters.set_max_contact_wrench = [this](const std::vector<CartesianArray>& wrench) {
        robot_pair_->SetMaxContactWrench({wrench[0], wrench[1]});
    };
    // Joints no ROS joint maps to keep their current position
    cartesian_setters.set_null_space_posture = [this, pair_joint_map](
                                                   const std::vector<double>& ref_positions) {
        const auto states = robot_pair_->states();
        robot_pair_->SetNullSpacePosture(
            ConvertROSToDRDKOrder(ref_positions, pair_joint_map, states.first.q, states.second.q));
    };
    cartesian_setters.set_null_space_objectives
        = [this](const std::vector<NullSpaceObjectives>& objectives) {
              robot_pair_->SetNullSpaceObjectives(
                  {objectives[0].linear_manipulability, objectives[1].linear_manipulability},
                  {objectives[0].angular_manipulability, objectives[1].angular_manipulability},
                  {objectives[0].ref_positions_tracking, objectives[1].ref_positions_tracking});
          };
    cartesian_setters.set_force_control_axis
        = [this](const std::vector<CartesianFlags>& enabled_axes,
              const std::vector<LinearArray>& max_linear_vel) {
              robot_pair_->SetForceControlAxis(
                  {enabled_axes[0], enabled_axes[1]}, {max_linear_vel[0], max_linear_vel[1]});
          };
    cartesian_setters.set_force_control_frame
        = [this](const std::vector<flexiv::rdk::CoordType>& root_coord,
              const std::vector<PoseArray>& t_in_root) {
              robot_pair_->SetForceControlFrame(
                  {root_coord[0], root_coord[1]}, {t_in_root[0], t_in_root[1]});
          };
    cartesian_setters.set_passive_force_control = [this](const std::vector<bool>& enabled) {
        robot_pair_->SetPassiveForceControl({enabled[0], enabled[1]});
    };
    cartesian_setters.set_motion_limits = [this](const std::vector<CartesianMotionLimits>& limits) {
        for (size_t robot = 0; robot < 2; robot++) {
            cartesian_max_linear_vel_[robot].store(limits[robot].max_linear_vel);
            cartesian_max_angular_vel_[robot].store(limits[robot].max_angular_vel);
            cartesian_max_linear_acc_[robot].store(limits[robot].max_linear_acc);
            cartesian_max_angular_acc_[robot].store(limits[robot].max_angular_acc);
        }
    };

    cartesian_config_node_
        = std::make_shared<CartesianMotionForceConfigNode>(robot_sn_left, std::move(joint_names),
            std::move(cartesian_bounds), driver_status_, std::move(cartesian_setters));
    executor_->add_node(cartesian_config_node_->get_node_base_interface());

    return hardware_interface::CallbackReturn::SUCCESS;
}

void FlexivDualHardwareInterface::TrackPositionChangeAcrossInterruption()
{
    const bool ready = driver_status_->driver_state.load() == DriverState::READY;

    if (was_ready_ && !ready) {
        // Joint states stop being refreshed once the pair is no longer operational, so this still
        // holds the last positions the robots were known to be at before they stopped.
        positions_before_interruption_ = hw_states_joint_positions_;
    } else if (!was_ready_ && ready && !positions_before_interruption_.empty()) {
        if (driver_status_->RequiresControllerRestart()) {
            const double deviation
                = MaxJointDeviation(positions_before_interruption_, hw_states_joint_positions_);

            RCLCPP_WARN(getLogger(),
                "The robots are ready again, %.3f rad from the last commanded "
                "position. Motion stays withheld until the controllers are restarted.",
                deviation);
        }
        positions_before_interruption_.clear();
    }

    was_ready_ = ready;
}

void FlexivDualHardwareInterface::StopIfOperational()
{
    if (robot_pair_ && robot_pair_->connected() && robot_pair_->operational()) {
        robot_pair_->Stop();
    }
}

bool FlexivDualHardwareInterface::ZeroForceTorqueSensor()
{
    if (driver_status_->driver_state.load() != DriverState::READY) {
        RCLCPP_ERROR(getLogger(), "Cannot zero the force/torque sensors: the robots are not ready");
        return false;
    }

    RCLCPP_WARN(getLogger(),
        "Zeroing the force/torque sensors, make sure nothing is in contact with either robot");
    try {
        robot_pair_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        const std::map<std::string, flexiv::rdk::FlexivDataTypes> no_params;
        robot_pair_->ExecutePrimitive({"ZeroFTSensor", "ZeroFTSensor"}, {no_params, no_params});

        const auto deadline = std::chrono::steady_clock::now() + kZeroFTSensorTimeout;
        while (true) {
            auto states = robot_pair_->primitive_states();
            if (std::get<int>(states.first["terminated"])
                && std::get<int>(states.second["terminated"])) {
                break;
            }
            if (robot_pair_->fault()) {
                throw std::runtime_error("a fault occurred on one of the robots");
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

void FlexivDualHardwareInterface::TeardownRecoveryNode()
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
    // Torn down here too, since its closures capture this and call through robot_pair_.
    if (joint_impedance_config_node_) {
        if (executor_) {
            executor_->remove_node(joint_impedance_config_node_->get_node_base_interface());
        }
        joint_impedance_config_node_.reset();
    }
    if (cartesian_config_node_) {
        if (executor_) {
            executor_->remove_node(cartesian_config_node_->get_node_base_interface());
        }
        cartesian_config_node_.reset();
    }
    executor_.reset();
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    TeardownRecoveryNode();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    TeardownRecoveryNode();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_ERROR(getLogger(), "Hardware component entered the error state, stopping the robots");

    driver_status_->driver_state.store(DriverState::FAULT);
    try {
        StopIfOperational();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not stop the robots: %s", e.what());
    }

    TeardownRecoveryNode();

    // SUCCESS puts the component in UNCONFIGURED, from which it can be configured and activated
    // again. FAILURE or ERROR would finalize it and force a restart of the whole process.
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool FlexivDualHardwareInterface::WaitUntilOperational(std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_log = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        if (robot_pair_->operational()) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= next_log) {
            RCLCPP_INFO(getLogger(), "Waiting for both robots to become operational ...");
            next_log = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        }
        std::this_thread::sleep_for(kOperationalPollPeriod);
    }
    return robot_pair_->operational();
}

void FlexivDualHardwareInterface::AdvanceVelocityTargets(double dt)
{
    for (size_t i = 0; i < velocity_targets_.size(); i++) {
        const double measured = hw_states_joint_positions_[i];
        velocity_targets_[i]
            = std::clamp(velocity_targets_[i] + hw_commands_joint_velocities_[i] * dt,
                measured - kMaxVelocityTargetLead, measured + kMaxVelocityTargetLead);
    }
}

void FlexivDualHardwareInterface::SynchronizeCommandsWithState()
{
    // Called from perform_command_mode_switch(), which is the controller restart the driver
    // requires after a fault. Once the buffers hold the measured position, motion may stream again.
    driver_status_->commands_synchronized.store(true);
    // Position commands start from where the robots actually are, so the first write() after a
    // mode switch commands a hold instead of a stale setpoint.
    hw_commands_joint_positions_ = hw_states_joint_positions_;
    std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(), 0.0);
    velocity_targets_ = hw_states_joint_positions_;

    // Effort commands stay NaN so that write() skips streaming. A zero torque command is streamed
    // with gravity compensation enabled and would leave the arms floating instead of holding.
    std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
        std::numeric_limits<double>::quiet_NaN());

    // The Cartesian commands likewise start as a hold at the measured TCP poses, without force.
    hw_commands_cartesian_pose_ = hw_states_cartesian_pose_;
    for (size_t robot = 0; robot < 2; robot++) {
        hw_commands_cartesian_wrench_[robot].fill(0.0);
        hw_commands_cartesian_velocity_[robot].fill(0.0);
    }
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
        // Clear fault on the connected robots if any
        if (robot_pair_->fault()) {
            RCLCPP_WARN(
                getLogger(), "Fault occurred on one of the connected robots, trying to clear ...");
            // Try to clear the fault for both robots
            auto result = robot_pair_->ClearFault();
            // If fault is not cleared on both robots
            if (!(result.first && result.second)) {
                RCLCPP_ERROR(getLogger(), "Fault cannot be cleared, exiting ...");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(getLogger(), "Fault on the connected robot is cleared");
        }

        if (!CheckJointCount()) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Enable the pair of robots
        RCLCPP_INFO(getLogger(), "Enabling robots ...");
        robot_pair_->Enable();

        // Wait for both robots to become operational, bounded so that a robot that never becomes
        // ready fails the activation instead of hanging the controller manager forever.
        if (!WaitUntilOperational(kActivationOperationalTimeout)) {
            RCLCPP_FATAL(getLogger(),
                "Robots did not become operational within %ld s. Check that the E-stop is "
                "released and that both robots are in Auto (Remote) mode.",
                static_cast<long>(kActivationOperationalTimeout.count()));
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(getLogger(), "Both robots are now operational");

        // Unlock external axes if any
        if (robot_pair_->info().first.DoF_e > 0 || robot_pair_->info().second.DoF_e > 0) {
            robot_pair_->LockExternalAxes(
                {robot_pair_->info().first.DoF_e == 0, robot_pair_->info().second.DoF_e == 0});
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable the robots");
        RCLCPP_FATAL(getLogger(), "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // The robots are enabled but in IDLE: a controller start has to establish the control mode and
    // synchronize the command buffers before any motion may be streamed.
    driver_status_->commands_synchronized.store(false);
    driver_status_->driver_state.store(DriverState::READY);

    RCLCPP_INFO(getLogger(), "System successfully started!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    // Hold off write() before stopping, so the real-time loop cannot stream a command into robots
    // that are being brought to a halt.
    driver_status_->driver_state.store(DriverState::FAULT);

    try {
        StopIfOperational();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not stop the robots: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivDualHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Latch the pair condition for the recovery node. DRDK reports the pair as a whole, so the
    // status topic carries the combined condition rather than per-robot detail.
    driver_status_->Latch(*robot_system_control_);

    // Recovery owns the driver state while it runs, so this is a no-op for its duration.
    driver_status_->TryApplyDerivedDriverState();

    if (!driver_status_->connected.load()) {
        RCLCPP_ERROR(getLogger(), "Lost connection with one or both robots");
        return hardware_interface::return_type::ERROR;
    }

    if (driver_status_->operational.load()) {
        auto robot_states_pair = robot_pair_->states();
        hw_flexiv_robot_states_left_ = robot_states_pair.first;
        hw_flexiv_robot_states_right_ = robot_states_pair.second;
        hw_states_cartesian_pose_
            = {robot_states_pair.first.tcp_pose, robot_states_pair.second.tcp_pose};

        for (size_t i = 0; i < info_.joints.size(); i++) {
            int robot_idx = joint_map_[i].robot_index;
            int dof_idx = joint_map_[i].dof_index;

            if (robot_idx == 0) {
                hw_states_joint_positions_[i] = robot_states_pair.first.q[dof_idx];
                hw_states_joint_velocities_[i] = robot_states_pair.first.dq[dof_idx];
                hw_states_joint_efforts_[i] = robot_states_pair.first.tau[dof_idx];
            } else {
                hw_states_joint_positions_[i] = robot_states_pair.second.q[dof_idx];
                hw_states_joint_velocities_[i] = robot_states_pair.second.dq[dof_idx];
                hw_states_joint_efforts_[i] = robot_states_pair.second.tau[dof_idx];
            }
        }

        // Read GPIO inputs
        auto gpio_inputs = robot_pair_->digital_inputs();
        for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_inputs.first[i]);
            hw_states_gpio_in_[i + flexiv::rdk::kIOPorts]
                = static_cast<double>(gpio_inputs.second[i]);
        }
    }

    TrackPositionChangeAcrossInterruption();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivDualHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    // Issue no DRDK call unless the robots are ready. While recovery runs it changes the control
    // mode and the fault state, and this early return is what guarantees the real-time loop is
    // quiescent for the duration without needing a lock on the hot path.
    if (driver_status_->driver_state.load() != DriverState::READY) {
        return hardware_interface::return_type::OK;
    }

    const auto info = robot_pair_->info();
    const std::pair<std::vector<double>, std::vector<double>> max_vel {
        std::vector<double>(info.first.DoF, kMaxJointVelocity),
        std::vector<double>(info.second.DoF, kMaxJointVelocity)};
    const std::pair<std::vector<double>, std::vector<double>> max_acc {
        std::vector<double>(info.first.DoF, kMaxJointAcceleration),
        std::vector<double>(info.second.DoF, kMaxJointAcceleration)};

    const auto any_nan = [](const std::vector<double>& values) {
        return std::any_of(values.begin(), values.end(), [](double v) { return std::isnan(v); });
    };

    // Withhold motion until a controller restart has re-synchronized the command buffers. Digital
    // outputs further down are unaffected -- they carry no setpoint that can go stale.
    const bool stream_motion = driver_status_->commands_synchronized.load();
    const auto joint_mode = std::pair {rdk_control_mode_, rdk_control_mode_};

    // Velocity control resumes from the measured position after NaN commands
    if (any_nan(hw_commands_joint_velocities_)) {
        velocity_targets_ = hw_states_joint_positions_;
    }

    if (stream_motion && position_controller_running_ && robot_pair_->mode() == joint_mode) {
        // A joint without a position command holds its measured position
        std::vector<double> target_pos(info_.joints.size());
        std::vector<double> target_vel(info_.joints.size());
        for (size_t i = 0; i < info_.joints.size(); i++) {
            target_pos[i] = std::isnan(hw_commands_joint_positions_[i])
                                ? hw_states_joint_positions_[i]
                                : hw_commands_joint_positions_[i];
            target_vel[i] = std::isnan(hw_commands_joint_velocities_[i])
                                ? 0.0
                                : hw_commands_joint_velocities_[i];
        }
        robot_pair_->SendJointPosition(
            ToDRDKOrder(target_pos, info), ToDRDKOrder(target_vel, info), max_vel, max_acc);
    } else if (stream_motion && velocity_controller_running_ && robot_pair_->mode() == joint_mode
               && !any_nan(hw_commands_joint_velocities_)) {
        // DRDK takes the velocity as the one to have on arriving at the target, so the target
        // moves ahead of the robots at the commanded velocity
        AdvanceVelocityTargets(period.seconds());
        robot_pair_->SendJointPosition(ToDRDKOrder(velocity_targets_, info),
            ToDRDKOrder(hw_commands_joint_velocities_, info), max_vel, max_acc);
    } else if (stream_motion && torque_controller_running_
               && robot_pair_->mode()
                      == std::pair {flexiv::rdk::Mode::RT_JOINT_TORQUE,
                          flexiv::rdk::Mode::RT_JOINT_TORQUE}
               && !any_nan(hw_commands_joint_efforts_)) {
        robot_pair_->StreamJointTorque(ToDRDKOrder(hw_commands_joint_efforts_, info));
    } else if (stream_motion && cartesian_controller_running_
               && robot_pair_->mode()
                      == std::pair {flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE,
                          flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE}
               && AllFinite(hw_commands_cartesian_pose_) && AllFinite(hw_commands_cartesian_wrench_)
               && AllFinite(hw_commands_cartesian_velocity_)) {
        robot_pair_->SendCartesianMotionForce(
            {hw_commands_cartesian_pose_[0], hw_commands_cartesian_pose_[1]},
            {hw_commands_cartesian_wrench_[0], hw_commands_cartesian_wrench_[1]},
            {hw_commands_cartesian_velocity_[0], hw_commands_cartesian_velocity_[1]},
            {cartesian_max_linear_vel_[0].load(), cartesian_max_linear_vel_[1].load()},
            {cartesian_max_angular_vel_[0].load(), cartesian_max_angular_vel_[1].load()},
            {cartesian_max_linear_acc_[0].load(), cartesian_max_linear_acc_[1].load()},
            {cartesian_max_angular_acc_[0].load(), cartesian_max_angular_acc_[1].load()});
    }

    // Write digital outputs
    std::map<unsigned int, bool> digital_outputs_left;
    std::map<unsigned int, bool> digital_outputs_right;
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        if (hw_commands_gpio_out_[i] == hw_commands_gpio_out_[i]) {
            digital_outputs_left[i] = static_cast<bool>(hw_commands_gpio_out_[i]);
        }
        if (hw_commands_gpio_out_[i + flexiv::rdk::kIOPorts]
            == hw_commands_gpio_out_[i + flexiv::rdk::kIOPorts]) {
            digital_outputs_right[i]
                = static_cast<bool>(hw_commands_gpio_out_[i + flexiv::rdk::kIOPorts]);
        }
    }
    // Check if there is any change in digital outputs before sending. A port never sent counts as
    // low.
    bool digital_outputs_changed = false;
    for (const auto& [port, value] : digital_outputs_left) {
        if (current_digital_outputs_left_[port] != value) {
            digital_outputs_changed = true;
            current_digital_outputs_left_[port] = value;
        }
    }
    for (const auto& [port, value] : digital_outputs_right) {
        if (current_digital_outputs_right_[port] != value) {
            digital_outputs_changed = true;
            current_digital_outputs_right_[port] = value;
        }
    }

    // Set digital outputs
    if (digital_outputs_changed) {
        robot_pair_->SetDigitalOutputs({digital_outputs_left, digital_outputs_right});
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivDualHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    start_modes_.clear();
    stop_modes_.clear();
    cartesian_start_requested_ = false;
    cartesian_stop_requested_ = false;

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

    // The Cartesian command interfaces of both robots must all be claimed together
    const auto count_cartesian = [this](const std::vector<std::string>& keys) {
        return static_cast<size_t>(std::count_if(keys.begin(), keys.end(), [this](const auto& key) {
            return std::find(cartesian_command_interface_names_.begin(),
                       cartesian_command_interface_names_.end(), key)
                   != cartesian_command_interface_names_.end();
        }));
    };
    const size_t cartesian_start_count = count_cartesian(start_interfaces);
    const size_t cartesian_stop_count = count_cartesian(stop_interfaces);
    if ((cartesian_start_count != 0
            && cartesian_start_count != cartesian_command_interface_names_.size())
        || (cartesian_stop_count != 0
            && cartesian_stop_count != cartesian_command_interface_names_.size())) {
        RCLCPP_ERROR(getLogger(),
            "A controller must claim all %ld Cartesian command interfaces of both robots",
            cartesian_command_interface_names_.size());
        return hardware_interface::return_type::ERROR;
    }
    cartesian_start_requested_ = cartesian_start_count != 0;
    cartesian_stop_requested_ = cartesian_stop_count != 0;

    // One control mode, so joint and Cartesian controllers cannot run together
    const bool joint_running = (position_controller_running_ || velocity_controller_running_
                                   || torque_controller_running_)
                               && stop_modes_.empty();
    const bool cartesian_running = cartesian_controller_running_ && !cartesian_stop_requested_;
    if ((cartesian_start_requested_ && (!start_modes_.empty() || joint_running))
        || (!start_modes_.empty() && cartesian_running)) {
        RCLCPP_ERROR(getLogger(),
            "A joint controller and a Cartesian controller cannot run at the same time. Stop one "
            "before starting the other.");
        return hardware_interface::return_type::ERROR;
    }

    // Zeroed right before the Cartesian mode is entered, off the real-time loop
    if (cartesian_start_requested_ && !ZeroForceTorqueSensor()) {
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivDualHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    if (cartesian_stop_requested_) {
        cartesian_controller_running_ = false;
        StopIfOperational();
        cartesian_config_node_->DisablePassiveForceControl();
    }

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
        robot_pair_->SwitchMode(rdk_control_mode_);

        // The robots reset their joint impedance properties on mode entry, so whatever was set has
        // to be re-applied before any motion is streamed.
        if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the joint impedance properties. The robots would run at "
                "nominal "
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
        robot_pair_->SwitchMode(rdk_control_mode_);

        // The robots reset their joint impedance properties on mode entry, so whatever was set has
        // to be re-applied before any motion is streamed.
        if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the joint impedance properties. The robots would run at "
                "nominal "
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

        // Set to joint torque mode. This is also the step that brings the robots back from IDLE to
        // RT_JOINT_TORQUE after a fault: recovery leaves them operational in IDLE, and restarting
        // the effort controller lands here with a freshly synchronized command buffer.
        robot_pair_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

        // The joint impedance properties do not govern RT_JOINT_TORQUE, so what the driver holds is
        // no longer in effect while the effort controller runs.
        if (joint_impedance_config_node_) {
            joint_impedance_config_node_->MarkNotInEffect();
        }

        torque_controller_running_ = true;
    } else if (cartesian_start_requested_) {
        // Hold the TCPs before user commands arrive
        SynchronizeCommandsWithState();

        // Every start begins from the defaults: the robot resets its settings on mode entry, and
        // the motion limits are reset here
        const CartesianMotionLimits defaults;
        for (size_t robot = 0; robot < 2; robot++) {
            cartesian_max_linear_vel_[robot].store(defaults.max_linear_vel);
            cartesian_max_angular_vel_[robot].store(defaults.max_angular_vel);
            cartesian_max_linear_acc_[robot].store(defaults.max_linear_acc);
            cartesian_max_angular_acc_[robot].store(defaults.max_angular_acc);
        }
        robot_pair_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

        // The joint impedance properties do not govern the Cartesian mode
        if (joint_impedance_config_node_) {
            joint_impedance_config_node_->MarkNotInEffect();
        }

        cartesian_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();
    cartesian_start_requested_ = false;
    cartesian_stop_requested_ = false;

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivDualHardwareInterface, hardware_interface::SystemInterface)

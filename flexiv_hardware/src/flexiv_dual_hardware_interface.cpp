/**
 * @file flexiv_dual_hardware_interface.cpp
 * @brief Hardware interface to a pair of Flexiv robots for ROS 2 control.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

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

// Bounded wait for both robots to become operational during activation.
constexpr std::chrono::seconds kActivationOperationalTimeout {30};
constexpr std::chrono::milliseconds kOperationalPollPeriod {200};
}

namespace flexiv_hardware {

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (hardware_interface::SystemInterface::on_init(params)
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
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // 16 ports per robot
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts * 2, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(
        flexiv::rdk::kIOPorts * 2, std::numeric_limits<double>::quiet_NaN());
    stop_modes_ = {StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE};
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;
    controllers_initialized_ = false;

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

    try {
        if (info_.hardware_parameters.count("external_axis_type")) {
            external_axis_type_ = info_.hardware_parameters.at("external_axis_type");
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN(getLogger(), "Failed to parse external_axis_type, using default empty");
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
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // The joint map below is built from the connected robots, so unlike the single-robot
    // interface the connection has to stay in on_init. on_configure only brings up the recovery
    // interface on top of it.
    driver_status_ = std::make_shared<DriverStatus>();
    executor_ = params.executor;
    robot_system_control_ = std::make_unique<DualRobotSystemControl>(*robot_pair_);

    // Check the DoF of both robots
    if (robot_pair_->info().first.DoF + robot_pair_->info().second.DoF != info_.joints.size()) {
        if (external_axis_type_.find("aico2") != std::string::npos) {
            RCLCPP_WARN(getLogger(),
                "Connected robots total DoF (%ld + %ld = %ld) do not match expected DoF (%ld)",
                robot_pair_->info().first.DoF, robot_pair_->info().second.DoF,
                robot_pair_->info().first.DoF + robot_pair_->info().second.DoF,
                info_.joints.size());
        } else {
            RCLCPP_FATAL(getLogger(),
                "Connected robots total DoF (%ld + %ld = %ld) do not match expected DoF (%ld)",
                robot_pair_->info().first.DoF, robot_pair_->info().second.DoF,
                robot_pair_->info().first.DoF + robot_pair_->info().second.DoF,
                info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Build joint map
    joint_map_.resize(info_.joints.size());
    std::vector<size_t> unmapped_indices;
    const std::string prefix_left = info_.hardware_parameters.at("prefix_left");
    const std::string prefix_right = info_.hardware_parameters.at("prefix_right");

    // Determine external DOFs first
    size_t extra_dof_left = robot_pair_->info().first.DoF_e;
    size_t extra_dof_right = robot_pair_->info().second.DoF_e;

    // For AICO2, external joints on both arms are identical but only mapped once (usually left)
    if (external_axis_type_.find("aico2") != std::string::npos) {
        extra_dof_right = 0;
    }

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

    return command_interfaces;
}

hardware_interface::CallbackReturn FlexivDualHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    driver_status_->driver_state.store(DriverState::FAULT);

    auto executor = executor_.lock();
    if (!executor) {
        RCLCPP_FATAL(getLogger(),
            "No executor available to host the recovery interface. The controller manager must "
            "provide one through HardwareComponentInterfaceParams.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Namespaced by the left robot so that the pair has a single, predictable recovery interface.
    const std::string robot_sn_left = info_.hardware_parameters.at("robot_sn_left");
    recovery_node_
        = std::make_shared<RecoveryNode>(robot_sn_left, *robot_system_control_, driver_status_);
    executor->add_node(recovery_node_->get_node_base_interface());

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

    // A joint of either robot that no ROS joint maps to keeps its nominal value rather than 0. With
    // an AICO2 external axis type the right robot's external axes are deliberately unmapped, and a
    // stiffness of 0 there would leave those axes free-floating.
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

    joint_impedance_config_node_
        = std::make_shared<JointImpedanceConfigNode>(robot_sn_left, std::move(joint_names),
            std::move(bounds), rdk_control_mode_ == flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE,
            driver_status_, std::move(setters));
    executor->add_node(joint_impedance_config_node_->get_node_base_interface());

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

void FlexivDualHardwareInterface::TeardownRecoveryNode()
{
    if (recovery_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(recovery_node_->get_node_base_interface());
        }
        recovery_node_.reset();
    }
    // Torn down here too, since its closures capture this and call through robot_pair_.
    if (joint_impedance_config_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(joint_impedance_config_node_->get_node_base_interface());
        }
        joint_impedance_config_node_.reset();
    }
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

void FlexivDualHardwareInterface::SynchronizeCommandsWithState()
{
    // Called from perform_command_mode_switch(), which is the controller restart the driver
    // requires after a fault. Once the buffers hold the measured position, motion may stream again.
    driver_status_->commands_synchronized.store(true);
    // Position commands start from where the robots actually are, so the first write() after a
    // mode switch commands a hold instead of a stale setpoint.
    hw_commands_joint_positions_ = hw_states_joint_positions_;
    std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(), 0.0);

    // Effort commands stay NaN so that write() skips streaming. A zero torque command is streamed
    // with gravity compensation enabled and would leave the arms floating instead of holding.
    std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
        std::numeric_limits<double>::quiet_NaN());
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

        // Check the DoF of both robots
        if (robot_pair_->info().first.DoF + robot_pair_->info().second.DoF != info_.joints.size()) {
            if (external_axis_type_.find("aico2") != std::string::npos) {
                RCLCPP_WARN(getLogger(),
                    "Connected robots total DoF (%ld + %ld = %ld) do not match expected DoF (%ld)",
                    robot_pair_->info().first.DoF, robot_pair_->info().second.DoF,
                    robot_pair_->info().first.DoF + robot_pair_->info().second.DoF,
                    info_.joints.size());
            } else {
                RCLCPP_FATAL(getLogger(),
                    "Connected robots total DoF (%ld + %ld = %ld) do not match expected DoF (%ld)",
                    robot_pair_->info().first.DoF, robot_pair_->info().second.DoF,
                    robot_pair_->info().first.DoF + robot_pair_->info().second.DoF,
                    info_.joints.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
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
        RCLCPP_FATAL(getLogger(), e.what());
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
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Issue no DRDK call unless the robots are ready. While recovery runs it changes the control
    // mode and the fault state, and this early return is what guarantees the real-time loop is
    // quiescent for the duration without needing a lock on the hot path.
    if (driver_status_->driver_state.load() != DriverState::READY) {
        return hardware_interface::return_type::OK;
    }

    // Initialize target position and velocity vectors
    std::vector<double> target_pos_left(robot_pair_->info().first.DoF);
    std::vector<double> target_vel_left(robot_pair_->info().first.DoF);
    std::vector<double> max_vel_left(robot_pair_->info().first.DoF, kMaxJointVelocity);
    std::vector<double> max_acc_left(robot_pair_->info().first.DoF, kMaxJointAcceleration);

    std::vector<double> target_pos_right(robot_pair_->info().second.DoF);
    std::vector<double> target_vel_right(robot_pair_->info().second.DoF);
    std::vector<double> max_vel_right(robot_pair_->info().second.DoF, kMaxJointVelocity);
    std::vector<double> max_acc_right(robot_pair_->info().second.DoF, kMaxJointAcceleration);

    // Populate target vectors, using current state if command is NaN
    for (size_t i = 0; i < info_.joints.size(); i++) {
        int robot_idx = joint_map_[i].robot_index;
        int dof_idx = joint_map_[i].dof_index;

        if (robot_idx == 0) {
            if (std::isnan(hw_commands_joint_positions_[i])) {
                target_pos_left[dof_idx] = hw_states_joint_positions_[i];
            } else {
                target_pos_left[dof_idx] = hw_commands_joint_positions_[i];
            }
            if (std::isnan(hw_commands_joint_velocities_[i])) {
                target_vel_left[dof_idx] = 0.0;
            } else {
                target_vel_left[dof_idx] = hw_commands_joint_velocities_[i];
            }
        } else {
            if (std::isnan(hw_commands_joint_positions_[i])) {
                target_pos_right[dof_idx] = hw_states_joint_positions_[i];
            } else {
                target_pos_right[dof_idx] = hw_commands_joint_positions_[i];
            }
            if (std::isnan(hw_commands_joint_velocities_[i])) {
                target_vel_right[dof_idx] = 0.0;
            } else {
                target_vel_right[dof_idx] = hw_commands_joint_velocities_[i];
            }
        }
    }

    // For AICO2, duplicate external axis commands from left to right
    if (external_axis_type_.find("aico2") != std::string::npos) {
        if (target_pos_left.size() >= 2 && target_pos_right.size() >= 2) {
            target_pos_right[0] = target_pos_left[0];
            target_pos_right[1] = target_pos_left[1];
            target_vel_right[0] = target_vel_left[0];
            target_vel_right[1] = target_vel_left[1];
        }
    }

    // Withhold motion until a controller restart has re-synchronized the command buffers. Digital
    // outputs further down are unaffected -- they carry no setpoint that can go stale.
    const bool stream_motion = driver_status_->commands_synchronized.load();

    if (stream_motion && position_controller_running_
        && robot_pair_->mode() == std::pair {rdk_control_mode_, rdk_control_mode_}) {
        robot_pair_->SendJointPosition({target_pos_left, target_pos_right},
            {target_vel_left, target_vel_right}, {max_vel_left, max_vel_right},
            {max_acc_left, max_acc_right});
    } else if (stream_motion && velocity_controller_running_
               && robot_pair_->mode() == std::pair {rdk_control_mode_, rdk_control_mode_}) {
        robot_pair_->SendJointPosition({target_pos_left, target_pos_right},
            {target_vel_left, target_vel_right}, {max_vel_left, max_vel_right},
            {max_acc_left, max_acc_right});
    } else if (stream_motion && torque_controller_running_
               && robot_pair_->mode()
                      == std::pair {
                          flexiv::rdk::Mode::RT_JOINT_TORQUE, flexiv::rdk::Mode::RT_JOINT_TORQUE}) {
        std::vector<double> target_torque_left(robot_pair_->info().first.DoF);
        std::vector<double> target_torque_right(robot_pair_->info().second.DoF);

        for (size_t i = 0; i < info_.joints.size(); i++) {
            int robot_idx = joint_map_[i].robot_index;
            int dof_idx = joint_map_[i].dof_index;

            if (robot_idx == 0) {
                if (std::isnan(hw_commands_joint_efforts_[i])) {
                    target_torque_left[dof_idx] = 0.0;
                } else {
                    target_torque_left[dof_idx] = hw_commands_joint_efforts_[i];
                }
            } else {
                if (std::isnan(hw_commands_joint_efforts_[i])) {
                    target_torque_right[dof_idx] = 0.0;
                } else {
                    target_torque_right[dof_idx] = hw_commands_joint_efforts_[i];
                }
            }
        }

        // For AICO2, duplicate external axis commands from left to right
        if (external_axis_type_.find("aico2") != std::string::npos) {
            if (target_torque_left.size() >= 2 && target_torque_right.size() >= 2) {
                target_torque_right[0] = target_torque_left[0];
                target_torque_right[1] = target_torque_left[1];
            }
        }

        robot_pair_->StreamJointTorque({target_torque_left, target_torque_right});
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
    // Check if there is any change in digital outputs before sending
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
    current_digital_outputs_left_.clear();
    current_digital_outputs_right_.clear();
    for (const auto& [port, value] : digital_outputs_left) {
        current_digital_outputs_left_[port] = value;
    }
    for (const auto& [port, value] : digital_outputs_right) {
        current_digital_outputs_right_[port] = value;
    }

    // Set digital outputs
    if (digital_outputs_changed
        && (!digital_outputs_left.empty() || !digital_outputs_right.empty())) {
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

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivDualHardwareInterface::perform_command_mode_switch(
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
    }

    start_modes_.clear();
    stop_modes_.clear();

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivDualHardwareInterface, hardware_interface::SystemInterface)

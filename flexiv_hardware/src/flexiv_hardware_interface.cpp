/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
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

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"
#include "flexiv_hardware/fault_recovery.hpp"

namespace {

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

// Bounded wait for the robot to become operational during activation. Brake release dominates the
// duration; a robot that is not ready within this window needs operator attention.
constexpr std::chrono::seconds kActivationOperationalTimeout {30};
constexpr std::chrono::milliseconds kOperationalPollPeriod {200};

// Bounded wait for the ZeroFTSensor primitive, which takes a few seconds.
constexpr std::chrono::seconds kZeroFTSensorTimeout {10};
constexpr std::chrono::milliseconds kPrimitivePollPeriod {100};

template <size_t N>
bool AllFinite(const std::array<double, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
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
    hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_commands_cartesian_wrench_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_commands_cartesian_velocity_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_states_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    stop_modes_ = {StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE};
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

    // Find 7 arm joints in standard order
    for (int j = 1; j <= 7; ++j) {
        std::string arm_joint_name = prefix + "joint" + std::to_string(j);
        bool found = false;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (info_.joints[i].name == arm_joint_name) {
                arm_indices.push_back(i);
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_FATAL(getLogger(), "Could not find arm joint '%s'", arm_joint_name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
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

    cartesian_command_interface_names_ = CartesianCommandInterfaceNames(prefix);

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
        info_.hardware_parameters.at("robot_sn");
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

    // The connection is established in on_configure, which is the lifecycle stage that owns
    // communication with the hardware. This is what lets a lost connection be recovered by
    // cleaning up and reconfiguring, instead of restarting the whole process.
    driver_status_ = std::make_shared<DriverStatus>();
    executor_ = params.executor;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    const std::string robot_sn = info_.hardware_parameters.at("robot_sn");

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");

    // Check the DoF of the robot against the URDF before exposing any interface.
    if (robot_->info().DoF != info_.joints.size()) {
        RCLCPP_FATAL(getLogger(), "Robot has %ld DoF. Expected %ld (from URDF).",
            robot_->info().DoF, info_.joints.size());
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    robot_system_control_ = std::make_unique<SingleRobotSystemControl>(*robot_);
    driver_status_->driver_state.store(DriverState::FAULT);

    // Host the recovery interface on the controller manager's executor, so that all blocking
    // system control calls happen off the real-time control loop without needing a thread here.
    auto executor = executor_.lock();
    if (!executor) {
        RCLCPP_FATAL(getLogger(),
            "No executor available to host the recovery interface. The controller manager must "
            "provide one through HardwareComponentInterfaceParams.");
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }
    recovery_node_
        = std::make_shared<RecoveryNode>(robot_sn, *robot_system_control_, driver_status_);
    executor->add_node(recovery_node_->get_node_base_interface());

    // The impedance setters only apply to the joint impedance control modes, but the interface is
    // advertised either way so that a request made against a joint_position driver is answered with
    // an explanation instead of a missing service.
    std::vector<std::string> joint_names;
    joint_names.reserve(info_.joints.size());
    for (const auto& joint : info_.joints) {
        joint_names.push_back(joint.name);
    }

    JointImpedanceBounds bounds;
    bounds.k_q_nom = ConvertRDKToROSOrder(robot_->info().K_q_nom, rdk_to_ros_map_);
    bounds.tau_max = ConvertRDKToROSOrder(robot_->info().tau_max, rdk_to_ros_map_);

    // The node works in ROS joint order and knows nothing about the RDK; these three closures are
    // where the order is translated and the RDK is actually called.
    JointImpedanceSetters setters;
    setters.set_joint_impedance
        = [this](const std::vector<double>& k_q, const std::vector<double>& z_q) {
              robot_->SetJointImpedance(ConvertROSToRDKOrder(k_q, rdk_to_ros_map_),
                  ConvertROSToRDKOrder(z_q, rdk_to_ros_map_));
          };
    setters.set_max_contact_torque = [this](const std::vector<double>& max_torques) {
        robot_->SetMaxContactTorque(ConvertROSToRDKOrder(max_torques, rdk_to_ros_map_));
    };
    setters.set_joint_inertia_scale = [this](const std::vector<double>& inertia_scales) {
        robot_->SetJointInertiaScale(ConvertROSToRDKOrder(inertia_scales, rdk_to_ros_map_));
    };

    joint_impedance_config_node_ = std::make_shared<JointImpedanceConfigNode>(robot_sn, joint_names,
        std::move(bounds), rdk_control_mode_ == flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE,
        driver_status_, std::move(setters));
    executor->add_node(joint_impedance_config_node_->get_node_base_interface());

    CartesianMotionForceBounds cartesian_bounds;
    cartesian_bounds.k_x_nom = {robot_->info().K_x_nom};
    cartesian_bounds.q_min = ConvertRDKToROSOrder(robot_->info().q_min, rdk_to_ros_map_);
    cartesian_bounds.q_max = ConvertRDKToROSOrder(robot_->info().q_max, rdk_to_ros_map_);

    // A single robot: every per-robot vector holds exactly one entry.
    CartesianMotionForceSetters cartesian_setters;
    cartesian_setters.set_cartesian_impedance
        = [this](const std::vector<CartesianArray>& k_x, const std::vector<CartesianArray>& z_x) {
              robot_->SetCartesianImpedance(k_x[0], z_x[0]);
          };
    cartesian_setters.set_max_contact_wrench = [this](const std::vector<CartesianArray>& wrench) {
        robot_->SetMaxContactWrench(wrench[0]);
    };
    cartesian_setters.set_null_space_posture = [this](const std::vector<double>& ref_positions) {
        robot_->SetNullSpacePosture(ConvertROSToRDKOrder(ref_positions, rdk_to_ros_map_));
    };
    cartesian_setters.set_null_space_objectives
        = [this](const std::vector<NullSpaceObjectives>& objectives) {
              robot_->SetNullSpaceObjectives(objectives[0].linear_manipulability,
                  objectives[0].angular_manipulability, objectives[0].ref_positions_tracking);
          };
    cartesian_setters.set_force_control_axis
        = [this](const std::vector<CartesianFlags>& enabled_axes,
              const std::vector<LinearArray>& max_linear_vel) {
              robot_->SetForceControlAxis(enabled_axes[0], max_linear_vel[0]);
          };
    cartesian_setters.set_force_control_frame
        = [this](const std::vector<flexiv::rdk::CoordType>& root_coord,
              const std::vector<PoseArray>& t_in_root) {
              robot_->SetForceControlFrame(root_coord[0], t_in_root[0]);
          };
    cartesian_setters.set_passive_force_control
        = [this](const std::vector<bool>& enabled) { robot_->SetPassiveForceControl(enabled[0]); };
    cartesian_setters.set_motion_limits = [this](const std::vector<CartesianMotionLimits>& limits) {
        cartesian_max_linear_vel_.store(limits[0].max_linear_vel);
        cartesian_max_angular_vel_.store(limits[0].max_angular_vel);
        cartesian_max_linear_acc_.store(limits[0].max_linear_acc);
        cartesian_max_angular_acc_.store(limits[0].max_angular_acc);
    };

    cartesian_config_node_
        = std::make_shared<CartesianMotionForceConfigNode>(robot_sn, std::move(joint_names),
            std::move(cartesian_bounds), driver_status_, std::move(cartesian_setters));
    executor->add_node(cartesian_config_node_->get_node_base_interface());

    return hardware_interface::CallbackReturn::SUCCESS;
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

bool FlexivHardwareInterface::ZeroForceTorqueSensor()
{
    if (driver_status_->driver_state.load() != DriverState::READY) {
        RCLCPP_ERROR(getLogger(), "Cannot zero the force/torque sensor: the robot is not ready");
        return false;
    }

    RCLCPP_WARN(getLogger(),
        "Zeroing the force/torque sensor, make sure nothing is in contact with the robot");
    try {
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive(
            "ZeroFTSensor", std::map<std::string, flexiv::rdk::FlexivDataTypes> {});

        const auto deadline = std::chrono::steady_clock::now() + kZeroFTSensorTimeout;
        while (!std::get<int>(robot_->primitive_states()["terminated"])) {
            if (robot_->fault()) {
                throw std::runtime_error("a fault occurred on the robot");
            }
            if (std::chrono::steady_clock::now() > deadline) {
                throw std::runtime_error("the ZeroFTSensor primitive did not finish in time");
            }
            std::this_thread::sleep_for(kPrimitivePollPeriod);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not zero the force/torque sensor: %s", e.what());
        StopIfOperational();
        return false;
    }

    // Back to IDLE, which passive force control requires
    StopIfOperational();
    RCLCPP_INFO(getLogger(), "Force/torque sensor zeroed");
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
    // Torn down before robot_ below, since its closures capture this and call through it.
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
    if (driver_status_) {
        driver_status_->driver_state.store(DriverState::UNINITIALIZED);
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

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        robot_sn, "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_)));

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    for (size_t i = 0; i < flexiv::rdk::kPoseSize; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "tcp", kCartesianPoseInterfaces[i], &hw_states_cartesian_pose_[i]));
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

    for (size_t i = 0; i < flexiv::rdk::kPoseSize; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "tcp", kCartesianPoseInterfaces[i], &hw_commands_cartesian_pose_[i]));
    }
    for (size_t i = 0; i < flexiv::rdk::kCartDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "tcp", kCartesianWrenchInterfaces[i], &hw_commands_cartesian_wrench_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "tcp", kCartesianVelocityInterfaces[i], &hw_commands_cartesian_velocity_[i]));
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

        // Enable the robot
        RCLCPP_INFO(getLogger(), "Enabling robot ...");
        robot_->Enable();

        // Wait for the robot to become operational, bounded so that a robot that never becomes
        // ready fails the activation instead of hanging the controller manager forever.
        if (!WaitUntilOperational(kActivationOperationalTimeout)) {
            RCLCPP_FATAL(getLogger(), "Robot did not become operational within %ld s. %s",
                static_cast<long>(kActivationOperationalTimeout.count()),
                DescribeRobotCondition(robot_system_control_->condition()).c_str());
            return hardware_interface::CallbackReturn::ERROR;
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

    // The robot is enabled but in IDLE: a controller start has to establish the control mode and
    // synchronize the command buffers before any motion may be streamed.
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
        hw_flexiv_robot_states_ = robot_->states();

        // Read joint states
        // Map RDK states (RDK order) to Hardware Interface states (ROS order)
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            if (ros_idx < info_.joints.size()) {
                hw_states_joint_positions_[ros_idx] = hw_flexiv_robot_states_.q[rdk_idx];
                hw_states_joint_velocities_[ros_idx] = hw_flexiv_robot_states_.dtheta[rdk_idx];
                hw_states_joint_efforts_[ros_idx] = hw_flexiv_robot_states_.tau[rdk_idx];
            }
        }
        hw_states_cartesian_pose_ = hw_flexiv_robot_states_.tcp_pose;

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
    if (driver_status_->driver_state.load() != DriverState::READY) {
        return hardware_interface::return_type::OK;
    }

    // Initialize target vectors to hold position
    std::vector<double> target_pos(robot_->info().DoF);
    std::vector<double> target_vel(robot_->info().DoF);

    std::vector<double> max_vel(robot_->info().DoF, kMaxJointVelocity);
    std::vector<double> max_acc(robot_->info().DoF, kMaxJointAcceleration);

    bool is_pos_nan = false;
    bool is_vel_nan = false;
    bool is_eff_nan = false;
    for (std::size_t i = 0; i < robot_->info().DoF; i++) {
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

    if (stream_motion && position_controller_running_ && robot_->mode() == rdk_control_mode_
        && !is_pos_nan) {
        // Map ROS commands to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_pos[rdk_idx] = hw_commands_joint_positions_[ros_idx];
        }
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (stream_motion && velocity_controller_running_ && robot_->mode() == rdk_control_mode_
               && !is_vel_nan) {
        // Map ROS commands/states to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_pos[rdk_idx] = hw_states_joint_positions_[ros_idx];
            target_vel[rdk_idx] = hw_commands_joint_velocities_[ros_idx];
        }
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (stream_motion && torque_controller_running_
               && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE && !is_eff_nan) {
        std::vector<double> target_torque(robot_->info().DoF);
        // Map ROS commands to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_torque[rdk_idx] = hw_commands_joint_efforts_[ros_idx];
        }
        robot_->StreamJointTorque(target_torque, true, true);
    } else if (stream_motion && cartesian_controller_running_
               && robot_->mode() == flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE
               && AllFinite(hw_commands_cartesian_pose_) && AllFinite(hw_commands_cartesian_wrench_)
               && AllFinite(hw_commands_cartesian_velocity_)) {
        robot_->SendCartesianMotionForce(hw_commands_cartesian_pose_, hw_commands_cartesian_wrench_,
            hw_commands_cartesian_velocity_, cartesian_max_linear_vel_.load(),
            cartesian_max_angular_vel_.load(), cartesian_max_linear_acc_.load(),
            cartesian_max_angular_acc_.load());
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

    // The Cartesian commands likewise start as a hold at the measured TCP pose, without force.
    hw_commands_cartesian_pose_ = hw_states_cartesian_pose_;
    hw_commands_cartesian_wrench_.fill(0.0);
    hw_commands_cartesian_velocity_.fill(0.0);
}

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
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

    // The Cartesian command interfaces must all be claimed together
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
        RCLCPP_ERROR(getLogger(), "A controller must claim all %ld Cartesian command interfaces",
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

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    if (cartesian_stop_requested_) {
        cartesian_controller_running_ = false;
        StopIfOperational();
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
    } else if (cartesian_start_requested_) {
        // Hold the TCP before user commands arrive
        SynchronizeCommandsWithState();

        // Passive force control needs IDLE; the rest is reset on mode entry, so re-applied after it
        bool applied = cartesian_config_node_->ApplyBeforeModeEntry();
        if (applied) {
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
            applied = cartesian_config_node_->Reapply();
        }
        if (!applied) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the Cartesian motion-force settings. The robot would run with "
                "default settings instead of the requested ones, so the controller start is "
                "refused.");
            driver_status_->commands_synchronized.store(false);
            StopIfOperational();
            return hardware_interface::return_type::ERROR;
        }

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
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)

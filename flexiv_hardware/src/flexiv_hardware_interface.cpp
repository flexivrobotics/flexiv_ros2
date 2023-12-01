/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/ros2_control_demo_hardware/src/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/Robot.hpp"
#include "flexiv/Exception.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"

namespace flexiv_hardware {

return_type FlexivHardwareInterface::configure(
    const hardware_interface::HardwareInfo& info)
{
    if (configure_default(info) != return_type::OK) {
        return return_type::ERROR;
    }

    hw_states_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_force_torque_sensor_.resize(
        info_.sensors[0].state_interfaces.size(),
        std::numeric_limits<double>::quiet_NaN());
    hw_states_external_wrench_in_base_.resize(
        info_.sensors[1].state_interfaces.size(),
        std::numeric_limits<double>::quiet_NaN());
    hw_states_external_wrench_in_tcp_.resize(
        info_.sensors[2].state_interfaces.size(),
        std::numeric_limits<double>::quiet_NaN());
    hw_states_tcp_pose_.resize(7, std::numeric_limits<double>::quiet_NaN());
    stop_modes_ = {StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE};
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;
    controllers_initialized_ = false;

    if (info_.joints.size() != n_joints) {
        RCLCPP_FATAL(getLogger(), "Got %d joints. Expected %d.",
            info_.joints.size(), n_joints);
        return return_type::ERROR;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has %d command interfaces found. 3 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return return_type::ERROR;
        }

        if (joint.command_interfaces[0].name
            != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return return_type::ERROR;
        }

        if (joint.command_interfaces[1].name
            != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return return_type::ERROR;
        }

        if (joint.command_interfaces[2].name
            != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has %d state interfaces found. 3 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return return_type::ERROR;
        }

        if (joint.state_interfaces[0].name
            != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return return_type::ERROR;
        }

        if (joint.state_interfaces[1].name
            != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return return_type::ERROR;
        }

        if (joint.state_interfaces[2].name
            != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(),
                "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return return_type::ERROR;
        }
    }

    std::string robot_ip;
    try {
        robot_ip = info_.hardware_parameters["robot_ip"];
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' not set");
        return return_type::ERROR;
    }

    std::string local_ip;
    try {
        local_ip = info_.hardware_parameters["local_ip"];
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'local_ip' not set");
        return return_type::ERROR;
    }

    try {
        RCLCPP_INFO(getLogger(),
            "Connecting to robot at \"%s\" from \"%s\" ...", robot_ip.c_str(),
            local_ip.c_str());
        robot_ = std::make_unique<flexiv::Robot>(robot_ip, local_ip);
    } catch (const flexiv::Exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), e.what());
        return return_type::ERROR;
    }

    status_ = hardware_interface::status::CONFIGURED;
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return return_type::OK;
}

rclcpp::Logger FlexivHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivHardwareInterface");
}

std::vector<hardware_interface::StateInterface>
FlexivHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
            &hw_states_joint_efforts_[i]));
    }

    for (std::size_t i = 0; i < info_.sensors.size(); i++) {
        const auto& sensor = info_.sensors[i];
        for (std::size_t j = 0; j < sensor.state_interfaces.size(); j++) {
            if (i == 0) {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(sensor.name,
                        sensor.state_interfaces[j].name,
                        &hw_states_force_torque_sensor_[j]));
            } else if (i == 1) {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(sensor.name,
                        sensor.state_interfaces[j].name,
                        &hw_states_external_wrench_in_base_[j]));
            } else if (i == 2) {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(sensor.name,
                        sensor.state_interfaces[j].name,
                        &hw_states_external_wrench_in_tcp_[j]));
            } else if (i == 3) {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(sensor.name,
                        sensor.state_interfaces[j].name,
                        &hw_states_tcp_pose_[j]));
            }
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
            &hw_commands_joint_efforts_[i]));
    }

    return command_interfaces;
}

return_type FlexivHardwareInterface::start()
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    try {
        robot_->enable();
    } catch (const flexiv::Exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable robot.");
        RCLCPP_FATAL(getLogger(), e.what());
        return return_type::ERROR;
    }

    // Wait for the robot to become operational
    while (!robot_->isOperational()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    RCLCPP_INFO(getLogger(), "Robot is now operational");

    status_ = hardware_interface::status::STARTED;
    RCLCPP_INFO(getLogger(), "System successfully started!");

    return return_type::OK;
}

return_type FlexivHardwareInterface::stop()
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    robot_->stop();
    robot_->disconnect();

    status_ = hardware_interface::status::STOPPED;
    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return return_type::OK;
}

return_type FlexivHardwareInterface::read()
{
    if (status_ != hardware_interface::status::STARTED) {
        RCLCPP_FATAL(getLogger(), "Hardware not started!");
        return return_type::ERROR;
    }

    flexiv::RobotStates robot_states;
    if (robot_->isOperational() && robot_->getMode() != flexiv::Mode::IDLE) {
        robot_->getRobotStates(robot_states);

        hw_states_joint_positions_ = robot_states.q;
        hw_states_joint_velocities_ = robot_states.dtheta;
        hw_states_joint_efforts_ = robot_states.tau;

        hw_states_force_torque_sensor_ = robot_states.ftSensorRaw;
        hw_states_external_wrench_in_base_ = robot_states.extWrenchInBase;
        hw_states_external_wrench_in_tcp_ = robot_states.extWrenchInTcp;

        // Convert quaternion order from [w, x, y, z] to [x, y, z, w]
        hw_states_tcp_pose_[0] = robot_states.tcpPose[0];
        hw_states_tcp_pose_[1] = robot_states.tcpPose[1];
        hw_states_tcp_pose_[2] = robot_states.tcpPose[2];
        hw_states_tcp_pose_[3] = robot_states.tcpPose[4];
        hw_states_tcp_pose_[4] = robot_states.tcpPose[5];
        hw_states_tcp_pose_[5] = robot_states.tcpPose[6];
        hw_states_tcp_pose_[6] = robot_states.tcpPose[3];
    }

    return return_type::OK;
}

return_type FlexivHardwareInterface::write()
{
    if (status_ != hardware_interface::status::STARTED) {
        RCLCPP_FATAL(getLogger(), "Hardware not started!");
        return return_type::ERROR;
    }

    std::vector<double> targetAcceleration(n_joints, 0);
    std::vector<double> targetVelocity(n_joints, 0);
    std::fill(targetAcceleration.begin(), targetAcceleration.end(), 0.0);
    std::fill(targetVelocity.begin(), targetVelocity.end(), 0.0);

    bool isNanPos = false;
    bool isNanVel = false;
    bool isNanEff = false;
    for (std::size_t i = 0; i < n_joints; i++) {
        if (hw_commands_joint_positions_[i] != hw_commands_joint_positions_[i])
            isNanPos = true;
        if (hw_commands_joint_velocities_[i]
            != hw_commands_joint_velocities_[i])
            isNanVel = true;
        if (hw_commands_joint_efforts_[i] != hw_commands_joint_efforts_[i])
            isNanEff = true;
    }

    if (position_controller_running_
        && robot_->getMode() == flexiv::Mode::RT_JOINT_POSITION && !isNanPos) {
        robot_->streamJointPosition(
            hw_commands_joint_positions_, targetVelocity, targetAcceleration);
    } else if (velocity_controller_running_
               && robot_->getMode() == flexiv::Mode::RT_JOINT_POSITION
               && !isNanVel) {
        robot_->streamJointPosition(hw_states_joint_positions_,
            hw_commands_joint_velocities_, targetAcceleration);
    } else if (torque_controller_running_
               && robot_->getMode() == flexiv::Mode::RT_JOINT_TORQUE
               && !isNanEff) {
        robot_->streamJointTorque(hw_commands_joint_efforts_, true, true);
    }

    return return_type::OK;
}

return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    start_modes_.clear();
    stop_modes_.clear();

    // Starting interfaces
    for (const auto& key : start_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key
                == info_.joints[i].name + "/"
                       + hardware_interface::HW_IF_POSITION) {
                start_modes_.push_back(hardware_interface::HW_IF_POSITION);
            }
            if (key
                == info_.joints[i].name + "/"
                       + hardware_interface::HW_IF_VELOCITY) {
                start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
            }
            if (key
                == info_.joints[i].name + "/"
                       + hardware_interface::HW_IF_EFFORT) {
                start_modes_.push_back(hardware_interface::HW_IF_EFFORT);
            }
        }
    }
    // All joints must be given new command mode at the same time
    if (start_modes_.size() != 0
        && start_modes_.size() != info_.joints.size()) {
        return return_type::ERROR;
    }
    // All joints must have the same command mode
    if (start_modes_.size() != 0
        && !std::equal(start_modes_.begin() + 1, start_modes_.end(),
            start_modes_.begin())) {
        return return_type::ERROR;
    }

    // Stop motion on all relevant joints that are stopping
    for (const auto& key : stop_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key
                == info_.joints[i].name + "/"
                       + hardware_interface::HW_IF_POSITION) {
                stop_modes_.push_back(StoppingInterface::STOP_POSITION);
            }
            if (key
                == info_.joints[i].name + "/"
                       + hardware_interface::HW_IF_VELOCITY) {
                stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
            }
            if (key
                == info_.joints[i].name + "/"
                       + hardware_interface::HW_IF_EFFORT) {
                stop_modes_.push_back(StoppingInterface::STOP_EFFORT);
            }
        }
    }
    // stop all interfaces at the same time
    if (stop_modes_.size() != 0
        && (stop_modes_.size() != info_.joints.size()
            || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(),
                stop_modes_.begin()))) {
        return return_type::ERROR;
    }

    controllers_initialized_ = true;
    return return_type::OK;
}

return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    if (stop_modes_.size() != 0
        && std::find(stop_modes_.begin(), stop_modes_.end(),
               StoppingInterface::STOP_POSITION)
               != stop_modes_.end()) {
        position_controller_running_ = false;
        robot_->stop();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(),
                      StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        robot_->stop();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(),
                      StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        robot_->stop();
    }

    if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(),
               hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_positions_.begin(),
            hw_commands_joint_positions_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position mode
        robot_->setMode(flexiv::Mode::RT_JOINT_POSITION);

        position_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(start_modes_.begin(), start_modes_.end(),
                      hardware_interface::HW_IF_VELOCITY)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_velocities_.begin(),
            hw_commands_joint_velocities_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position mode
        robot_->setMode(flexiv::Mode::RT_JOINT_POSITION);

        velocity_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(start_modes_.begin(), start_modes_.end(),
                      hardware_interface::HW_IF_EFFORT)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        std::fill(hw_commands_joint_efforts_.begin(),
            hw_commands_joint_efforts_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint torque mode
        robot_->setMode(flexiv::Mode::RT_JOINT_TORQUE);

        torque_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();

    return return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(flexiv_hardware::FlexivHardwareInterface,
    hardware_interface::SystemInterface)

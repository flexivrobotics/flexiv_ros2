/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// ros2_control hardware_interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// Flexiv
#include "flexiv/rdk/robot.hpp"

#include "flexiv_hardware/fault_recovery.hpp"
#include "flexiv_hardware/joint_impedance_config_node.hpp"
#include "flexiv_hardware/recovery_node.hpp"
#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT
};

class FlexivHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief [Blocking] Wait for the robot to become operational, up to [timeout]. Logs the
     * operational status while waiting so a stuck robot explains itself.
     * @return True if the robot became operational, false on timeout.
     */
    bool WaitUntilOperational(std::chrono::seconds timeout);

    /**
     * @brief Set the joint command buffers to hold the currently measured position, so that
     * resuming control cannot apply a stale command.
     */
    void SynchronizeCommandsWithState();

    /**
     * @brief Notice a robot that was moved while the driver was not ready, and warn about it once
     * on the return to READY. Called from read().
     */
    void TrackPositionChangeAcrossInterruption();

    /**
     * @brief [Blocking] Stop the robot, but only if it is operational. Stop() switches the control
     * mode internally, which the robot rejects unless it is operational -- and a robot that is not
     * operational is not executing anything, so there is nothing to stop.
     */
    void StopIfOperational();

    /** @brief Tear down the recovery node and release the robot connection. */
    void Disconnect();

    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;

    // Recovery interface, hosted on the controller manager's executor
    std::unique_ptr<RobotSystemControl> robot_system_control_;
    std::shared_ptr<DriverStatus> driver_status_;
    std::shared_ptr<RecoveryNode> recovery_node_;
    rclcpp::Executor::WeakPtr executor_;

    // Joint impedance interface, hosted on the same executor. Only brought up when the driver runs
    // in a joint impedance control mode.
    std::shared_ptr<JointImpedanceConfigNode> joint_impedance_config_node_;

    // RDK control mode for joint position and velocity interfaces
    flexiv::rdk::Mode rdk_control_mode_;

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Robot States
    flexiv::rdk::RobotStates hw_flexiv_robot_states_;
    flexiv::rdk::RobotStates* hw_flexiv_robot_states_addr_ = &hw_flexiv_robot_states_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;

    // Joint positions as last measured before the driver left READY, for detecting a robot that
    // was moved while it was not being commanded. Empty while the driver is ready.
    std::vector<double> positions_before_interruption_;
    bool was_ready_ = false;

    std::vector<double> hw_states_gpio_in_;

    // Map from RDK joint index to ROS joint index
    // RDK expects: [ext_axis_1, ..., ext_axis_N, arm_joint_1, ..., arm_joint_7]
    std::vector<size_t> rdk_to_ros_map_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static rclcpp::Logger getLogger();

    // Control modes
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */

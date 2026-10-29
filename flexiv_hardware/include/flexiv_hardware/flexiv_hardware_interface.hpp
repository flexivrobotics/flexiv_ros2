/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <utility>
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

#include "flexiv_hardware/cartesian_motion_force_config_node.hpp"
#include "flexiv_hardware/fault_recovery.hpp"
#include "flexiv_hardware/interface_bindings.hpp"
#include "flexiv_hardware/joint_impedance_config_node.hpp"
#include "flexiv_hardware/joint_motion_limits.hpp"
#include "flexiv_hardware/recovery_node.hpp"
#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

/**
 * Maximum number of RDK-commandable joint groups this interface can drive: an optional external
 * axis group plus up to two single arms (EXT_AXIS, ARM_1, ARM_2).
 */
constexpr size_t kMaxJointGroups = 3;

/**
 * ROS 2 joint command interface types that this hardware interface can claim and drive.
 * kInterfaceNone means the group is unclaimed.
 */
enum CommandInterfaceType : uint8_t
{
    kInterfaceNone = 0,
    kInterfacePosition,
    kInterfaceVelocity,
    kInterfaceEffort,
};

using JointGroupClaims = std::array<uint8_t, kMaxJointGroups>;

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

    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
    on_export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface::SharedPtr>
    on_export_command_interfaces() override;

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
     * @brief [Blocking] Connect to the robot named by the 'robot_sn' hardware parameter, unless a
     * connection is already open.
     * @return True on success, false if the connection could not be established.
     */
    bool Connect();

    /**
     * @brief Resolve the robot's commandable joint groups against the joint groups the URDF
     * resolved in on_init(), and size the per-group command buffers.
     * @return False if the robot and the URDF disagree.
     */
    bool ResolveJointGroups();

    /**
     * @brief [Blocking] Wait for the robot to become operational, up to [timeout]. Logs the
     * operational status while waiting so a stuck robot explains itself.
     * @return True if the robot became operational, false on timeout.
     */
    bool WaitUntilOperational(std::chrono::seconds timeout);

    /**
     * @brief Set the command buffers of the given joint groups, and the Cartesian commands if
     * [cartesian], to hold the currently measured state, so that a newly started controller cannot
     * apply a stale command.
     */
    void SynchronizeCommandsWithState(
        const std::array<bool, kMaxJointGroups>& groups, bool cartesian);

    /**
     * @brief Advance the velocity control targets of joint group [g] by the commanded velocities
     * over one cycle, keeping each within kMaxVelocityTargetLead of the measured position.
     */
    void AdvanceVelocityTargets(size_t g, double dt);

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

    /**
     * @brief [Blocking] Zero the force/torque sensors of every arm with the ZeroFTSensor primitive,
     * then return the robot to IDLE. Called before entering the Cartesian motion-force mode.
     * @return False if the robot is not ready, faulted or the primitive did not finish in time.
     */
    bool ZeroForceTorqueSensors();

    /**
     * @brief Send the motion commands of the running controllers, if any. Throws whatever the RDK
     * throws, which write() handles.
     * @return True if a real-time command was streamed.
     */
    bool SendMotionCommands(double dt);

    /** @brief Joint-space part of SendMotionCommands(). */
    bool SendJointCommands(double dt);

    /** @brief Cartesian part of SendMotionCommands(). */
    bool SendCartesianCommands();

    /**
     * @brief Resolve which joint groups a set of command interface names fully claims, and with
     * which interface type.
     * @param[in] keys Command interface names to resolve.
     * @param[out] claimed Per joint group, the claimed CommandInterfaceType.
     * @return False when a joint group would be only partially claimed, or claimed with more than
     *         one interface type at once.
     */
    bool ResolveClaimedGroups(
        const std::vector<std::string>& keys, JointGroupClaims& claimed) const;

    /**
     * @brief Count of the Cartesian command interfaces among [keys].
     */
    size_t CountCartesianInterfaces(const std::vector<std::string>& keys) const;

    /**
     * @brief RDK control mode implied by what is currently claimed: the Cartesian mode, the joint
     * torque mode, the joint motion mode, or Mode::UNKNOWN when nothing is claimed.
     */
    flexiv::rdk::Mode RequiredRdkMode() const;

    /**
     * @brief Bring up the recovery, joint impedance and Cartesian motion-force nodes on the
     * controller manager's executor.
     * @return True on success, false if no executor is available.
     */
    bool StartSupervisoryNodes(const std::string& robot_sn);

    /** @brief Tear down the supervisory nodes and release the robot connection. */
    void Disconnect();

    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;

    // Recovery interface, hosted on the controller manager's executor
    std::unique_ptr<RobotSystemControl> robot_system_control_;
    std::shared_ptr<DriverStatus> driver_status_;
    std::shared_ptr<RecoveryNode> recovery_node_;
    rclcpp::Executor::WeakPtr executor_;

    // Joint impedance interface, hosted on the same executor. Advertised regardless of the
    // configured control mode, so that a request against a joint_position driver is answered with
    // an explanation instead of a missing service.
    std::shared_ptr<JointImpedanceConfigNode> joint_impedance_config_node_;

    // Arm joint groups in RDK order, and the DoF of each. The impedance setters are issued one
    // group at a time, and only these groups report a nominal joint stiffness.
    std::vector<std::pair<flexiv::rdk::JointGroup, size_t>> impedance_groups_;

    // Index is the RDK arm-joint index, value is the index into the impedance joint list.
    std::vector<size_t> impedance_rdk_to_ros_map_;

    // Cartesian motion-force interface, hosted on the same executor
    std::shared_ptr<CartesianMotionForceConfigNode> cartesian_config_node_;

    // Index is the RDK arm-joint index over all arms, value is the index into the Cartesian node's
    // arm joint list.
    std::vector<size_t> cartesian_rdk_to_ros_map_;

    // RDK control modes for the joint position and velocity interfaces and for the Cartesian
    // interfaces, real-time (streamed every cycle) or non-real-time
    flexiv::rdk::Mode rdk_control_mode_;
    flexiv::rdk::Mode rdk_cartesian_mode_;
    bool rdk_realtime_ = true;
    // Withhold motion once real-time commands arrive late too often, or only warn as the RDK does
    bool withhold_on_timeliness_failure_ = false;

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;
    // Position targets for velocity control, advanced by the commanded velocity every cycle
    std::vector<double> velocity_targets_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Limits passed with every non-real-time joint position command, in RDK order
    std::vector<double> max_joint_vel_;
    std::vector<double> max_joint_acc_;

    // Per-cycle targets in RDK order. A real-time mode must receive a command every cycle, so a
    // joint group without a valid command keeps streaming the last target it held.
    std::vector<double> target_pos_;
    std::vector<double> target_vel_;
    std::vector<double> target_torque_;

    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointPositionCmd> rt_joint_position_cmds_;
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointTorqueCmd> rt_joint_torque_cmds_;
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtJointPositionCmd> nrt_joint_position_cmds_;
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtCartesianCmd> rt_cartesian_cmds_;
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtCartesianCmd> nrt_cartesian_cmds_;

    // Whether the timeliness failure flag has been seen clear since the last controller start. The
    // RDK flag does not decay while nothing is streamed, so a restart begins with it still raised.
    bool timeliness_clear_since_sync_ = false;

    // Cartesian commands and states of each arm, in RDK order, parallel to arm_groups_
    std::vector<std::array<double, flexiv::rdk::kPoseSize>> hw_commands_cartesian_pose_;
    std::vector<std::array<double, flexiv::rdk::kCartDoF>> hw_commands_cartesian_wrench_;
    std::vector<std::array<double, flexiv::rdk::kCartDoF>> hw_commands_cartesian_velocity_;
    std::vector<std::array<double, flexiv::rdk::kPoseSize>> hw_states_cartesian_pose_;
    // Last pose streamed in the real-time Cartesian mode, held while the commands are not finite
    std::vector<std::array<double, flexiv::rdk::kPoseSize>> last_cartesian_target_;

    // Limits passed with every non-real-time Cartesian command of each arm, written by the
    // Cartesian config node
    std::array<std::atomic<double>, kMaxJointGroups> cartesian_max_linear_vel_;
    std::array<std::atomic<double>, kMaxJointGroups> cartesian_max_angular_vel_;
    std::array<std::atomic<double>, kMaxJointGroups> cartesian_max_linear_acc_;
    std::array<std::atomic<double>, kMaxJointGroups> cartesian_max_angular_acc_;

    // Joint prefix of each arm, naming its "<prefix>tcp" interfaces, ARM_1 (left) first
    std::vector<std::string> arm_prefixes_;

    // Full names of the Cartesian command interfaces of all arms, for matching controller claims
    std::vector<std::string> cartesian_command_interface_names_;

    // Robot states exported per joint group.
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RobotStates> hw_flexiv_robot_states_by_group_;
    std::map<flexiv::rdk::JointGroup, double> hw_flexiv_robot_state_handles_by_group_;

    // Exported interfaces, mirroring the buffers above
    InterfaceBindings interfaces_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;

    // Joint positions as last measured before the driver left READY, for detecting a robot that
    // was moved while it was not being commanded. Empty while the driver is ready.
    std::vector<double> positions_before_interruption_;
    bool was_ready_ = false;

    // Map from RDK joint index to ROS joint index
    // RDK expects: [ext_axis_1, ..., ext_axis_N, arm_joint_1, ..., arm_joint_7]
    std::vector<size_t> rdk_to_ros_map_;

    // Commandable RDK joint groups and their DoF, ordered [EXT_AXIS, ARM_1, ARM_2] to match
    // rdk_to_ros_map_. The groups are named on configure, once the robot reports them.
    std::vector<std::pair<flexiv::rdk::JointGroup, size_t>> active_groups_;

    // Index into active_groups_ of each arm, parallel to arm_prefixes_
    std::vector<size_t> arm_groups_;

    // Joint command interface type currently claimed on each joint group, parallel to
    // active_groups_, and whether a controller holds the Cartesian interfaces
    JointGroupClaims claimed_interfaces_ {};
    bool cartesian_claimed_ = false;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static rclcpp::Logger getLogger();

    // Clock for the throttled logging macros on error paths.
    rclcpp::Clock log_clock_ {RCL_STEADY_TIME};
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */

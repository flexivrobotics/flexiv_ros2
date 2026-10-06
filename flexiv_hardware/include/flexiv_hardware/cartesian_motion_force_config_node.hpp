/**
 * @file cartesian_motion_force_config_node.hpp
 * @brief ROS node hosted by the hardware interface, exposing the settings of the robot's unified
 * Cartesian motion-force controller used in the Cartesian motion-force control mode.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__CARTESIAN_MOTION_FORCE_CONFIG_NODE_HPP_
#define FLEXIV_HARDWARE__CARTESIAN_MOTION_FORCE_CONFIG_NODE_HPP_

#include <array>
#include <functional>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "flexiv/rdk/data.hpp"

#include "flexiv_msgs/srv/set_cartesian_impedance.hpp"
#include "flexiv_msgs/srv/set_cartesian_motion_limits.hpp"
#include "flexiv_msgs/srv/set_force_control_axis.hpp"
#include "flexiv_msgs/srv/set_force_control_frame.hpp"
#include "flexiv_msgs/srv/set_max_contact_wrench.hpp"
#include "flexiv_msgs/srv/set_null_space_objectives.hpp"
#include "flexiv_msgs/srv/set_null_space_posture.hpp"
#include "flexiv_msgs/srv/set_passive_force_control.hpp"

#include "flexiv_hardware/fault_recovery.hpp"

namespace flexiv_hardware {

//======================================= INTERFACE NAMES ==========================================

/** Interfaces of the "<prefix>tcp" component, in RDK order: [x, y, z, qw, qx, qy, qz]. */
constexpr std::array<const char*, flexiv::rdk::kPoseSize> kCartesianPoseInterfaces
    = {"cartesian_pose_x", "cartesian_pose_y", "cartesian_pose_z", "cartesian_pose_qw",
        "cartesian_pose_qx", "cartesian_pose_qy", "cartesian_pose_qz"};
constexpr std::array<const char*, flexiv::rdk::kCartDoF> kCartesianWrenchInterfaces
    = {"cartesian_wrench_fx", "cartesian_wrench_fy", "cartesian_wrench_fz", "cartesian_wrench_mx",
        "cartesian_wrench_my", "cartesian_wrench_mz"};
constexpr std::array<const char*, flexiv::rdk::kCartDoF> kCartesianVelocityInterfaces
    = {"cartesian_velocity_vx", "cartesian_velocity_vy", "cartesian_velocity_vz",
        "cartesian_velocity_wx", "cartesian_velocity_wy", "cartesian_velocity_wz"};

/**
 * @brief [Non-blocking] Full names of the Cartesian command interfaces of one robot, pose first,
 * then wrench, then velocity.
 * @param[in] prefix Joint prefix of the robot, e.g. "Rizon4s-123456_".
 */
std::vector<std::string> CartesianCommandInterfaceNames(const std::string& prefix);

//==================================== SETTING VALUE TYPES ========================================

using CartesianArray = std::array<double, flexiv::rdk::kCartDoF>;
using CartesianFlags = std::array<bool, flexiv::rdk::kCartDoF>;
using LinearArray = std::array<double, flexiv::rdk::kCartDoF / 2>;
using PoseArray = std::array<double, flexiv::rdk::kPoseSize>;

/** Damping ratio the robot uses when SetCartesianImpedance() is given no Z_x. */
constexpr double kNominalCartesianDampingRatio = 0.7;

/** Arguments of SetNullSpaceObjectives() for one robot, defaulting to the RDK defaults. */
struct NullSpaceObjectives
{
    double linear_manipulability = 0.0;
    double angular_manipulability = 0.0;
    double ref_positions_tracking = 0.5;
};

/** Limits passed with every SendCartesianMotionForce() of one robot, defaulting to the RDK's. */
struct CartesianMotionLimits
{
    double max_linear_vel = 0.5;  // [m/s]
    double max_angular_vel = 1.0; // [rad/s]
    double max_linear_acc = 2.0;  // [m/s^2]
    double max_angular_acc = 5.0; // [rad/s^2]
};

/**
 * @brief The calls the node makes, supplied by the hardware interface. Every vector holds one entry
 * per robot, except the null-space posture, which is ROS-ordered over all joints. All but
 * set_motion_limits are blocking RDK setters and throw as the RDK does.
 */
struct CartesianMotionForceSetters
{
    std::function<void(
        const std::vector<CartesianArray>& k_x, const std::vector<CartesianArray>& z_x)>
        set_cartesian_impedance;
    std::function<void(const std::vector<CartesianArray>& max_wrench)> set_max_contact_wrench;
    std::function<void(const std::vector<double>& ref_positions)> set_null_space_posture;
    std::function<void(const std::vector<NullSpaceObjectives>& objectives)>
        set_null_space_objectives;
    std::function<void(const std::vector<CartesianFlags>& enabled_axes,
        const std::vector<LinearArray>& max_linear_vel)>
        set_force_control_axis;
    std::function<void(const std::vector<flexiv::rdk::CoordType>& root_coord,
        const std::vector<PoseArray>& t_in_root)>
        set_force_control_frame;
    std::function<void(const std::vector<bool>& enabled)> set_passive_force_control;
    /** Non-blocking: stores the limits the control loop passes to SendCartesianMotionForce(). */
    std::function<void(const std::vector<CartesianMotionLimits>& limits)> set_motion_limits;
};

/** @brief Bounds the RDK validates against. */
struct CartesianMotionForceBounds
{
    std::vector<CartesianArray> k_x_nom; // One per robot [N/m]:[Nm/rad]
    std::vector<double> q_min;           // ROS joint order [rad]
    std::vector<double> q_max;           // ROS joint order [rad]
};

//========================================== THE NODE ==============================================

/**
 * @brief Node that configures the robot's unified Cartesian motion-force controller, hosted on the
 * controller manager's executor. As in the RDK, the settings are only accepted while the robot is
 * in the Cartesian motion-force mode, except passive force control, which is only accepted in IDLE.
 * Nothing is held or re-applied, so every controller start begins from the robot's defaults.
 */
class CartesianMotionForceConfigNode : public rclcpp::Node
{
public:
    using SetCartesianImpedance = flexiv_msgs::srv::SetCartesianImpedance;
    using SetCartesianMotionLimits = flexiv_msgs::srv::SetCartesianMotionLimits;
    using SetForceControlAxis = flexiv_msgs::srv::SetForceControlAxis;
    using SetForceControlFrame = flexiv_msgs::srv::SetForceControlFrame;
    using SetMaxContactWrench = flexiv_msgs::srv::SetMaxContactWrench;
    using SetNullSpaceObjectives = flexiv_msgs::srv::SetNullSpaceObjectives;
    using SetNullSpacePosture = flexiv_msgs::srv::SetNullSpacePosture;
    using SetPassiveForceControl = flexiv_msgs::srv::SetPassiveForceControl;

    /**
     * @param[in] robot_sn Serial number of the robot, used as the node namespace. For a robot pair,
     * the left robot's.
     * @param[in] joint_names ROS joint names in URDF order, indexing the null-space posture.
     * @param[in] bounds Bounds, with one k_x_nom per robot. Its size sets the number of robots.
     * @param[in] status Status shared with the real-time control loop. Must outlive this node.
     * @param[in] setters The calls to make. Must stay valid for the lifetime of this node.
     */
    CartesianMotionForceConfigNode(const std::string& robot_sn,
        std::vector<std::string> joint_names, CartesianMotionForceBounds bounds,
        std::shared_ptr<DriverStatus> status, CartesianMotionForceSetters setters);

    ~CartesianMotionForceConfigNode() override;

    /**
     * @brief [Blocking] Disable passive force control if a request enabled it, since the robot
     * keeps it across mode entries. Called from perform_command_mode_switch() once the Cartesian
     * controller has stopped and the robot is in IDLE.
     * @return True if there was nothing to disable or the setter succeeded. Never throws.
     */
    bool DisablePassiveForceControl();

private:
    size_t num_robots() const { return bounds_.k_x_nom.size(); }

    /**
     * @brief Check the preconditions shared by all services.
     * @param[in] idle_only Whether the setter is only accepted in IDLE rather than in a Cartesian
     * motion-force mode.
     * @return False if the request must be refused, with [message] explaining why.
     */
    bool CheckPreconditions(bool idle_only, std::string& message) const;

    /**
     * @brief Shared flow of all services: refuse on [error] or a failed precondition, otherwise
     * deliver.
     * @param[in] error Validation error, empty if the request is valid.
     * @param[in] deliver Makes the call.
     */
    void Serve(const std::string& property, const std::string& error, bool idle_only,
        const std::function<void()>& deliver, bool& success, std::string& message);

    void HandleSetCartesianImpedance(const std::shared_ptr<SetCartesianImpedance::Request> request,
        std::shared_ptr<SetCartesianImpedance::Response> response);
    void HandleSetCartesianMotionLimits(
        const std::shared_ptr<SetCartesianMotionLimits::Request> request,
        std::shared_ptr<SetCartesianMotionLimits::Response> response);
    void HandleSetForceControlAxis(const std::shared_ptr<SetForceControlAxis::Request> request,
        std::shared_ptr<SetForceControlAxis::Response> response);
    void HandleSetForceControlFrame(const std::shared_ptr<SetForceControlFrame::Request> request,
        std::shared_ptr<SetForceControlFrame::Response> response);
    void HandleSetMaxContactWrench(const std::shared_ptr<SetMaxContactWrench::Request> request,
        std::shared_ptr<SetMaxContactWrench::Response> response);
    void HandleSetNullSpaceObjectives(
        const std::shared_ptr<SetNullSpaceObjectives::Request> request,
        std::shared_ptr<SetNullSpaceObjectives::Response> response);
    void HandleSetNullSpacePosture(const std::shared_ptr<SetNullSpacePosture::Request> request,
        std::shared_ptr<SetNullSpacePosture::Response> response);
    void HandleSetPassiveForceControl(
        const std::shared_ptr<SetPassiveForceControl::Request> request,
        std::shared_ptr<SetPassiveForceControl::Response> response);

    const std::vector<std::string> joint_names_;
    const CartesianMotionForceBounds bounds_;
    std::shared_ptr<DriverStatus> status_;
    CartesianMotionForceSetters setters_;

    rclcpp::Service<SetCartesianImpedance>::SharedPtr set_cartesian_impedance_service_;
    rclcpp::Service<SetCartesianMotionLimits>::SharedPtr set_cartesian_motion_limits_service_;
    rclcpp::Service<SetForceControlAxis>::SharedPtr set_force_control_axis_service_;
    rclcpp::Service<SetForceControlFrame>::SharedPtr set_force_control_frame_service_;
    rclcpp::Service<SetMaxContactWrench>::SharedPtr set_max_contact_wrench_service_;
    rclcpp::Service<SetNullSpaceObjectives>::SharedPtr set_null_space_objectives_service_;
    rclcpp::Service<SetNullSpacePosture>::SharedPtr set_null_space_posture_service_;
    rclcpp::Service<SetPassiveForceControl>::SharedPtr set_passive_force_control_service_;

    /** Shared by all services, so two blocking RDK calls are never in flight at once. */
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    /** Whether a request has enabled passive force control on any robot. */
    std::atomic<bool> passive_force_control_enabled_ {false};
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__CARTESIAN_MOTION_FORCE_CONFIG_NODE_HPP_ */

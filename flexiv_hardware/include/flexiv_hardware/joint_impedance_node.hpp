/**
 * @file joint_impedance_node.hpp
 * @brief ROS node hosted by the hardware interface, exposing the impedance properties of the
 * robot's joint motion controller used in the joint impedance control modes.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__JOINT_IMPEDANCE_NODE_HPP_
#define FLEXIV_HARDWARE__JOINT_IMPEDANCE_NODE_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "flexiv_msgs/msg/joint_impedance.hpp"
#include "flexiv_msgs/msg/joint_inertia_scale.hpp"
#include "flexiv_msgs/msg/max_contact_torque.hpp"
#include "flexiv_msgs/srv/set_joint_impedance.hpp"
#include "flexiv_msgs/srv/set_joint_inertia_scale.hpp"
#include "flexiv_msgs/srv/set_max_contact_torque.hpp"

#include "flexiv_hardware/fault_recovery.hpp"

namespace flexiv_hardware {

/** Damping ratio the robot uses when SetJointImpedance() is given an empty Z_q. */
constexpr double kNominalDampingRatio = 0.7;
constexpr double kMinDampingRatio = 0.3;
constexpr double kMaxDampingRatio = 0.8;

/** Inertia shaping scale that means no shaping. */
constexpr double kNominalInertiaScale = 1.0;
constexpr double kMinInertiaScale = 0.75;
constexpr double kMaxInertiaScale = 1.0;

//======================================== JOINT ORDERING ==========================================

/**
 * @brief Where one ROS joint lives in a robot pair. Declared here so that the dual hardware
 * interface can build the map from its own joint map without exposing its internals.
 */
struct PairJointIndex
{
    int robot_index; // 0: Left, 1: Right
    int dof_index;   // Index in that robot's joint vector
};

/**
 * @brief [Non-blocking] Reorder a ROS-ordered vector into RDK order, the same permutation write()
 * applies.
 * @param[in] rdk_to_ros_map Index is the RDK index, value is the ROS index.
 * @throw std::invalid_argument if any mapped index is out of range for [ros_values].
 */
std::vector<double> PermuteRosToRdk(
    const std::vector<double>& ros_values, const std::vector<size_t>& rdk_to_ros_map);

/**
 * @brief [Non-blocking] Reorder an RDK-ordered vector into ROS order. Inverse of PermuteRosToRdk().
 * @return Vector of size rdk_to_ros_map.size(). Entries no RDK index maps to stay 0.
 * @throw std::invalid_argument if any mapped index is out of range for the result.
 */
std::vector<double> GatherRdkToRos(
    const std::vector<double>& rdk_values, const std::vector<size_t>& rdk_to_ros_map);

/**
 * @brief [Non-blocking] Split a ROS-ordered vector into the left/right pair DRDK expects.
 * @param[in] fill_left,fill_right Starting values for each robot, one per joint of that robot. A
 * joint that no ROS joint maps to keeps its fill value. This matters: with an AICO2 external axis
 * type the right robot's external axes are deliberately left unmapped, and a stiffness of 0 there
 * would make those axes free-floating, so the robot's own nominal value is used instead.
 * @throw std::invalid_argument if [ros_values] and [joint_map] differ in size, or a mapped index is
 * out of range for its fill vector.
 */
std::pair<std::vector<double>, std::vector<double>> SplitRosToPair(
    const std::vector<double>& ros_values, const std::vector<PairJointIndex>& joint_map,
    const std::vector<double>& fill_left, const std::vector<double>& fill_right);

/**
 * @brief [Non-blocking] Gather a left/right pair back into ROS order. Inverse of SplitRosToPair().
 * @throw std::invalid_argument if a mapped index is out of range for its source vector.
 */
std::vector<double> GatherPairToRos(const std::vector<double>& left,
    const std::vector<double>& right, const std::vector<PairJointIndex>& joint_map);

//========================================== VALIDATION ============================================

/**
 * @brief [Non-blocking] Check a requested vector before it reaches the RDK, so that a bad value is
 * refused with the joint name and the bound it violated instead of a bare RDK exception. Also
 * rejects NaN and infinity, which a plain range comparison would let through.
 * @param[in] upper Per-joint upper bound, same size as [joint_names].
 * @return False on the first violation, with [message] naming the joint and the bound.
 */
bool ValidateJointValues(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower, const std::vector<double>& upper,
    const std::string& property, std::string& message);

/** @brief [Non-blocking] As above, for a property whose bounds are the same on every joint. */
bool ValidateJointValues(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower, double upper,
    const std::string& property, std::string& message);

//======================================= IMPEDANCE SETTING ========================================

/**
 * @brief The three blocking RDK setters, supplied by the hardware interface, which is the only
 * thing that knows its own joint layout and RDK handle. Each takes ROS-ordered, full-length vectors
 * and throws as the RDK does.
 */
struct JointImpedanceSetters
{
    std::function<void(const std::vector<double>& k_q, const std::vector<double>& z_q)>
        set_joint_impedance;
    std::function<void(const std::vector<double>& max_torques)> set_max_contact_torque;
    std::function<void(const std::vector<double>& inertia_scales)> set_joint_inertia_scale;
};

/** @brief Per-joint upper bounds the RDK validates against, in ROS joint order. */
struct JointImpedanceBounds
{
    std::vector<double> k_q_nom; // [Nm/rad]
    std::vector<double> tau_max; // [Nm]
};

/**
 * @brief Node that owns the impedance properties of the robot's joint motion controller.
 *
 * The properties are held here and initialized to the values the robot boots with, so the held
 * configuration is always complete and always legal.
 *
 * The RDK setters are only accepted in a joint impedance control mode, which the robot is not in
 * while it is IDLE -- where it sits between activation and the first controller start, and after
 * every fault. A request that arrives then is held and delivered by Reapply() on the next control
 * mode switch, so a configuration never has to be re-sent by hand after a recovery.
 *
 * The node is added to the controller manager's executor by the hardware interface, so it needs no
 * thread of its own. Every RDK call it makes is blocking and happens on an executor thread or, for
 * Reapply(), on the controller manager's update thread -- never in read() or write().
 */
class JointImpedanceNode : public rclcpp::Node
{
public:
    using SetJointImpedance = flexiv_msgs::srv::SetJointImpedance;
    using SetMaxContactTorque = flexiv_msgs::srv::SetMaxContactTorque;
    using SetJointInertiaScale = flexiv_msgs::srv::SetJointInertiaScale;

    /**
     * @param[in] robot_sn Serial number of the robot, used as the node namespace so that dual-arm
     * setups do not collide.
     * @param[in] joint_names ROS joint names in URDF order. Every vector the node handles is
     * indexed by this.
     * @param[in] bounds Per-joint upper bounds, in the same order.
     * @param[in] impedance_mode_configured Whether the driver runs with
     * rdk_control_mode:=joint_impedance. When false every request is refused, with an explanation.
     * @param[in] status Status shared with the real-time control loop. Must outlive this node.
     * @param[in] setters The RDK calls to make. Must stay valid for the lifetime of this node.
     */
    JointImpedanceNode(const std::string& robot_sn, std::vector<std::string> joint_names,
        JointImpedanceBounds bounds, bool impedance_mode_configured,
        std::shared_ptr<DriverStatus> status, JointImpedanceSetters setters);

    ~JointImpedanceNode() override;

    /**
     * @brief [Blocking] Re-deliver the held properties to the robot. Called from
     * perform_command_mode_switch() right after SwitchMode(), which is when the setters are
     * accepted again and when the robot has just reset the properties to nominal.
     *
     * Only the properties a request has actually changed are re-sent, and every one of them is:
     * a property nobody has touched is already at the nominal value the mode entry reset it to, so
     * sending it would be a blocking call for nothing. Returns immediately having called nothing
     * when no request has ever been accepted.
     *
     * Deliberately does not consult DriverStatus::control_mode: read() has not run since
     * SwitchMode(), so that field is still stale. The caller knows the mode is right.
     * @return True if there was nothing to re-apply or every setter succeeded. Never throws: the
     * caller is the control loop.
     */
    bool Reapply();

    /**
     * @brief [Non-blocking] Note that the held properties no longer govern the robot, e.g. after a
     * switch to RT_JOINT_TORQUE, so that the published in_effect stays honest.
     */
    void MarkNotInEffect();

private:
    /** @brief Whether the robot is in a control mode that accepts the setters. */
    bool InImpedanceMode() const;

    /**
     * @brief Common precondition check for all three services. Populates [message] on refusal.
     * @param[out] deliverable Whether the request can be delivered to the robot right now, as
     * opposed to being held for the next control mode switch.
     */
    bool CheckPreconditions(std::string& message, bool& deliverable) const;

    /** @brief Build the messages from the held properties. setting_mutex_ must be held. */
    flexiv_msgs::msg::JointImpedance BuildJointImpedanceMessage() const;
    flexiv_msgs::msg::MaxContactTorque BuildMaxContactTorqueMessage() const;
    flexiv_msgs::msg::JointInertiaScale BuildJointInertiaScaleMessage() const;

    /** @brief Publish all three latched topics. Takes setting_mutex_. */
    void PublishAll();

    void HandleSetJointImpedance(const std::shared_ptr<SetJointImpedance::Request> request,
        std::shared_ptr<SetJointImpedance::Response> response);
    void HandleSetMaxContactTorque(const std::shared_ptr<SetMaxContactTorque::Request> request,
        std::shared_ptr<SetMaxContactTorque::Response> response);
    void HandleSetJointInertiaScale(const std::shared_ptr<SetJointInertiaScale::Request> request,
        std::shared_ptr<SetJointInertiaScale::Response> response);

    const std::vector<std::string> joint_names_;
    const JointImpedanceBounds bounds_;
    const bool impedance_mode_configured_;
    std::shared_ptr<DriverStatus> status_;
    JointImpedanceSetters setters_;

    rclcpp::Service<SetJointImpedance>::SharedPtr set_joint_impedance_service_;
    rclcpp::Service<SetMaxContactTorque>::SharedPtr set_max_contact_torque_service_;
    rclcpp::Service<SetJointInertiaScale>::SharedPtr set_joint_inertia_scale_service_;

    rclcpp::Publisher<flexiv_msgs::msg::JointImpedance>::SharedPtr joint_impedance_publisher_;
    rclcpp::Publisher<flexiv_msgs::msg::MaxContactTorque>::SharedPtr max_contact_torque_publisher_;
    rclcpp::Publisher<flexiv_msgs::msg::JointInertiaScale>::SharedPtr
        joint_inertia_scale_publisher_;

    /** One group for all three services, so two blocking RDK calls are never in flight at once. */
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    /**
     * Guards everything below. Held to snapshot and to commit, never across an RDK call, so that a
     * blocking delivery cannot stall the query path or Reapply().
     */
    mutable std::mutex setting_mutex_;
    std::vector<double> k_q_;
    std::vector<double> z_q_;
    std::vector<double> max_contact_torque_;
    std::vector<double> inertia_scale_;
    // Which properties a request has changed, and therefore have to be re-sent after a mode
    // entry. Never cleared: everything the operator has ever set has to survive a controller
    // restart, not just whichever property they set most recently.
    bool customized_joint_impedance_ = false;
    bool customized_max_contact_torque_ = false;
    bool customized_inertia_scale_ = false;
    /** Whether the held properties have been delivered to the robot. */
    bool in_effect_ = false;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__JOINT_IMPEDANCE_NODE_HPP_ */

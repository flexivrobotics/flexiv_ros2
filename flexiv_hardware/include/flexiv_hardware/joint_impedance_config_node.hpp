/**
 * @file joint_impedance_config_node.hpp
 * @brief ROS node hosted by the hardware interface, exposing the impedance properties of the
 * robot's joint motion controller used in the joint impedance control modes.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__JOINT_IMPEDANCE_CONFIG_NODE_HPP_
#define FLEXIV_HARDWARE__JOINT_IMPEDANCE_CONFIG_NODE_HPP_

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

/** @brief Where one ROS joint lives in a robot pair. */
struct PairJointIndex
{
    int robot_index; // 0: Left, 1: Right
    int dof_index;   // Index in that robot's joint vector
};

/**
 * @brief [Non-blocking] Convert a ROS-ordered vector into RDK joint order.
 * @param[in] rdk_to_ros_map Index is the RDK index, value is the ROS index.
 * @throw std::invalid_argument if any mapped index is out of range for [ros_values].
 */
std::vector<double> ConvertROSToRDKOrder(
    const std::vector<double>& ros_values, const std::vector<size_t>& rdk_to_ros_map);

/**
 * @brief [Non-blocking] Convert an RDK-ordered vector into ROS joint order.
 * @return Vector of size rdk_to_ros_map.size(). Entries no RDK index maps to stay 0.
 * @throw std::invalid_argument if any mapped index is out of range for the result.
 */
std::vector<double> ConvertRDKToROSOrder(
    const std::vector<double>& rdk_values, const std::vector<size_t>& rdk_to_ros_map);

/**
 * @brief [Non-blocking] Convert a ROS-ordered vector into the left/right pair DRDK expects.
 * @param[in] fill_left,fill_right Starting values for each robot, one per joint of that robot. A
 * joint that no ROS joint maps to keeps its fill value.
 * @throw std::invalid_argument if [ros_values] and [joint_map] differ in size, or a mapped index is
 * out of range for its fill vector.
 */
std::pair<std::vector<double>, std::vector<double>> ConvertROSToDRDKOrder(
    const std::vector<double>& ros_values, const std::vector<PairJointIndex>& joint_map,
    const std::vector<double>& fill_left, const std::vector<double>& fill_right);

/**
 * @brief [Non-blocking] Convert a left/right DRDK pair into ROS joint order.
 * @throw std::invalid_argument if a mapped index is out of range for its source vector.
 */
std::vector<double> ConvertDRDKToROSOrder(const std::vector<double>& left,
    const std::vector<double>& right, const std::vector<PairJointIndex>& joint_map);

//========================================== VALIDATION ============================================

/**
 * @brief [Non-blocking] Validate joint values against a per-joint upper bound. Rejects a wrong
 * length, a value outside the range, and NaN or infinity.
 * @param[in] upper Per-joint upper bound, same size as [joint_names].
 * @return False on the first violation, with [message] naming the joint and the bound.
 */
bool ValidateJointValues(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower, const std::vector<double>& upper,
    const std::string& property, std::string& message);

/** @brief [Non-blocking] Validate joint values whose bounds are the same on every joint. */
bool ValidateJointValues(const std::vector<double>& values,
    const std::vector<std::string>& joint_names, double lower, double upper,
    const std::string& property, std::string& message);

//======================================= IMPEDANCE SETTING ========================================

/**
 * @brief The three blocking RDK setters, supplied by the hardware interface. Each takes
 * ROS-ordered, full-length vectors and throws as the RDK does.
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
 * @brief Node that configures the impedance properties of the robot's joint motion controller.
 *
 * The RDK setters are only accepted in a joint impedance control mode. The robot is in IDLE between
 * activation and the first controller start, and after every fault; a request that arrives then is
 * held and delivered by Reapply() on the next control mode switch, so a configuration never has to
 * be re-sent by hand after a recovery.
 *
 * The node is added to the controller manager's executor by the hardware interface, so it needs no
 * thread of its own. Every RDK call it makes is blocking and happens on an executor thread or, for
 * Reapply(), on the controller manager's update thread.
 */
class JointImpedanceConfigNode : public rclcpp::Node
{
public:
    using SetJointImpedance = flexiv_msgs::srv::SetJointImpedance;
    using SetMaxContactTorque = flexiv_msgs::srv::SetMaxContactTorque;
    using SetJointInertiaScale = flexiv_msgs::srv::SetJointInertiaScale;

    /**
     * @param[in] robot_sn Serial number of the robot, used as the node namespace.
     * @param[in] joint_names ROS joint names in URDF order. Every vector the node handles is
     * indexed by this.
     * @param[in] bounds Per-joint upper bounds, in the same order.
     * @param[in] impedance_mode_configured Whether the driver runs with
     * rdk_control_mode:=joint_impedance. When false every request is refused.
     * @param[in] status Status shared with the real-time control loop. Must outlive this node.
     * @param[in] setters The RDK calls to make. Must stay valid for the lifetime of this node.
     */
    JointImpedanceConfigNode(const std::string& robot_sn, std::vector<std::string> joint_names,
        JointImpedanceBounds bounds, bool impedance_mode_configured,
        std::shared_ptr<DriverStatus> status, JointImpedanceSetters setters);

    ~JointImpedanceConfigNode() override;

    /**
     * @brief [Blocking] Re-deliver the held properties. Called from
     * perform_command_mode_switch() right after SwitchMode(), which resets them on the robot.
     *
     * Only the properties a request has changed are re-sent; one nobody has touched is already at
     * the nominal value the mode entry reset it to.
     * @return True if there was nothing to re-apply or every setter succeeded. Never throws: the
     * caller is the control loop.
     */
    bool Reapply();

    /**
     * @brief [Non-blocking] Mark the held properties as no longer in effect, e.g. after a switch to
     * RT_JOINT_TORQUE.
     */
    void MarkNotInEffect();

private:
    /** @brief Whether the robot is in a control mode that accepts the setters. */
    bool InImpedanceMode() const;

    /**
     * @brief Check the preconditions shared by all three services.
     * @param[out] deliverable Whether the request can be delivered now, rather than held for the
     * next control mode switch.
     * @return False if the request must be refused, with [message] explaining why.
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

    /** Shared by all three services, so two blocking RDK calls are never in flight at once. */
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    /** Guards everything below. Never held across an RDK call. */
    mutable std::mutex setting_mutex_;
    std::vector<double> k_q_;
    std::vector<double> z_q_;
    std::vector<double> max_contact_torques_;
    std::vector<double> inertia_scales_;
    // Which properties a request has changed, and so have to be re-sent after a mode entry. Never
    // cleared: everything that was ever set has to survive a controller restart.
    bool customized_joint_impedance_ = false;
    bool customized_max_contact_torque_ = false;
    bool customized_inertia_scale_ = false;
    /** Whether the held properties have been delivered to the robot. */
    bool in_effect_ = false;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__JOINT_IMPEDANCE_CONFIG_NODE_HPP_ */

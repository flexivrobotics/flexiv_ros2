/**
 * @file joint_motion_limits.hpp
 * @brief Limits for the robot's non-real-time joint motion generator, shared by the single and dual
 * robot hardware interfaces.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__JOINT_MOTION_LIMITS_HPP_
#define FLEXIV_HARDWARE__JOINT_MOTION_LIMITS_HPP_

#include <vector>

#include <hardware_interface/hardware_info.hpp>

namespace flexiv_hardware {

// Velocity limit for a joint the URDF gives none [rad/s]
constexpr double kDefaultMaxJointVelocity = 2.0;
// Default of the max_joint_acceleration hardware parameter [rad/s^2], the acceleration limit
// flexiv_moveit_config plans with
constexpr double kDefaultMaxJointAcceleration = 5.0;

/**
 * @brief Velocity limit of each joint in ROS order: the URDF limit, or kDefaultMaxJointVelocity
 * for a joint without one.
 */
std::vector<double> JointVelocityLimits(const hardware_interface::HardwareInfo& info);

/**
 * @brief Read the optional max_joint_acceleration hardware parameter, applied to every joint.
 * @throw std::invalid_argument if it is not a positive number.
 */
double MaxJointAcceleration(const hardware_interface::HardwareInfo& info);

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__JOINT_MOTION_LIMITS_HPP_ */

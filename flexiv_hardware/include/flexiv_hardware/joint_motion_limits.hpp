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
// Acceleration limit for a joint the URDF gives none [rad/s^2]
constexpr double kDefaultMaxJointAcceleration = 5.0;

/**
 * @brief Velocity limit of each joint in ROS order: the URDF limit, or kDefaultMaxJointVelocity
 * for a joint without one.
 */
std::vector<double> JointVelocityLimits(const hardware_interface::HardwareInfo& info);

/**
 * @brief Acceleration limit of each joint in ROS order: the lower of the URDF 1.2 acceleration
 * and deceleration limits, or kDefaultMaxJointAcceleration for a joint without one.
 * @throw std::invalid_argument if the robot description cannot be parsed.
 */
std::vector<double> JointAccelerationLimits(const hardware_interface::HardwareInfo& info);

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__JOINT_MOTION_LIMITS_HPP_ */

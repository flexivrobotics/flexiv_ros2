/**
 * @file joint_motion_limits.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "flexiv_hardware/joint_motion_limits.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace flexiv_hardware {

std::vector<double> JointVelocityLimits(const hardware_interface::HardwareInfo& info)
{
    std::vector<double> limits(info.joints.size(), kDefaultMaxJointVelocity);
    for (size_t i = 0; i < info.joints.size(); i++) {
        const auto it = info.limits.find(info.joints[i].name);
        if (it != info.limits.end() && it->second.has_velocity_limits
            && std::isfinite(it->second.max_velocity) && it->second.max_velocity > 0.0) {
            limits[i] = it->second.max_velocity;
        }
    }
    return limits;
}

double MaxJointAcceleration(const hardware_interface::HardwareInfo& info)
{
    const auto it = info.hardware_parameters.find("max_joint_acceleration");
    if (it == info.hardware_parameters.end()) {
        return kDefaultMaxJointAcceleration;
    }
    double value = 0.0;
    try {
        value = std::stod(it->second);
    } catch (const std::exception&) {
        throw std::invalid_argument(
            "Parameter 'max_joint_acceleration' is not a number: '" + it->second + "'");
    }
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
            "Parameter 'max_joint_acceleration' must be positive, got " + it->second);
    }
    return value;
}

} /* namespace flexiv_hardware */

/**
 * @file joint_motion_limits.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "flexiv_hardware/joint_motion_limits.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <urdf/model.h>

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

std::vector<double> JointAccelerationLimits(const hardware_interface::HardwareInfo& info)
{
    // ros2_control's own joint limits carry no acceleration, so read it from the URDF itself
    urdf::Model model;
    if (!model.initString(info.original_xml)) {
        throw std::invalid_argument("Failed to parse the robot description for joint limits");
    }
    std::vector<double> limits(info.joints.size(), kDefaultMaxJointAcceleration);
    for (size_t i = 0; i < info.joints.size(); i++) {
        const auto joint = model.getJoint(info.joints[i].name);
        if (!joint || !joint->limits) {
            continue;
        }
        const double acc = std::min(joint->limits->acceleration, joint->limits->deceleration);
        if (std::isfinite(acc) && acc > 0.0) {
            limits[i] = acc;
        }
    }
    return limits;
}

} /* namespace flexiv_hardware */

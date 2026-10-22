/**
 * @file test_joint_motion_limits.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "flexiv_hardware/joint_motion_limits.hpp"

namespace {

std::string MakeUrdf(const std::string& version, const std::string& joint2_limit)
{
    return R"(<robot name="r" )" + version + R"(>
  <link name="base_link" /><link name="link1" /><link name="link2" />
  <joint name="joint1" type="revolute">
    <parent link="base_link" /><child link="link1" /><axis xyz="0 0 1" />
    <limit lower="-1" upper="1" effort="10" velocity="2" acceleration="8" jerk="64" />
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1" /><child link="link2" /><axis xyz="0 0 1" />
    )" + joint2_limit
           + R"(
  </joint>
</robot>)";
}

hardware_interface::HardwareInfo MakeInfo(const std::string& urdf)
{
    hardware_interface::HardwareInfo info;
    info.original_xml = urdf;
    for (const auto* name : {"joint1", "joint2", "joint_missing"}) {
        hardware_interface::ComponentInfo joint;
        joint.name = name;
        info.joints.push_back(joint);
    }
    return info;
}

} // namespace

TEST(JointAccelerationLimits, ReadsUrdfLimitsAndDeceleration)
{
    const auto limits = flexiv_hardware::JointAccelerationLimits(MakeInfo(
        MakeUrdf(R"(version="1.2")", R"(<limit lower="-1" upper="1" effort="10" velocity="2" )"
                                     R"(acceleration="8" deceleration="6" jerk="64" />)")));
    ASSERT_EQ(limits.size(), 3u);
    EXPECT_DOUBLE_EQ(limits[0], 8.0);
    // The lower of acceleration and deceleration
    EXPECT_DOUBLE_EQ(limits[1], 6.0);
    EXPECT_DOUBLE_EQ(limits[2], flexiv_hardware::kDefaultMaxJointAcceleration);
}

TEST(JointAccelerationLimits, FallsBackBelowUrdfVersion12)
{
    // Older URDF versions leave acceleration at infinity
    const auto limits = flexiv_hardware::JointAccelerationLimits(
        MakeInfo(MakeUrdf("", R"(<limit lower="-1" upper="1" effort="10" velocity="2" />)")));
    for (double acc : limits) {
        EXPECT_DOUBLE_EQ(acc, flexiv_hardware::kDefaultMaxJointAcceleration);
    }
}

TEST(JointAccelerationLimits, ThrowsOnInvalidDescription)
{
    EXPECT_THROW(
        flexiv_hardware::JointAccelerationLimits(MakeInfo("<robot")), std::invalid_argument);
}

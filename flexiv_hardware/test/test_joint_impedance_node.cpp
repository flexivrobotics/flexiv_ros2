/**
 * @file test_joint_impedance_node.cpp
 * @brief Unit tests for the joint order conversions and the range validation used by the joint
 * impedance interface. Needs no robot connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "flexiv_hardware/joint_impedance_node.hpp"

using flexiv_hardware::GatherPairToRos;
using flexiv_hardware::GatherRdkToRos;
using flexiv_hardware::kMaxDampingRatio;
using flexiv_hardware::kMaxInertiaScale;
using flexiv_hardware::kMinDampingRatio;
using flexiv_hardware::kMinInertiaScale;
using flexiv_hardware::PairJointIndex;
using flexiv_hardware::PermuteRosToRdk;
using flexiv_hardware::SplitRosToPair;
using flexiv_hardware::ValidateJointValues;

namespace {

std::vector<std::string> JointNames(size_t count)
{
    std::vector<std::string> names;
    for (size_t i = 1; i <= count; ++i) {
        names.push_back("joint" + std::to_string(i));
    }
    return names;
}

/** @brief The map the hardware interface builds for a 7-DoF arm with no external axis. */
std::vector<size_t> IdentityMap(size_t count)
{
    std::vector<size_t> map(count);
    for (size_t i = 0; i < count; ++i) {
        map[i] = i;
    }
    return map;
}

/** @brief A contiguous left/right map: the first [left] ROS joints are the left robot's. */
std::vector<PairJointIndex> ContiguousPairMap(int left, int right)
{
    std::vector<PairJointIndex> map;
    for (int i = 0; i < left; ++i) {
        map.push_back({0, i});
    }
    for (int i = 0; i < right; ++i) {
        map.push_back({1, i});
    }
    return map;
}

}

//========================================== VALIDATION ============================================

TEST(JointImpedanceValidation, AcceptsValuesWithinPerJointBounds)
{
    // Deliberately non-uniform, so a per-joint bound is genuinely exercised rather than a single
    // maximum that happens to hold for every joint.
    const std::vector<double> bounds {3000.0, 3000.0, 800.0, 800.0, 100.0, 100.0, 100.0};
    std::string message;
    EXPECT_TRUE(ValidateJointValues({3000.0, 1500.0, 800.0, 0.0, 100.0, 50.0, 25.0}, JointNames(7),
        0.0, bounds, "k_q", message))
        << message;
}

TEST(JointImpedanceValidation, RejectsValueAbovePerJointBound)
{
    const std::vector<double> bounds {3000.0, 3000.0, 800.0, 800.0, 100.0, 100.0, 100.0};
    std::string message;
    // Legal for joints 1 and 2, but above the bound for joint 3.
    EXPECT_FALSE(ValidateJointValues(
        {0.0, 0.0, 900.0, 0.0, 0.0, 0.0, 0.0}, JointNames(7), 0.0, bounds, "k_q", message));
    EXPECT_NE(message.find("joint3"), std::string::npos) << message;
    EXPECT_NE(message.find("k_q"), std::string::npos) << message;
}

TEST(JointImpedanceValidation, RejectsNegativeStiffnessButAcceptsZero)
{
    const std::vector<double> bounds(7, 100.0);
    std::string message;
    EXPECT_FALSE(ValidateJointValues(
        {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, JointNames(7), 0.0, bounds, "k_q", message));

    // Zero is legal: the RDK documents it as making the axis free-floating.
    EXPECT_TRUE(ValidateJointValues(
        std::vector<double>(7, 0.0), JointNames(7), 0.0, bounds, "k_q", message))
        << message;
}

TEST(JointImpedanceValidation, RejectsWrongLengthNamingBothSizes)
{
    const std::vector<double> bounds(7, 100.0);
    std::string message;
    EXPECT_FALSE(ValidateJointValues({1.0, 2.0, 3.0}, JointNames(7), 0.0, bounds, "k_q", message));
    EXPECT_NE(message.find('3'), std::string::npos) << message;
    EXPECT_NE(message.find('7'), std::string::npos) << message;
}

TEST(JointImpedanceValidation, RejectsNonFiniteValues)
{
    const std::vector<double> bounds(7, 100.0);
    std::string message;

    auto with_value_at_index_two = [](double value) {
        std::vector<double> values(7, 1.0);
        values[2] = value;
        return values;
    };

    // A NaN passes both halves of a plain range comparison, so it needs its own check.
    EXPECT_FALSE(
        ValidateJointValues(with_value_at_index_two(std::numeric_limits<double>::quiet_NaN()),
            JointNames(7), 0.0, bounds, "k_q", message));
    EXPECT_NE(message.find("joint3"), std::string::npos) << message;

    EXPECT_FALSE(
        ValidateJointValues(with_value_at_index_two(std::numeric_limits<double>::infinity()),
            JointNames(7), 0.0, bounds, "k_q", message));
}

TEST(JointImpedanceValidation, DampingRatioBoundsAreInclusive)
{
    std::string message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMinDampingRatio), JointNames(7),
        kMinDampingRatio, kMaxDampingRatio, "z_q", message))
        << message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMaxDampingRatio), JointNames(7),
        kMinDampingRatio, kMaxDampingRatio, "z_q", message))
        << message;
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 0.29), JointNames(7), kMinDampingRatio,
        kMaxDampingRatio, "z_q", message));
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 0.81), JointNames(7), kMinDampingRatio,
        kMaxDampingRatio, "z_q", message));
}

TEST(JointImpedanceValidation, InertiaScaleBoundsAreInclusive)
{
    std::string message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMinInertiaScale), JointNames(7),
        kMinInertiaScale, kMaxInertiaScale, "inertia_scale", message))
        << message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMaxInertiaScale), JointNames(7),
        kMinInertiaScale, kMaxInertiaScale, "inertia_scale", message))
        << message;
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 0.74), JointNames(7), kMinInertiaScale,
        kMaxInertiaScale, "inertia_scale", message));
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 1.01), JointNames(7), kMinInertiaScale,
        kMaxInertiaScale, "inertia_scale", message));
}

//===================================== SINGLE ROBOT ORDERING ======================================

TEST(JointOrderSingle, IdentityMapPassesValuesThrough)
{
    const std::vector<double> ros {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    EXPECT_EQ(PermuteRosToRdk(ros, IdentityMap(7)), ros);
}

TEST(JointOrderSingle, ExternalAxisComesFirstInRdkOrder)
{
    // The hardware interface puts external axes at the front of the RDK vector, so a URDF that
    // lists the external axis last maps RDK index 0 to ROS index 7.
    const std::vector<size_t> map {7, 0, 1, 2, 3, 4, 5, 6};
    const std::vector<double> ros {10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 99.0};

    const auto rdk = PermuteRosToRdk(ros, map);
    ASSERT_EQ(rdk.size(), 8u);
    EXPECT_DOUBLE_EQ(rdk[0], 99.0);
    EXPECT_DOUBLE_EQ(rdk[1], 10.0);
    EXPECT_DOUBLE_EQ(rdk[7], 16.0);
}

TEST(JointOrderSingle, PermuteAndGatherRoundTrip)
{
    const std::vector<size_t> map {7, 0, 1, 2, 3, 4, 5, 6};
    const std::vector<double> ros {10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 99.0};
    EXPECT_EQ(GatherRdkToRos(PermuteRosToRdk(ros, map), map), ros);
}

TEST(JointOrderSingle, ThrowsWhenTheMapExceedsTheValues)
{
    EXPECT_THROW(PermuteRosToRdk({1.0, 2.0}, IdentityMap(7)), std::invalid_argument);
    EXPECT_THROW(GatherRdkToRos({1.0, 2.0}, IdentityMap(7)), std::invalid_argument);
}

//====================================== ROBOT PAIR ORDERING =======================================

TEST(JointOrderPair, SplitsContiguousLeftRightMap)
{
    const auto map = ContiguousPairMap(7, 7);
    std::vector<double> ros(14);
    for (size_t i = 0; i < ros.size(); ++i) {
        ros[i] = static_cast<double>(i);
    }

    const auto split
        = SplitRosToPair(ros, map, std::vector<double>(7, -1.0), std::vector<double>(7, -1.0));
    ASSERT_EQ(split.first.size(), 7u);
    ASSERT_EQ(split.second.size(), 7u);
    EXPECT_DOUBLE_EQ(split.first[0], 0.0);
    EXPECT_DOUBLE_EQ(split.first[6], 6.0);
    EXPECT_DOUBLE_EQ(split.second[0], 7.0);
    EXPECT_DOUBLE_EQ(split.second[6], 13.0);
}

TEST(JointOrderPair, SplitFollowsTheMapNotThePosition)
{
    // Left and right joints alternating in URDF order: a split that assumed the first half belongs
    // to the left robot would silently swap values between the arms.
    std::vector<PairJointIndex> map;
    for (int i = 0; i < 3; ++i) {
        map.push_back({0, i});
        map.push_back({1, i});
    }
    const std::vector<double> ros {0.0, 100.0, 1.0, 101.0, 2.0, 102.0};

    const auto split
        = SplitRosToPair(ros, map, std::vector<double>(3, -1.0), std::vector<double>(3, -1.0));
    EXPECT_EQ(split.first, std::vector<double>({0.0, 1.0, 2.0}));
    EXPECT_EQ(split.second, std::vector<double>({100.0, 101.0, 102.0}));
}

TEST(JointOrderPair, UnmappedRobotJointsKeepTheirFillValue)
{
    // The AICO2 case: the right robot's two external axes are deliberately left unmapped, so no ROS
    // joint supplies a value for them. They must keep the robot's own nominal value -- a stiffness
    // of 0 there would make those axes free-floating and they would sag.
    std::vector<PairJointIndex> map;
    for (int i = 0; i < 9; ++i) {
        map.push_back({0, i}); // left: 2 external axes + 7 arm joints
    }
    for (int i = 2; i < 9; ++i) {
        map.push_back({1, i}); // right: arm joints only, external axes 0 and 1 unmapped
    }
    ASSERT_EQ(map.size(), 16u);

    const std::vector<double> nominal_left(9, 3000.0);
    const std::vector<double> nominal_right(9, 2500.0);
    const std::vector<double> ros(16, 10.0);

    const auto split = SplitRosToPair(ros, map, nominal_left, nominal_right);
    ASSERT_EQ(split.second.size(), 9u);
    EXPECT_DOUBLE_EQ(split.second[0], 2500.0);
    EXPECT_DOUBLE_EQ(split.second[1], 2500.0);
    EXPECT_DOUBLE_EQ(split.second[2], 10.0);
}

TEST(JointOrderPair, SplitAndGatherRoundTrip)
{
    const auto map = ContiguousPairMap(7, 7);
    std::vector<double> ros(14);
    for (size_t i = 0; i < ros.size(); ++i) {
        ros[i] = static_cast<double>(i) + 0.5;
    }

    const auto split
        = SplitRosToPair(ros, map, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0));
    EXPECT_EQ(GatherPairToRos(split.first, split.second, map), ros);
}

TEST(JointOrderPair, ThrowsOnSizeMismatch)
{
    const auto map = ContiguousPairMap(7, 7);
    EXPECT_THROW(
        SplitRosToPair({1.0, 2.0}, map, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0)),
        std::invalid_argument);

    // A joint mapped past the end of its robot's vector.
    const std::vector<PairJointIndex> bad_map {{0, 9}};
    EXPECT_THROW(
        SplitRosToPair({1.0}, bad_map, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0)),
        std::invalid_argument);
}

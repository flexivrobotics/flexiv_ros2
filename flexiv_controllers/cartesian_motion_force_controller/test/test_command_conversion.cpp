/**
 * @file test_command_conversion.cpp
 * @brief Unit tests for the validation of Cartesian motion-force commands.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "cartesian_motion_force_controller/cartesian_motion_force_controller.hpp"

using cartesian_motion_force_controller::CmdType;
using cartesian_motion_force_controller::Command;
using cartesian_motion_force_controller::ToCommand;

namespace {

CmdType ValidMessage()
{
    CmdType msg;
    msg.pose.position.x = 0.5;
    msg.pose.position.y = -0.1;
    msg.pose.position.z = 0.3;
    msg.pose.orientation.w = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 2.0;
    msg.pose.orientation.z = 0.0;
    msg.wrench.force.z = 5.0;
    msg.velocity.linear.y = 0.1;
    return msg;
}

}

TEST(ToCommand, OrdersFieldsLikeTheInterfacesAndNormalizesTheQuaternion)
{
    Command command {};
    std::string error;
    ASSERT_TRUE(ToCommand(ValidMessage(), command, error)) << error;

    EXPECT_DOUBLE_EQ(command[0], 0.5);
    EXPECT_DOUBLE_EQ(command[3], 0.0);  // qw
    EXPECT_DOUBLE_EQ(command[5], 1.0);  // qy, normalized from 2.0
    EXPECT_DOUBLE_EQ(command[9], 5.0);  // fz
    EXPECT_DOUBLE_EQ(command[14], 0.1); // vy
}

TEST(ToCommand, RejectsInvalidMessagesAndKeepsTheLastCommand)
{
    Command command {};
    command.fill(7.0);
    std::string error;

    // A NaN quaternion must not slip through the normalization.
    auto msg = ValidMessage();
    msg.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(ToCommand(msg, command, error));

    msg = ValidMessage();
    msg.wrench.force.z = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(ToCommand(msg, command, error));

    // No fallback to identity for a zero quaternion.
    msg = ValidMessage();
    msg.pose.orientation.y = 0.0;
    EXPECT_FALSE(ToCommand(msg, command, error));

    for (double value : command) {
        EXPECT_DOUBLE_EQ(value, 7.0);
    }
}

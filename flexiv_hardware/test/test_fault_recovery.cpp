/**
 * @file test_operational_status.cpp
 * @brief Unit tests for the recovery policy classification. Needs no robot connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <gtest/gtest.h>

#include "flexiv_hardware/fault_recovery.hpp"

using flexiv::rdk::OperationalStatus;
using flexiv_hardware::ClassifyRecoveryPolicy;
using flexiv_hardware::DescribeRobotCondition;
using flexiv_hardware::OperationalStatusName;
using flexiv_hardware::RecoveryPolicy;
using flexiv_hardware::RecoveryPolicyName;
using flexiv_hardware::RobotCondition;

namespace {

RobotCondition Connected(OperationalStatus status)
{
    RobotCondition condition;
    condition.connected = true;
    condition.operational_status = status;
    return condition;
}

}

TEST(OperationalStatus, DisconnectedOutranksEveryOtherStatus)
{
    RobotCondition condition = Connected(OperationalStatus::READY);
    condition.connected = false;
    EXPECT_EQ(ClassifyRecoveryPolicy(condition), RecoveryPolicy::CONNECTION_LOST);
}

TEST(OperationalStatus, ReadyRobotNeedsNoRecovery)
{
    EXPECT_EQ(ClassifyRecoveryPolicy(Connected(OperationalStatus::READY)), RecoveryPolicy::NONE);
}

TEST(OperationalStatus, TimelinessFailureOnAReadyRobotIsRecoverable)
{
    RobotCondition condition = Connected(OperationalStatus::READY);
    condition.reached_timeliness_failure_limit = true;
    EXPECT_EQ(ClassifyRecoveryPolicy(condition), RecoveryPolicy::AUTO_RECOVERABLE);
    EXPECT_NE(DescribeRobotCondition(condition).find("1 kHz"), std::string::npos);
}

TEST(OperationalStatus, FaultsAreAutoRecoverable)
{
    EXPECT_EQ(ClassifyRecoveryPolicy(Connected(OperationalStatus::MINOR_FAULT)),
        RecoveryPolicy::AUTO_RECOVERABLE);
    EXPECT_EQ(ClassifyRecoveryPolicy(Connected(OperationalStatus::CRITICAL_FAULT)),
        RecoveryPolicy::AUTO_RECOVERABLE);
    EXPECT_EQ(ClassifyRecoveryPolicy(Connected(OperationalStatus::NOT_ENABLED)),
        RecoveryPolicy::AUTO_RECOVERABLE);
}

TEST(OperationalStatus, CriticalFaultWarnsAboutTheDedicatedDevice)
{
    const auto message = DescribeRobotCondition(Connected(OperationalStatus::CRITICAL_FAULT));
    EXPECT_NE(message.find("30 seconds"), std::string::npos);
    EXPECT_NE(message.find("dedicated device"), std::string::npos);
}

TEST(OperationalStatus, TransientStatesJustWait)
{
    EXPECT_EQ(
        ClassifyRecoveryPolicy(Connected(OperationalStatus::BOOTING)), RecoveryPolicy::TRANSIENT);
    EXPECT_EQ(ClassifyRecoveryPolicy(Connected(OperationalStatus::RELEASING_BRAKE)),
        RecoveryPolicy::TRANSIENT);
}

TEST(OperationalStatus, EstopIsASafetyLockout)
{
    EXPECT_EQ(ClassifyRecoveryPolicy(Connected(OperationalStatus::ESTOP_NOT_RELEASED)),
        RecoveryPolicy::SAFETY_LOCKOUT);
    EXPECT_NE(
        DescribeRobotCondition(Connected(OperationalStatus::ESTOP_NOT_RELEASED)).find("E-stop"),
        std::string::npos);
}

TEST(OperationalStatus, OperatorGatedStates)
{
    for (const auto status : {OperationalStatus::IN_REDUCED_STATE,
             OperationalStatus::IN_RECOVERY_STATE, OperationalStatus::IN_MANUAL_MODE,
             OperationalStatus::IN_AUTO_MODE, OperationalStatus::UNKNOWN}) {
        EXPECT_EQ(ClassifyRecoveryPolicy(Connected(status)), RecoveryPolicy::WAIT_OPERATOR)
            << "status " << OperationalStatusName(status);
    }
}

TEST(OperationalStatus, RecoveryStateExplainsTheOptIn)
{
    const auto message = DescribeRobotCondition(Connected(OperationalStatus::IN_RECOVERY_STATE));
    EXPECT_NE(message.find("run_auto_recovery"), std::string::npos);
    EXPECT_NE(message.find("reboot"), std::string::npos);
}

TEST(OperationalStatus, EveryStatusIsClassifiedAndDescribed)
{
    for (uint8_t i = 0; i <= static_cast<uint8_t>(OperationalStatus::IN_AUTO_MODE); ++i) {
        const auto status = static_cast<OperationalStatus>(i);
        const auto condition = Connected(status);
        // Must not throw, and must always produce something the operator can act on.
        EXPECT_FALSE(DescribeRobotCondition(condition).empty());
        EXPECT_FALSE(OperationalStatusName(status).empty());
        EXPECT_NE(RecoveryPolicyName(ClassifyRecoveryPolicy(condition)), "UNKNOWN");
    }
}

TEST(OperationalStatus, NamesComeFromTheRdkStrings)
{
    EXPECT_EQ(OperationalStatusName(OperationalStatus::READY), "Ready");
    EXPECT_EQ(OperationalStatusName(OperationalStatus::MINOR_FAULT), "Minor fault occurred");
    // Out-of-range values must not read past the RDK name table.
    EXPECT_EQ(OperationalStatusName(static_cast<OperationalStatus>(200)), "Unknown status");
}

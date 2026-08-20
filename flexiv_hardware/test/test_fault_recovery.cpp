/**
 * @file test_fault_recovery.cpp
 * @brief Unit tests for the recovery policy classification and the derived driver state. Needs no
 * robot connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "flexiv_hardware/fault_recovery.hpp"

using flexiv::rdk::OperationalStatus;
using flexiv_hardware::ClassifyRecoveryPolicy;
using flexiv_hardware::ControlModeName;
using flexiv_hardware::DescribeRobotCondition;
using flexiv_hardware::DriverState;
using flexiv_hardware::DriverStatus;
using flexiv_hardware::MaxJointDeviation;
using flexiv_hardware::OperationalStatusName;
using flexiv_hardware::RecoveryPolicy;
using flexiv_hardware::RecoveryPolicyName;
using flexiv_hardware::RecoveryStateMachine;
using flexiv_hardware::RobotCondition;
using flexiv_hardware::RobotSystemControl;

namespace {

RobotCondition Connected(OperationalStatus status)
{
    RobotCondition condition;
    condition.connected = true;
    condition.operational_status = status;
    return condition;
}

/**
 * @brief A DriverStatus with the condition fields set as read() would have latched them. Latch()
 * itself needs a robot, so the fields are stored directly here.
 */
void SetLatchedCondition(
    DriverStatus& status, bool connected, bool operational, OperationalStatus operational_status)
{
    status.connected.store(connected);
    status.operational.store(operational);
    status.operational_status.store(operational_status);
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

TEST(OperationalStatus, CriticalFaultPointsAtAPowerCycle)
{
    const auto message = DescribeRobotCondition(Connected(OperationalStatus::CRITICAL_FAULT));
    EXPECT_NE(message.find("power cycle"), std::string::npos);
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

TEST(DriverStateDerivation, StartsUninitialized)
{
    DriverStatus status;
    EXPECT_EQ(status.driver_state.load(), DriverState::UNINITIALIZED);
}

TEST(DriverStateDerivation, DisconnectedOutranksEveryOtherCondition)
{
    DriverStatus status;
    SetLatchedCondition(status, false, true, OperationalStatus::READY);
    EXPECT_EQ(status.DeriveDriverState(), DriverState::DISCONNECTED);
}

TEST(DriverStateDerivation, OperationalRobotIsReady)
{
    DriverStatus status;
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    EXPECT_EQ(status.DeriveDriverState(), DriverState::READY);
}

TEST(DriverStateDerivation, EstopIsALockoutNotAPlainFault)
{
    DriverStatus status;
    SetLatchedCondition(status, true, false, OperationalStatus::ESTOP_NOT_RELEASED);
    EXPECT_EQ(status.DeriveDriverState(), DriverState::LOCKOUT);
}

TEST(DriverStateDerivation, EveryOtherNonOperationalConditionIsAFault)
{
    for (const auto operational_status :
        {OperationalStatus::NOT_ENABLED, OperationalStatus::MINOR_FAULT,
            OperationalStatus::CRITICAL_FAULT, OperationalStatus::IN_REDUCED_STATE,
            OperationalStatus::IN_RECOVERY_STATE, OperationalStatus::IN_MANUAL_MODE,
            OperationalStatus::IN_AUTO_MODE, OperationalStatus::BOOTING,
            OperationalStatus::RELEASING_BRAKE, OperationalStatus::UNKNOWN}) {
        DriverStatus status;
        SetLatchedCondition(status, true, false, operational_status);
        EXPECT_EQ(status.DeriveDriverState(), DriverState::FAULT)
            << "status " << OperationalStatusName(operational_status);
    }
}

TEST(DriverStateDerivation, ApplyingTheDerivedStateStoresIt)
{
    DriverStatus status;
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    EXPECT_TRUE(status.TryApplyDerivedDriverState());
    EXPECT_EQ(status.driver_state.load(), DriverState::READY);
}

TEST(DriverStateDerivation, ApplyingTheDerivedStateNeverOverridesRecovery)
{
    DriverStatus status;
    // A recovery sequence holds the state, while the robot itself already reads operational: the
    // hold must survive, or write() would resume streaming mid-sequence.
    status.driver_state.store(DriverState::RECOVERING);
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    EXPECT_FALSE(status.TryApplyDerivedDriverState());
    EXPECT_EQ(status.driver_state.load(), DriverState::RECOVERING);
}

namespace {

/**
 * @brief RobotSystemControl backed by plain fields instead of a robot, so the recovery sequence can
 * be driven without hardware. Records which system control calls it received.
 */
class FakeRobot : public RobotSystemControl
{
public:
    bool connected_ = true;
    bool fault_ = false;
    bool operational_ = false;
    bool estop_released_ = true;
    bool in_recovery_ = false;
    bool has_external_axes_ = false;
    bool timeliness_limit_reached_ = false;
    OperationalStatus status_ = OperationalStatus::NOT_ENABLED;
    flexiv::rdk::Mode mode_ = flexiv::rdk::Mode::IDLE;

    /** Stop() throws the way the RDK does when the robot is not operational. */
    bool stop_throws_ = false;
    bool clear_fault_succeeds_ = true;

    int stop_calls = 0;
    int clear_fault_calls = 0;
    int enable_calls = 0;
    int auto_recovery_calls = 0;

    bool connected() const override { return connected_; }
    bool fault() const override { return fault_; }
    bool operational() const override { return operational_; }
    bool estop_released() const override { return estop_released_; }
    bool recovery() const override { return in_recovery_; }
    bool reduced() const override { return false; }
    bool reached_timeliness_failure_limit() const override { return timeliness_limit_reached_; }
    OperationalStatus operational_status() const override { return status_; }
    flexiv::rdk::Mode mode() const override { return mode_; }
    std::vector<flexiv::rdk::RobotEvent> event_log() const override { return {}; }
    bool has_external_axes() const override { return has_external_axes_; }

    void Stop() override
    {
        ++stop_calls;
        if (stop_throws_) {
            throw std::runtime_error("Robot is not operational: Not enabled");
        }
        mode_ = flexiv::rdk::Mode::IDLE;
    }

    bool ClearFault() override
    {
        ++clear_fault_calls;
        if (!clear_fault_succeeds_) {
            return false;
        }
        fault_ = false;
        return true;
    }

    void Enable() override
    {
        ++enable_calls;
        operational_ = true;
        status_ = OperationalStatus::READY;
    }

    void RunAutoRecovery() override { ++auto_recovery_calls; }
    void UnlockExternalAxes() override { }
};

/**
 * @brief Drive the sequence to COMPLETE or FAILED, bounded so a stuck state cannot hang the test.
 */
void RunToCompletion(RecoveryStateMachine& machine)
{
    for (int step = 0; step < 100 && machine.Step(); ++step) {
    }
}

}

TEST(RecoverySequence, DoesNotStopARobotThatIsNotOperational)
{
    // An E-stop that was just released leaves the robot NOT_ENABLED. Stop() switches the control
    // mode internally and the robot rejects that until it is enabled, so the sequence must go
    // straight to enabling it.
    FakeRobot robot;
    robot.stop_throws_ = true;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.stop_calls, 0);
    EXPECT_EQ(robot.enable_calls, 1);
    EXPECT_EQ(machine.policy(), RecoveryPolicy::AUTO_RECOVERABLE);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
}

TEST(RecoverySequence, StopsARobotThatIsStillOperational)
{
    FakeRobot robot;
    robot.operational_ = true;
    robot.status_ = OperationalStatus::READY;
    robot.timeliness_limit_reached_ = true;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.stop_calls, 1);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
}

TEST(RecoverySequence, AFailedStopDoesNotAbortTheSequence)
{
    FakeRobot robot;
    robot.operational_ = true;
    robot.status_ = OperationalStatus::READY;
    robot.timeliness_limit_reached_ = true;
    robot.stop_throws_ = true;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.stop_calls, 1);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
}

TEST(RecoverySequence, MinorFaultIsClearedAndThenEnabled)
{
    FakeRobot robot;
    robot.fault_ = true;
    robot.status_ = OperationalStatus::MINOR_FAULT;
    robot.stop_throws_ = true;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.stop_calls, 0);
    EXPECT_EQ(robot.clear_fault_calls, 1);
    EXPECT_EQ(robot.enable_calls, 1);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
}

TEST(RecoverySequence, ReportsTheControlModeTheRobotIsActuallyIn)
{
    // A minor fault leaves the robot non-operational, so the STOP step never calls Stop() and the
    // pre-fault control mode survives the recovery. The message must say so rather than claim IDLE.
    FakeRobot robot;
    robot.fault_ = true;
    robot.status_ = OperationalStatus::MINOR_FAULT;
    robot.mode_ = flexiv::rdk::Mode::NRT_JOINT_POSITION;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.stop_calls, 0);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
    EXPECT_NE(machine.message().find("NRT_JOINT_POSITION"), std::string::npos) << machine.message();
    EXPECT_EQ(machine.message().find("IDLE"), std::string::npos) << machine.message();
}

TEST(RecoverySequence, ReportsIdleWhenTheSequenceStoppedTheRobot)
{
    // An operational robot that reached the timeliness failure limit is stopped by the STOP step,
    // which does transit it to IDLE.
    FakeRobot robot;
    robot.operational_ = true;
    robot.status_ = OperationalStatus::READY;
    robot.timeliness_limit_reached_ = true;
    robot.mode_ = flexiv::rdk::Mode::RT_JOINT_POSITION;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.stop_calls, 1);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
    EXPECT_NE(machine.message().find("IDLE"), std::string::npos) << machine.message();
}

TEST(ControlMode, NamesComeFromTheRdkStrings)
{
    EXPECT_EQ(ControlModeName(flexiv::rdk::Mode::IDLE), "IDLE");
    EXPECT_EQ(ControlModeName(flexiv::rdk::Mode::NRT_JOINT_POSITION), "NRT_JOINT_POSITION");
    EXPECT_EQ(ControlModeName(flexiv::rdk::Mode::UNKNOWN), "UNKNOWN");
}

TEST(ControlMode, AnOutOfRangeModeReportsUnknown)
{
    EXPECT_EQ(ControlModeName(static_cast<flexiv::rdk::Mode>(255)), "UNKNOWN");
}

TEST(RecoverySequence, AFaultThatCannotBeClearedFails)
{
    FakeRobot robot;
    robot.fault_ = true;
    robot.status_ = OperationalStatus::CRITICAL_FAULT;
    robot.clear_fault_succeeds_ = false;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(robot.enable_calls, 0);
    EXPECT_FALSE(machine.succeeded());
    EXPECT_NE(machine.message().find("Power cycle"), std::string::npos) << machine.message();
}

TEST(RecoverySequence, APressedEstopIsRefusedWithoutTouchingTheRobot)
{
    FakeRobot robot;
    robot.status_ = OperationalStatus::ESTOP_NOT_RELEASED;
    robot.estop_released_ = false;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(machine.policy(), RecoveryPolicy::SAFETY_LOCKOUT);
    EXPECT_FALSE(machine.succeeded());
    EXPECT_EQ(robot.stop_calls, 0);
    EXPECT_EQ(robot.clear_fault_calls, 0);
    EXPECT_EQ(robot.enable_calls, 0);
}

TEST(RecoverySequence, RecoveryStateNeedsTheAutoRecoveryOptIn)
{
    FakeRobot robot;
    robot.status_ = OperationalStatus::IN_RECOVERY_STATE;
    robot.in_recovery_ = true;

    RecoveryStateMachine refused(robot, false);
    RunToCompletion(refused);
    EXPECT_FALSE(refused.succeeded());
    EXPECT_EQ(robot.auto_recovery_calls, 0);

    RecoveryStateMachine opted_in(robot, true);
    RunToCompletion(opted_in);
    EXPECT_EQ(robot.auto_recovery_calls, 1);
    EXPECT_EQ(robot.enable_calls, 0);
    EXPECT_TRUE(opted_in.succeeded()) << opted_in.message();
    EXPECT_NE(opted_in.message().find("Reboot"), std::string::npos);
}

TEST(RecoverySequence, AHealthyRobotIsLeftStrictlyAlone)
{
    // Recovery sent to a robot that is fine must not stop it. It may well be executing a
    // trajectory, and stopping it would also force a controller restart to resume.
    FakeRobot robot;
    robot.operational_ = true;
    robot.status_ = OperationalStatus::READY;

    RecoveryStateMachine machine(robot, false);
    RunToCompletion(machine);

    EXPECT_EQ(machine.policy(), RecoveryPolicy::NONE);
    EXPECT_TRUE(machine.succeeded()) << machine.message();
    EXPECT_EQ(robot.stop_calls, 0);
    EXPECT_EQ(robot.clear_fault_calls, 0);
    EXPECT_EQ(robot.enable_calls, 0);
    EXPECT_EQ(robot.auto_recovery_calls, 0);
}

TEST(CommandSynchronization, StartsUnsynchronized)
{
    // Nothing may be streamed until a controller start has established the control mode.
    DriverStatus status;
    EXPECT_FALSE(status.commands_synchronized.load());
}

TEST(CommandSynchronization, LeavingReadyInvalidatesTheSetpoint)
{
    DriverStatus status;
    status.commands_synchronized.store(true);

    // The robot stayed connected and reports no fault, it was only switched to Manual mode -- the
    // case where an operator can hand-guide it away from the commanded position.
    SetLatchedCondition(status, true, false, OperationalStatus::IN_MANUAL_MODE);
    EXPECT_TRUE(status.TryApplyDerivedDriverState());
    EXPECT_FALSE(status.commands_synchronized.load());

    // Returning to Auto (Remote) makes the robot ready again, but the setpoint stays invalidated
    // until a controller restart re-synchronizes it.
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    EXPECT_TRUE(status.TryApplyDerivedDriverState());
    EXPECT_EQ(status.driver_state.load(), DriverState::READY);
    EXPECT_FALSE(status.commands_synchronized.load());
}

TEST(CommandSynchronization, StayingReadyKeepsTheSetpoint)
{
    DriverStatus status;
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    status.commands_synchronized.store(true);

    for (int cycle = 0; cycle < 5; ++cycle) {
        EXPECT_TRUE(status.TryApplyDerivedDriverState());
    }
    EXPECT_TRUE(status.commands_synchronized.load());
}

TEST(JointDeviation, IdenticalPositionsDeviateNotAtAll)
{
    EXPECT_DOUBLE_EQ(MaxJointDeviation({0.1, -0.2, 0.3}, {0.1, -0.2, 0.3}), 0.0);
}

TEST(JointDeviation, ReportsTheLargestAbsoluteChange)
{
    // The middle joint moved furthest, and backwards: the sign must not hide it.
    EXPECT_DOUBLE_EQ(MaxJointDeviation({0.0, 0.5, 0.0}, {0.1, -0.3, 0.05}), 0.8);
}

TEST(JointDeviation, MismatchedSizesReportNothing)
{
    // A buffer that was never populated must not be read as a deviation.
    EXPECT_DOUBLE_EQ(MaxJointDeviation({}, {0.1, 0.2}), 0.0);
    EXPECT_DOUBLE_EQ(MaxJointDeviation({0.1, 0.2, 0.3}, {0.1, 0.2}), 0.0);
}

TEST(JointDeviation, NanEntriesAreIgnored)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(MaxJointDeviation({nan, 0.5}, {0.1, 0.9}), 0.4);
    EXPECT_DOUBLE_EQ(MaxJointDeviation({nan, nan}, {nan, nan}), 0.0);
}

TEST(CommandSynchronization, ARestartIsRequiredWhileTheRobotIsReadyButUnsynchronized)
{
    DriverStatus status;
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    status.driver_state.store(DriverState::READY);

    status.commands_synchronized.store(false);
    EXPECT_TRUE(status.RequiresControllerRestart());

    status.commands_synchronized.store(true);
    EXPECT_FALSE(status.RequiresControllerRestart());
}

TEST(CommandSynchronization, NoRestartIsReportedWhileTheRobotIsNotReady)
{
    // A restart cannot help a robot that is still faulted, so it must not be asked for yet.
    DriverStatus status;
    status.commands_synchronized.store(false);

    for (const auto state : {DriverState::UNINITIALIZED, DriverState::FAULT,
             DriverState::RECOVERING, DriverState::LOCKOUT, DriverState::DISCONNECTED}) {
        status.driver_state.store(state);
        EXPECT_FALSE(status.RequiresControllerRestart());
    }
}

TEST(CommandSynchronization, AManualModeExcursionLeavesARestartOutstanding)
{
    // The exact sequence an operator produces: Auto (Remote) -> Manual -> Auto (Remote), with no
    // recovery action involved. The restart must still be reported as required afterwards.
    DriverStatus status;
    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    status.TryApplyDerivedDriverState();
    status.commands_synchronized.store(true);
    EXPECT_FALSE(status.RequiresControllerRestart());

    SetLatchedCondition(status, true, false, OperationalStatus::IN_MANUAL_MODE);
    status.TryApplyDerivedDriverState();

    SetLatchedCondition(status, true, true, OperationalStatus::READY);
    status.TryApplyDerivedDriverState();
    EXPECT_EQ(status.driver_state.load(), DriverState::READY);
    EXPECT_TRUE(status.RequiresControllerRestart());
}

TEST(RecoverySequence, ClassificationTouchesNothing)
{
    // The recovery node relies on this: it withholds the driver state until after the first step,
    // so that a robot streaming a trajectory is not interrupted by a recovery that turns out to
    // have nothing to do. The first step must therefore issue no system control call.
    FakeRobot robot;
    robot.fault_ = true;
    robot.status_ = OperationalStatus::MINOR_FAULT;

    RecoveryStateMachine machine(robot, false);
    EXPECT_TRUE(machine.Step());

    EXPECT_EQ(robot.stop_calls, 0);
    EXPECT_EQ(robot.clear_fault_calls, 0);
    EXPECT_EQ(robot.enable_calls, 0);
    EXPECT_EQ(robot.auto_recovery_calls, 0);
}

TEST(RecoverySequence, AHealthyRobotFinishesInTheClassificationStep)
{
    // Finishing in one step is what keeps the driver hold from ever being claimed.
    FakeRobot robot;
    robot.operational_ = true;
    robot.status_ = OperationalStatus::READY;

    RecoveryStateMachine machine(robot, false);
    EXPECT_FALSE(machine.Step());
    EXPECT_TRUE(machine.succeeded()) << machine.message();
}

TEST(RecoverySequence, ARefusedConditionFinishesInTheClassificationStep)
{
    FakeRobot robot;
    robot.status_ = OperationalStatus::ESTOP_NOT_RELEASED;
    robot.estop_released_ = false;

    RecoveryStateMachine machine(robot, false);
    EXPECT_FALSE(machine.Step());
    EXPECT_FALSE(machine.succeeded());
}

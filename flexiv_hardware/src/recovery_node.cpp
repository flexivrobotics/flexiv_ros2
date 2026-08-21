/**
 * @file recovery_node.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <algorithm>
#include <chrono>
#include <thread>

#include "flexiv_hardware/recovery_node.hpp"
#include "flexiv_hardware/fault_recovery.hpp"

namespace {

constexpr int kStatusPublishRate = 10; // [Hz]
constexpr int kRecoveryStepPeriodMs = 100;
constexpr size_t kMaxRecentEvents = 10;

/**
 * @brief Sentence appended when the controllers still have to be restarted before the robot moves.
 * Deliberately short: the default `ros2 topic echo` truncates this field at 128 characters.
 */
std::string DescribeControllerRestart(bool required)
{
    if (!required) {
        return {};
    }
    return " Restart the controllers to resume motion.";
}

}

namespace flexiv_hardware {

RecoveryNode::RecoveryNode(
    const std::string& robot_sn, RobotSystemControl& robot, std::shared_ptr<DriverStatus> status)
: rclcpp::Node("flexiv_recovery_node", SanitizeNamespace(robot_sn))
, robot_(robot)
, status_(std::move(status))
{
    // The recovery sequence can block for up to 30 seconds clearing a critical fault. Keeping it
    // in its own callback group lets the status publisher and the query service keep running.
    recovery_callback_group_
        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_callback_group_
        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    error_recovery_action_server_ = rclcpp_action::create_server<ErrorRecovery>(
        this, "~/error_recovery",
        [this](
            const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ErrorRecovery::Goal> goal) {
            return this->HandleGoal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleErrorRecovery> goal_handle) {
            return this->HandleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleErrorRecovery> goal_handle) {
            this->HandleAccepted(goal_handle);
        },
        rcl_action_server_get_default_options(), recovery_callback_group_);

    get_operational_status_service_ = this->create_service<GetOperationalStatus>(
        "~/get_operational_status",
        [this](const std::shared_ptr<GetOperationalStatus::Request> /*request*/,
            std::shared_ptr<GetOperationalStatus::Response> response) {
            response->status = this->BuildStatusMessage();
        },
        rclcpp::ServicesQoS(), status_callback_group_);

    // Latched so that a late subscriber immediately sees why the robot is not moving.
    operational_status_publisher_ = this->create_publisher<flexiv_msgs::msg::OperationalStatus>(
        "~/operational_status", rclcpp::QoS(1).reliable().transient_local());

    status_publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / kStatusPublishRate),
        [this]() { this->PublishStatus(); }, status_callback_group_);

    RCLCPP_INFO(this->get_logger(),
        "Fault recovery interface ready: action '%s/error_recovery', service "
        "'%s/get_operational_status', topic '%s/operational_status'",
        this->get_fully_qualified_name(), this->get_fully_qualified_name(),
        this->get_fully_qualified_name());
}

RecoveryNode::~RecoveryNode() = default;

void RecoveryNode::RefreshRecentEvents()
{
    std::vector<flexiv_msgs::msg::RobotEvent> events;
    try {
        const auto event_log = robot_.event_log();
        // Only the tail matters, and only the entries that explain a fault.
        for (auto it = event_log.rbegin();
             it != event_log.rend() && events.size() < kMaxRecentEvents; ++it) {
            if (it->level != flexiv::rdk::RobotEvent::ERROR
                && it->level != flexiv::rdk::RobotEvent::CRITICAL) {
                continue;
            }
            flexiv_msgs::msg::RobotEvent event;
            event.level = static_cast<uint8_t>(it->level);
            event.id = it->id;
            event.description = it->description;
            event.consequences = it->consequences;
            event.probable_causes = it->probable_causes;
            event.recommended_actions = it->recommended_actions;
            const auto since_epoch = it->timestamp.time_since_epoch();
            const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
            event.timestamp.sec = static_cast<int32_t>(seconds.count());
            event.timestamp.nanosec = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - seconds)
                    .count());
            events.push_back(std::move(event));
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not read the robot event log: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(recent_events_mutex_);
    recent_events_ = std::move(events);
}

flexiv_msgs::msg::OperationalStatus RecoveryNode::BuildStatusMessage()
{
    flexiv_msgs::msg::OperationalStatus message;
    message.header.stamp = this->now();

    const auto condition = status_->condition();

    message.operational_status = static_cast<uint8_t>(condition.operational_status);
    message.operational_status_name = OperationalStatusName(condition.operational_status);
    message.driver_state = static_cast<uint8_t>(status_->driver_state.load());
    message.connected = condition.connected;
    message.fault = status_->fault.load();
    message.operational = status_->operational.load();
    message.estop_released = status_->estop_released.load();
    message.reduced = status_->reduced.load();
    message.recovery_state = status_->recovery_state.load();
    message.control_mode = static_cast<int8_t>(status_->control_mode.load());
    message.recovery_policy = static_cast<uint8_t>(ClassifyRecoveryPolicy(condition));
    message.message = DescribeRobotCondition(condition)
                      + DescribeControllerRestart(status_->RequiresControllerRestart());

    std::lock_guard<std::mutex> lock(recent_events_mutex_);
    message.recent_events = recent_events_;

    return message;
}

void RecoveryNode::PublishStatus()
{
    // Reading the event log every cycle would copy the whole log at 10 Hz, so only refresh it when
    // a new fault appears.
    const bool fault = status_->fault.load();
    if (fault && !previous_fault_) {
        RefreshRecentEvents();
    }
    previous_fault_ = fault;

    operational_status_publisher_->publish(BuildStatusMessage());
}

rclcpp_action::GoalResponse RecoveryNode::HandleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const ErrorRecovery::Goal> /*goal*/)
{
    if (recovery_in_progress_.load()) {
        RCLCPP_WARN(this->get_logger(), "Recovery is already in progress, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecoveryNode::HandleCancel(
    const std::shared_ptr<GoalHandleErrorRecovery>& /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel recovery");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RecoveryNode::HandleAccepted(const std::shared_ptr<GoalHandleErrorRecovery>& goal_handle)
{
    // Recovery blocks for seconds at a time, so it must not run on an executor thread.
    std::thread {[this, goal_handle]() { this->ExecuteRecovery(goal_handle); }}.detach();
}

void RecoveryNode::ExecuteRecovery(const std::shared_ptr<GoalHandleErrorRecovery>& goal_handle)
{
    recovery_in_progress_.store(true);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ErrorRecovery::Feedback>();
    auto result = std::make_shared<ErrorRecovery::Result>();

    RCLCPP_INFO(this->get_logger(), "Starting recovery sequence");
    RecoveryStateMachine state_machine(robot_, goal->run_auto_recovery);

    bool canceled = false;
    bool hold_claimed = false;

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            canceled = true;
            break;
        }

        const bool running = state_machine.Step();

        feedback->recovery_state = static_cast<uint8_t>(state_machine.state());
        feedback->recovery_state_name = RecoveryStateName(state_machine.state());
        feedback->elapsed_seconds = static_cast<float>(state_machine.elapsed_seconds());
        goal_handle->publish_feedback(feedback);

        if (!running) {
            break;
        }

        if (!hold_claimed) {
            status_->driver_state.store(DriverState::RECOVERING);
            hold_claimed = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kRecoveryStepPeriodMs));
    }

    if (hold_claimed) {
        // The sequence issued system control calls, so it may have stopped the robot and dropped it
        // to IDLE -- including one that was canceled or failed part way through. Withhold motion
        // until a controller restart re-synchronizes the command buffers.
        status_->commands_synchronized.store(false);

        // Release the claim. The driver state is re-derived from the robot rather than guessed per
        // outcome, so it reports what the robot actually is now regardless of how recovery ended.
        // Latching here also means write() is unblocked without waiting for the next read().
        status_->Latch(robot_);
        status_->driver_state.store(status_->DeriveDriverState());
    }

    // Reported only once the hold above is released, so that the gate reflects the robot rather
    // than the recovery that just finished. The robot is left in IDLE control mode on purpose:
    // re-entering a control mode goes through a controller restart, so that the controller
    // re-initializes its own setpoint and cannot apply a stale pre-fault command.
    const bool restart_required = status_->RequiresControllerRestart();

    result->recovery_policy = static_cast<uint8_t>(state_machine.policy());
    result->success = state_machine.succeeded();
    result->message = state_machine.message() + DescribeControllerRestart(restart_required);
    result->requires_controller_restart = restart_required;

    if (canceled) {
        result->success = false;
        result->message
            = "Recovery canceled by the caller." + DescribeControllerRestart(restart_required);
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "Recovery canceled");
    } else if (state_machine.succeeded()) {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Recovery failed [%s]: %s",
            RecoveryPolicyName(state_machine.policy()).c_str(), result->message.c_str());
    }

    RefreshRecentEvents();
    recovery_in_progress_.store(false);
}

} /* namespace flexiv_hardware */

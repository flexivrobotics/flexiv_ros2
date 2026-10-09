#include <algorithm>
#include <cmath>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

#include "flexiv_gripper/gripper_action_server.hpp"

namespace {

constexpr int kDefaultStatePublishRate = 30;     // [Hz]
constexpr int kDefaultFeedbackPublishRate = 10;  // [Hz]
constexpr double kDefaultVelocity = 0.1;         // [m/s]
constexpr double kDefaultMaxForce = 20;          // [N]
constexpr double kDefaultWidthTolerance = 0.002; // [m]
constexpr double kDefaultActionTimeout = 10.0;   // [s]
constexpr char kDefaultGripperJointName[] = "finger_width_joint";

// Bounded wait for the robot to become operational when using a normal RDK instance.
constexpr std::chrono::seconds kOperationalTimeout {30};

// Time the fingers must stay still before a command is considered finished, which also gives a
// freshly delivered command time to show up as motion in the gripper states.
constexpr std::chrono::milliseconds kSettleTime {500};

std::chrono::nanoseconds ToNanoseconds(double seconds)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds));
}

} // namespace

namespace flexiv_gripper {

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
: Node("flexiv_gripper_node", options)
{
    this->declare_parameter("robot_sn", std::string());
    this->declare_parameter("gripper_name", std::string());
    this->declare_parameter("tool_name", std::string());
    this->declare_parameter("state_publish_rate", kDefaultStatePublishRate);
    this->declare_parameter("feedback_publish_rate", kDefaultFeedbackPublishRate);
    this->declare_parameter("default_velocity", kDefaultVelocity);
    this->declare_parameter("default_max_force", kDefaultMaxForce);
    this->declare_parameter("width_tolerance", kDefaultWidthTolerance);
    this->declare_parameter("action_timeout", kDefaultActionTimeout);
    this->declare_parameter("gripper_joint_names", std::vector<std::string>());
    this->declare_parameter("use_lite_rdk", false);

    const std::string robot_sn = this->get_parameter("robot_sn").as_string();
    if (robot_sn.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_sn' is not set");
        throw std::invalid_argument("Parameter 'robot_sn' is not set");
    }

    const std::string gripper_name = this->get_parameter("gripper_name").as_string();
    if (gripper_name.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'gripper_name' is not set");
        throw std::invalid_argument("Parameter 'gripper_name' is not set");
    }

    // The tool is usually named after the gripper, but Flexiv Elements lists tools and devices
    // separately
    std::string tool_name = this->get_parameter("tool_name").as_string();
    if (tool_name.empty()) {
        tool_name = gripper_name;
    }

    this->default_velocity_ = this->get_parameter("default_velocity").as_double();
    this->default_max_force_ = this->get_parameter("default_max_force").as_double();
    this->width_tolerance_ = this->get_parameter("width_tolerance").as_double();
    const double action_timeout = this->get_parameter("action_timeout").as_double();
    if (width_tolerance_ <= 0.0 || action_timeout <= 0.0) {
        RCLCPP_FATAL(this->get_logger(),
            "Parameters 'width_tolerance' and 'action_timeout' must be positive");
        throw std::invalid_argument(
            "Parameters 'width_tolerance' and 'action_timeout' must be positive");
    }
    this->action_timeout_ = ToNanoseconds(action_timeout);

    this->gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
    if (this->gripper_joint_names_.empty() || this->gripper_joint_names_[0].empty()) {
        RCLCPP_WARN(this->get_logger(), "Parameter 'gripper_joint_names' is not set, using '%s'",
            kDefaultGripperJointName);
        this->gripper_joint_names_ = {kDefaultGripperJointName};
    }

    const bool use_lite_rdk = this->get_parameter("use_lite_rdk").as_bool();
    this->use_lite_rdk_ = use_lite_rdk;
    const double kStatePublishRate
        = static_cast<double>(this->get_parameter("state_publish_rate").as_int());
    const double kFeedbackPublishRate
        = static_cast<double>(this->get_parameter("feedback_publish_rate").as_int());
    this->feedback_period_ = rclcpp::WallRate(kFeedbackPublishRate).period();
    this->gripper_ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "~/ready", rclcpp::QoS(1).reliable().transient_local());

    try {
        RCLCPP_INFO(this->get_logger(), "Connecting to robot %s with a %s RDK instance ...",
            robot_sn.c_str(), use_lite_rdk ? "lite" : "normal");
        robot_ = std::make_unique<flexiv::rdk::Robot>(
            robot_sn, std::vector<std::string> {}, true, use_lite_rdk);

        RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");

        if (!use_lite_rdk) {
            if (robot_->fault()) {
                RCLCPP_WARN(
                    this->get_logger(), "Fault occurred on robot server, trying to clear ...");
                if (!robot_->ClearFault()) {
                    RCLCPP_FATAL(get_logger(), "Fault cannot be cleared, exiting ...");
                    throw std::runtime_error("Fault cannot be cleared");
                }
                RCLCPP_INFO(this->get_logger(), "Fault on robot server is cleared");
            }

            if (!robot_->operational()) {
                // Enable() throws if the E-stop is not released, so report the real cause first.
                if (!robot_->estop_released()) {
                    throw std::runtime_error(flexiv_hardware::DescribeRobotCondition(
                        {robot_->connected(), robot_->operational_status(), false}));
                }

                RCLCPP_INFO(this->get_logger(), "Enabling robot ...");
                robot_->Enable();

                // Bounded, so that a robot that never becomes ready fails the node startup with
                // an actionable message instead of hanging in the constructor forever.
                const auto deadline = std::chrono::steady_clock::now() + kOperationalTimeout;
                while (!robot_->operational()) {
                    if (std::chrono::steady_clock::now() >= deadline) {
                        throw std::runtime_error(
                            "Robot did not become operational within "
                            + std::to_string(kOperationalTimeout.count()) + " s. "
                            + flexiv_hardware::DescribeRobotCondition(
                                {robot_->connected(), robot_->operational_status(), false}));
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "Waiting for the robot to become operational: %s",
                        flexiv_hardware::OperationalStatusName(robot_->operational_status())
                            .c_str());
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                RCLCPP_INFO(this->get_logger(), "Robot is now operational");
            }
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Flexiv gripper control interface");
        this->gripper_ = std::make_unique<flexiv::rdk::Gripper>(*robot_);
        this->tool_ = std::make_unique<flexiv::rdk::Tool>(*robot_);

        // Enable the specified gripper as a device
        RCLCPP_INFO(this->get_logger(), "Enabling gripper %s ...", gripper_name.c_str());
        gripper_->Enable(gripper_name);
        gripper_params_ = gripper_->params();
        RCLCPP_INFO(this->get_logger(),
            "Gripper limits: width [%.3f, %.3f] m, velocity [%.3f, %.3f] m/s, force [%.1f, %.1f] N",
            gripper_params_.min_width, gripper_params_.max_width, gripper_params_.min_vel,
            gripper_params_.max_vel, gripper_params_.min_force, gripper_params_.max_force);

        // Switch robot tool to gripper so the gravity compensation and TCP location is updated
        RCLCPP_INFO(this->get_logger(), "Switching robot tool to %s ...", tool_name.c_str());
        tool_->Switch(tool_name);

        // Manually initialize the gripper, not all grippers need this step
        RCLCPP_INFO(
            this->get_logger(), "Initializing gripper, this process takes about 10 seconds ..");
        gripper_->Init();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Gripper initialization completed");

        // Get the current gripper states
        this->current_gripper_states_ = gripper_->states();
    } catch (const std::exception& e) {
        if (use_lite_rdk) {
            RCLCPP_FATAL(this->get_logger(),
                "Failed to start gripper with a lite RDK instance: %s. Ensure the robot driver "
                "is already running with a normal RDK connection, or relaunch the gripper with "
                "parameter 'use_lite_rdk:=false' for standalone operation.",
                e.what());
        } else {
            RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        }
        throw;
    }

    // Create the stop service server
    this->stop_service_
        = create_service<Trigger>("~/stop", [this](std::shared_ptr<Trigger::Request> /*request*/,
                                                std::shared_ptr<Trigger::Response> response) {
              return StopServiceCallback(std::move(response));
          });

    // Create the action servers
    const auto kMoveAction = GripperAction::kMove;
    this->move_action_server_ = rclcpp_action::create_server<Move>(
        this, "~/move",
        [this, kMoveAction](auto /*uuid*/, auto /*goal*/) { return HandleGoal(kMoveAction); },
        [this, kMoveAction](const auto& /*goal_handle*/) { return HandleCancel(kMoveAction); },
        [this](const auto& goal_handle) {
            return std::thread {[this, goal_handle]() { ExecuteMove(goal_handle); }}.detach();
        });

    const auto kGraspAction = GripperAction::kGrasp;
    this->grasp_action_server_ = rclcpp_action::create_server<Grasp>(
        this, "~/grasp",
        [this, kGraspAction](auto /*uuid*/, auto /*goal*/) { return HandleGoal(kGraspAction); },
        [this, kGraspAction](const auto& /*goal_handle*/) { return HandleCancel(kGraspAction); },
        [this](const auto& goal_handle) {
            return std::thread {[this, goal_handle]() { ExecuteGrasp(goal_handle); }}.detach();
        });

    const auto kGripperCommandAction = GripperAction::kGripperCommand;
    this->gripper_command_action_server_ = rclcpp_action::create_server<GripperCommand>(
        this, "~/gripper_action",
        [this, kGripperCommandAction](
            auto /*uuid*/, auto /*goal*/) { return HandleGoal(kGripperCommandAction); },
        [this, kGripperCommandAction](
            const auto& /*goal_handle*/) { return HandleCancel(kGripperCommandAction); },
        [this](const auto& goal_handle) {
            return std::thread {[this, goal_handle]() {
                ExecuteGripperCommand(goal_handle);
            }}.detach();
        });

    this->gripper_joint_states_publisher_
        = this->create_publisher<sensor_msgs::msg::JointState>("~/gripper_joint_states", 1);
    this->state_publish_timer_ = this->create_wall_timer(
        rclcpp::WallRate(kStatePublishRate).period(), [this]() { return PublishGripperStates(); });

    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    this->gripper_ready_publisher_->publish(ready_msg);
    RCLCPP_INFO(this->get_logger(), "Published gripper readiness on ~/ready");
}

std::string GripperActionServer::GetGripperActionName(GripperAction action)
{
    switch (action) {
        case GripperAction::kGrasp:
            return {"Grasping"};
        case GripperAction::kMove:
            return {"Moving"};
        case GripperAction::kGripperCommand:
            return {"GripperCommand"};
        default:
            throw std::invalid_argument("Invalid gripper action");
    }
}

rclcpp_action::CancelResponse GripperActionServer::HandleCancel(GripperAction action)
{
    const auto action_name = GetGripperActionName(action);
    RCLCPP_INFO(this->get_logger(), "Canceling %s action", action_name.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GripperActionServer::HandleGoal(GripperAction action)
{
    const auto action_name = GetGripperActionName(action);
    RCLCPP_INFO(this->get_logger(), "Received %s action request", action_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

double GripperActionServer::ClampToGripperRange(
    double value, double min, double max, const char* what) const
{
    // No limits reported, or a 0 the RDK rejects with its own message
    if (max <= 0.0 || value == 0.0) {
        return value;
    }
    const double clamped = std::copysign(std::clamp(std::abs(value), min, max), value);
    if (clamped != value) {
        RCLCPP_WARN(this->get_logger(), "Gripper %s %.3f is outside [%.3f, %.3f], using %.3f", what,
            value, min, max, clamped);
    }
    return clamped;
}

void GripperActionServer::ExecuteMove(const std::shared_ptr<GoalHandleMove>& goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const double width = goal->width;
    const double velocity
        = ClampToGripperRange(goal->velocity > 0.0 ? goal->velocity : default_velocity_,
            gripper_params_.min_vel, gripper_params_.max_vel, "velocity");
    const double max_force
        = ClampToGripperRange(goal->max_force > 0.0 ? goal->max_force : default_max_force_,
            gripper_params_.min_force, gripper_params_.max_force, "force");
    auto command
        = [this, width, velocity, max_force]() { gripper_->Move(width, velocity, max_force); };
    ExecuteCommand(goal_handle, GripperAction::kMove, command, width, velocity);
}

void GripperActionServer::ExecuteGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle)
{
    const double force = ClampToGripperRange(goal_handle->get_goal()->force,
        gripper_params_.min_force, gripper_params_.max_force, "force");
    auto command = [this, force]() { gripper_->Grasp(force); };
    ExecuteCommand(goal_handle, GripperAction::kGrasp, command, std::nullopt,
        ClampToGripperRange(
            default_velocity_, gripper_params_.min_vel, gripper_params_.max_vel, "velocity"));
}

template <typename T>
void GripperActionServer::ExecuteCommand(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle, GripperAction action,
    const std::function<void()>& command, std::optional<double> target_width, double velocity)
{
    const auto action_name = GetGripperActionName(action);
    RCLCPP_INFO(this->get_logger(), "Gripper %s action has been received", action_name.c_str());

    const auto goal_id = ++active_goal_id_;
    auto result = std::make_shared<typename T::Result>();
    try {
        command();
    } catch (const std::exception& e) {
        result->success = false;
        result->error = e.what();
        RCLCPP_ERROR(
            this->get_logger(), "Gripper %s action has failed: %s", action_name.c_str(), e.what());
        goal_handle->abort(result);
        return;
    }

    flexiv::rdk::GripperStates states;
    const auto completion = WaitForCompletion(
        goal_id, target_width, velocity, [&goal_handle]() { return goal_handle->is_canceling(); },
        [&goal_handle](const flexiv::rdk::GripperStates& current) {
            auto feedback = std::make_shared<typename T::Feedback>();
            feedback->current_width = current.width;
            feedback->current_force = current.force;
            feedback->moving = current.is_moving;
            goal_handle->publish_feedback(feedback);
        },
        states);

    switch (completion) {
        case Completion::kShutdown:
            return;
        case Completion::kCanceled:
            result->success = false;
            result->error = "Canceled";
            RCLCPP_INFO(
                this->get_logger(), "Gripper %s action has been canceled", action_name.c_str());
            goal_handle->canceled(result);
            return;
        case Completion::kReachedGoal:
        case Completion::kUnverified:
            result->success = true;
            RCLCPP_INFO(this->get_logger(),
                "Gripper %s action has been completed, width: %.4f m, force: %.2f N",
                action_name.c_str(), states.width, states.force);
            goal_handle->succeed(result);
            return;
        case Completion::kStalled:
            result->error = "Gripper stopped at width " + std::to_string(states.width)
                            + " m before reaching the target width "
                            + std::to_string(target_width.value_or(0.0)) + " m";
            break;
        case Completion::kTimedOut:
            result->error = "Gripper did not finish within the action timeout";
            break;
        case Completion::kPreempted:
            result->error = "Preempted by a newer gripper goal or a stop request";
            break;
        case Completion::kNoMotion:
            result->error
                = "The gripper did not move. Grasp requires a gripper that supports "
                  "direct force control";
            break;
    }
    result->success = false;
    RCLCPP_ERROR(this->get_logger(), "Gripper %s action has failed: %s", action_name.c_str(),
        result->error.c_str());
    goal_handle->abort(result);
}

void GripperActionServer::ExecuteGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle)
{
    const auto action_name = GetGripperActionName(GripperAction::kGripperCommand);
    RCLCPP_INFO(this->get_logger(), "Gripper %s action has been received", action_name.c_str());

    const auto goal = goal_handle->get_goal();
    const double target_width = goal->command.position;
    const double max_force = ClampToGripperRange(
        goal->command.max_effort > 0.0 ? goal->command.max_effort : default_max_force_,
        gripper_params_.min_force, gripper_params_.max_force, "force");
    const double velocity = ClampToGripperRange(
        default_velocity_, gripper_params_.min_vel, gripper_params_.max_vel, "velocity");

    auto result = std::make_shared<GripperCommand::Result>();
    const auto fill_result = [&result](const flexiv::rdk::GripperStates& states) {
        result->position = states.width;
        result->effort = states.force;
    };

    const double max_width = gripper_->params().max_width;
    if (target_width > max_width || target_width < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid gripper target width: %f. Max width = %f",
            target_width, max_width);
        fill_result(gripper_->states());
        goal_handle->abort(result);
        return;
    }

    const auto goal_id = ++active_goal_id_;
    try {
        gripper_->Move(target_width, velocity, max_force);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            this->get_logger(), "Gripper %s action has failed: %s", action_name.c_str(), e.what());
        fill_result(gripper_->states());
        goal_handle->abort(result);
        return;
    }

    flexiv::rdk::GripperStates states;
    const auto completion = WaitForCompletion(
        goal_id, target_width, velocity, [&goal_handle]() { return goal_handle->is_canceling(); },
        [&goal_handle](const flexiv::rdk::GripperStates& current) {
            auto feedback = std::make_shared<GripperCommand::Feedback>();
            feedback->position = current.width;
            feedback->effort = current.force;
            goal_handle->publish_feedback(feedback);
        },
        states);
    fill_result(states);

    switch (completion) {
        case Completion::kShutdown:
            return;
        case Completion::kCanceled:
            RCLCPP_INFO(
                this->get_logger(), "Gripper %s action has been canceled", action_name.c_str());
            goal_handle->canceled(result);
            return;
        case Completion::kReachedGoal:
        case Completion::kUnverified:
            result->reached_goal = true;
            RCLCPP_INFO(
                this->get_logger(), "Gripper %s action has been completed", action_name.c_str());
            goal_handle->succeed(result);
            return;
        case Completion::kStalled:
            // Blocked by an object, the usual outcome when closing on it to grasp
            result->stalled = true;
            RCLCPP_INFO(this->get_logger(),
                "Gripper %s action stalled at width %.4f m (target %.4f m), force: %.2f N",
                action_name.c_str(), states.width, target_width, states.force);
            goal_handle->succeed(result);
            return;
        case Completion::kTimedOut:
            RCLCPP_ERROR(this->get_logger(),
                "Gripper %s action has failed: did not finish within the action timeout",
                action_name.c_str());
            break;
        case Completion::kPreempted:
            RCLCPP_ERROR(this->get_logger(),
                "Gripper %s action has failed: preempted by a newer goal or a stop request",
                action_name.c_str());
            break;
        case Completion::kNoMotion:
            RCLCPP_ERROR(this->get_logger(),
                "Gripper %s action has failed: the gripper did not move", action_name.c_str());
            break;
    }
    goal_handle->abort(result);
}

GripperActionServer::Completion GripperActionServer::WaitForCompletion(std::uint64_t goal_id,
    std::optional<double> target_width, double velocity, const std::function<bool()>& is_canceling,
    const std::function<void(const flexiv::rdk::GripperStates&)>& publish_feedback,
    flexiv::rdk::GripperStates& final_states)
{
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto deadline = start + action_timeout_;

    // Only used when the states never change: a full stroke at the commanded velocity
    const double stroke_time = velocity > 0.0 ? gripper_->params().max_width / velocity : 0.0;
    const auto unverified_after
        = std::min(deadline, start + kSettleTime + ToNanoseconds(stroke_time));

    // Long enough for the fingers to cover the width tolerance even at a low velocity
    const auto steady_time = std::max<std::chrono::nanoseconds>(
        kSettleTime, ToNanoseconds(velocity > 0.0 ? 2.0 * width_tolerance_ / velocity : 0.0));

    const auto initial = gripper_->states();
    bool states_changed = false;
    double reference_width = initial.width;
    auto last_width_change = start;

    while (true) {
        std::this_thread::sleep_for(feedback_period_);
        final_states = gripper_->states();
        const auto now = Clock::now();

        if (!rclcpp::ok()) {
            return Completion::kShutdown;
        }
        if (active_goal_id_ != goal_id) {
            return Completion::kPreempted;
        }
        if (is_canceling()) {
            try {
                gripper_->Stop();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to stop the gripper: %s", e.what());
            }
            final_states = gripper_->states();
            return Completion::kCanceled;
        }
        publish_feedback(final_states);

        states_changed = states_changed || final_states.width != initial.width
                         || final_states.force != initial.force
                         || final_states.is_moving != initial.is_moving;
        states_live_ = states_live_ || states_changed;

        // A lite RDK instance may not receive gripper states, so unchanged states prove nothing
        // until they have been seen to change
        if (use_lite_rdk_ && !states_live_) {
            if (now >= unverified_after) {
                RCLCPP_WARN(this->get_logger(),
                    "Gripper states have not changed since the node started, so completion cannot "
                    "be verified (a lite RDK instance may not receive gripper states). Assuming it "
                    "finished after %.2f s",
                    std::chrono::duration<double>(now - start).count());
                return Completion::kUnverified;
            }
            continue;
        }

        if (std::abs(final_states.width - reference_width) > width_tolerance_) {
            reference_width = final_states.width;
            last_width_change = now;
        }
        // Also accept a steady width, in case is_moving stays set while holding force
        const bool stopped = !final_states.is_moving || now - last_width_change >= steady_time;
        const bool settled = now - start >= kSettleTime;

        if (target_width) {
            const bool at_target = std::abs(final_states.width - *target_width) <= width_tolerance_;
            if (stopped && at_target) {
                return Completion::kReachedGoal;
            }
            if (stopped && settled) {
                return Completion::kStalled;
            }
        } else if (stopped && settled) {
            // A grasp that moves nothing was ignored, e.g. without direct force control support
            return states_changed ? Completion::kReachedGoal : Completion::kNoMotion;
        }

        if (now >= deadline) {
            return Completion::kTimedOut;
        }
    }
}

void GripperActionServer::StopServiceCallback(const std::shared_ptr<Trigger::Response>& response)
{
    RCLCPP_INFO(this->get_logger(), "Stopping the gripper...");
    ++active_goal_id_;
    try {
        gripper_->Stop();
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Gripper has been stopped");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Failed to stop the gripper: %s", e.what());
    }
}

void GripperActionServer::PublishGripperStates()
{
    std::lock_guard<std::mutex> lock(gripper_states_mutex_);
    const auto previous = this->current_gripper_states_;
    this->current_gripper_states_ = gripper_->states();
    if (current_gripper_states_.width != previous.width
        || current_gripper_states_.force != previous.force
        || current_gripper_states_.is_moving != previous.is_moving) {
        states_live_ = true;
    }
    // Modify the gripper joint states based on the mounted gripper type
    // The gripper joint states below is for the Flexiv Grav GN-01 gripper
    sensor_msgs::msg::JointState gripper_joint_states;
    gripper_joint_states.header.stamp = this->now();
    gripper_joint_states.name.push_back(this->gripper_joint_names_[0]);
    gripper_joint_states.position.push_back(this->current_gripper_states_.width);
    gripper_joint_states.velocity.push_back(0.0);
    gripper_joint_states.effort.push_back(this->current_gripper_states_.force);
    this->gripper_joint_states_publisher_->publish(gripper_joint_states);
}

} // namespace flexiv_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(flexiv_gripper::GripperActionServer)

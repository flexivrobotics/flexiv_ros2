#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

#include "flexiv_gripper/gripper_action_server.hpp"

namespace flexiv_gripper {

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
: Node("flexiv_gripper_node", options)
{
    this->declare_parameter("robot_sn", std::string());
    this->declare_parameter("gripper_name", std::string());
    this->declare_parameter("state_publish_rate", kDefaultStatePublishRate);
    this->declare_parameter("feedback_publish_rate", kDefaultFeedbackPublishRate);
    this->declare_parameter("default_velocity", kDefaultVelocity);
    this->declare_parameter("default_max_force", kDefaultMaxForce);
    this->declare_parameter("gripper_joint_names", std::vector<std::string>());

    std::string robot_sn;
    if (!this->get_parameter("robot_sn", robot_sn)) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_sn' is not set");
        throw std::invalid_argument("Parameter 'robot_sn' is not set");
    }

    std::string gripper_name;
    if (!this->get_parameter("gripper_name", gripper_name)) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'gripper_name' is not set");
        throw std::invalid_argument("Parameter 'gripper_name' is not set");
    }

    this->default_velocity_ = this->get_parameter("default_velocity").as_double();
    this->default_max_force_ = this->get_parameter("default_max_force").as_double();

    if (!this->get_parameter("gripper_joint_names", this->gripper_joint_names_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter 'gripper_joint_names' is not set");
        this->gripper_joint_names_ = {""};
    }

    const double kStatePublishRate
        = static_cast<double>(this->get_parameter("state_publish_rate").as_int());
    const double kFeedbackPublishRate
        = static_cast<double>(this->get_parameter("feedback_publish_rate").as_int());
    this->future_wait_timeout_ = rclcpp::WallRate(kFeedbackPublishRate).period();

    try {
        RCLCPP_INFO(this->get_logger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);

        RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");

        // Clear fault on robot server if any
        if (robot_->fault()) {
            RCLCPP_WARN(this->get_logger(), "Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot_->ClearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot_->fault()) {
                RCLCPP_FATAL(get_logger(), "Fault cannot be cleared, exiting ...");
                throw std::runtime_error("Fault cannot be cleared");
            }
            RCLCPP_INFO(this->get_logger(), "Fault on robot server is cleared");
        }

        // Enable the robot
        if (!robot_->operational(false)) {
            RCLCPP_INFO(this->get_logger(), "Enabling robot ...");
            robot_->Enable();

            // Wait for the robot to become operational
            while (!robot_->operational(false)) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            RCLCPP_INFO(this->get_logger(), "Robot is now operational");
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Flexiv gripper control interface");
        this->gripper_ = std::make_unique<flexiv::rdk::Gripper>(*robot_);
        this->tool_ = std::make_unique<flexiv::rdk::Tool>(*robot_);

        // Enable the specified gripper as a device
        RCLCPP_INFO(this->get_logger(), "Enabling gripper %s ...", gripper_name.c_str());
        gripper_->Enable(gripper_name);

        // Switch robot tool to gripper so the gravity compensation and TCP location is updated
        RCLCPP_INFO(this->get_logger(), "Switching robot tool to %s ...", gripper_name.c_str());
        tool_->Switch(gripper_name);

        // Manually initialize the gripper, not all grippers need this step
        RCLCPP_INFO(
            this->get_logger(), "Initializing gripper, this process takes about 10 seconds ..");
        gripper_->Init();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Gripper initialization completed");

        // Get the current gripper states
        this->current_gripper_states_ = gripper_->states();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        throw e;
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

void GripperActionServer::ExecuteMove(const std::shared_ptr<GoalHandleMove>& goal_handle)
{
    auto command = [this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        gripper_->Move(goal->width, goal->velocity, goal->max_force);
    };
    ExecuteCommand(goal_handle, GripperAction::kMove, command);
}

void GripperActionServer::ExecuteGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle)
{
    auto command = [this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        gripper_->Grasp(goal->force);
    };
    ExecuteCommand(goal_handle, GripperAction::kGrasp, command);
}

void GripperActionServer::ExecuteGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const double target_width = goal->command.position;

    std::unique_lock<std::mutex> guard(gripper_states_mutex_);
    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    const double current_width = current_gripper_states_.width;
    if (target_width > gripper_->params().max_width || target_width < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid gripper target width: %f. Max width = %f",
            target_width, gripper_->params().max_width);
        goal_handle->abort(result);
        return;
    }
    if (std::abs(target_width - current_width) < 1e-3) {
        RCLCPP_INFO(this->get_logger(), "Gripper is already at the target width: %f", target_width);
        result->effort = current_gripper_states_.force;
        result->position = current_gripper_states_.width;
        result->reached_goal = true;
        result->stalled = false;
        goal_handle->succeed(result);
        return;
    }
    guard.unlock();

    auto command = [this, target_width]() {
        gripper_->Move(target_width, kDefaultVelocity, kDefaultMaxForce);
    };

    ExecuteGripperCommandHelper(goal_handle, command);
}

void GripperActionServer::ExecuteGripperCommandHelper(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
    const std::function<void()>& command)
{
    const auto action_name = GetGripperActionName(GripperAction::kGripperCommand);
    RCLCPP_INFO(this->get_logger(), "Gripper %s action has been received", action_name.c_str());

    auto command_execution_result = [this, command]() {
        auto result = std::make_shared<GripperCommand::Result>();
        try {
            command();
            result->reached_goal = true;
        } catch (const std::exception& e) {
            result->reached_goal = false;
            RCLCPP_INFO(this->get_logger(), "Gripper command failed: %s", e.what());
        }
        return result;
    };

    std::future<std::shared_ptr<typename GripperCommand::Result>> result_future
        = std::async(std::launch::async, command_execution_result);

    while (!IsResultReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            gripper_->Stop();
            auto result = result_future.get();
            RCLCPP_INFO(
                this->get_logger(), "Gripper %s action has been canceled", action_name.c_str());
            goal_handle->canceled(result);
            return;
        }
        PublishGripperCommandFeedback(goal_handle);
    }

    if (rclcpp::ok()) {
        const auto result = result_future.get();
        std::lock_guard<std::mutex> guard(gripper_states_mutex_);
        result->position = current_gripper_states_.width;
        result->effort = current_gripper_states_.force;
        if (result->reached_goal) {
            RCLCPP_INFO(
                this->get_logger(), "Gripper %s action has been completed", action_name.c_str());
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper %s action has failed", action_name.c_str());
            goal_handle->abort(result);
        }
    }
}

void GripperActionServer::StopServiceCallback(const std::shared_ptr<Trigger::Response>& response)
{
    RCLCPP_INFO(this->get_logger(), "Stopping the gripper...");
    auto result = CommandExecutionResult<Move>([this]() { gripper_->Stop(); })();
    response->success = result->success;
    response->message = result->error;
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Gripper has been stopped");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop the gripper");
    }
    if (!response->message.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Error message: %s", response->message.c_str());
    }
}

void GripperActionServer::PublishGripperStates()
{
    std::lock_guard<std::mutex> lock(gripper_states_mutex_);
    this->current_gripper_states_ = gripper_->states();
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

void GripperActionServer::PublishGripperCommandFeedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle)
{
    auto feedback = std::make_shared<GripperCommand::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_states_mutex_);
    feedback->position = current_gripper_states_.width;
    feedback->effort = current_gripper_states_.force;
    goal_handle->publish_feedback(feedback);
}

} // namespace flexiv_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(flexiv_gripper::GripperActionServer)

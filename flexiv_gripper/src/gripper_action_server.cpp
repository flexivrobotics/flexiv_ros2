#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

#include "flexiv_gripper/gripper_action_server.hpp"

namespace flexiv_gripper {

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
: Node("flexiv_gripper_node", options)
{
    this->declare_parameter("robot_sn", std::string());
    this->declare_parameter("default_state_publish_rate", kDefaultStatePublishRate);
    this->declare_parameter("default_feedback_publish_rate", kDefaultFeedbackPublishRate);
    this->declare_parameter("default_velocity", kDefaultVelocity);
    this->declare_parameter("default_max_force", kDefaultMaxForce);

    std::string robot_sn;
    if (!this->get_parameter("robot_sn", robot_sn)) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_sn' is not set");
        throw std::invalid_argument("Parameter 'robot_sn' is not set");
    }

    this->default_velocity_ = this->get_parameter("default_velocity").as_double();
    this->default_max_force_ = this->get_parameter("default_max_force").as_double();

    const double kStatePublishRate
        = static_cast<double>(this->get_parameter("default_state_publish_rate").as_int());
    const double kFeedbackPublishRate
        = static_cast<double>(this->get_parameter("default_feedback_publish_rate").as_int());
    this->future_wait_timeout_ = rclcpp::WallRate(kFeedbackPublishRate).period();

    try {
        RCLCPP_INFO(this->get_logger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Could not connect to robot");
        throw e;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");

    try {
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
        RCLCPP_INFO(this->get_logger(), "Enabling robot ...");
        robot_->Enable();

        // Wait for the robot to become operational
        while (!robot_->operational(false)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "Robot is now operational");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Could not enable robot.");
        throw e;
    }

    // Gripper control is not available if the robot is in IDLE mode, so switch to some mode other
    // than IDLE, e.g. NRT_JOINT_POSITION
    if (robot_->mode() == flexiv::rdk::Mode::IDLE) {
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_JOINT_POSITION);
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Flexiv gripper control interface");
    this->gripper_ = std::make_unique<flexiv::rdk::Gripper>(*robot_);

    // Manually initialize the gripper, not all grippers need this step
    RCLCPP_INFO(this->get_logger(), "Initializing gripper, this process takes about 10 seconds ..");
    gripper_->Init();
    RCLCPP_INFO(this->get_logger(), "Gripper initialization completed");

    // Get the current gripper states
    this->current_gripper_states_ = gripper_->states();
    this->is_gripper_moving_ = gripper_->moving();

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
        [this](const auto goal_handle) {
            return std::thread {[goal_handle, this]() { ExecuteMove(goal_handle); }}.detach();
        });

    const auto kGraspAction = GripperAction::kGrasp;
    this->grasp_action_server_ = rclcpp_action::create_server<Grasp>(
        this, "~/grasp",
        [this, kGraspAction](auto /*uuid*/, auto /*goal*/) { return HandleGoal(kGraspAction); },
        [this, kGraspAction](const auto& /*goal_handle*/) { return HandleCancel(kGraspAction); },
        [this](const auto goal_handle) {
            return std::thread {[goal_handle, this]() { ExecuteGrasp(goal_handle); }}.detach();
        });

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
    auto command = [goal_handle, this]() {
        const auto goal = goal_handle->get_goal();
        return gripper_->Move(goal->width, goal->velocity, goal->max_force);
    };
    ExecuteCommand(goal_handle, GripperAction::kMove, command);
}

void GripperActionServer::ExecuteGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle)
{
    auto command = [goal_handle, this]() {
        const auto goal = goal_handle->get_goal();
        return gripper_->Grasp(goal->force);
    };
    ExecuteCommand(goal_handle, GripperAction::kGrasp, command);
}

void GripperActionServer::StopServiceCallback(const std::shared_ptr<Trigger::Response>& response)
{
    RCLCPP_INFO(this->get_logger(), "Stopping the gripper...");
    auto result = CommandExecutionResult<Move>([this]() { return gripper_->Stop(); })();
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
    this->is_gripper_moving_ = gripper_->moving();
    // TODO: Publish the gripper states to the topic
}

} // namespace flexiv_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(flexiv_gripper::GripperActionServer)

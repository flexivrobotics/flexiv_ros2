/**
 * @file gripper_action_server.hpp
 * @brief Header file for GripperActionServer class
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_GRIPPER__GRIPPER_ACTION_SERVER_HPP_
#define FLEXIV_GRIPPER__GRIPPER_ACTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// ROS
#include "control_msgs/action/gripper_command.hpp"
#include "flexiv_msgs/action/grasp.hpp"
#include "flexiv_msgs/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// Flexiv
#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/gripper.hpp"
#include "flexiv/rdk/robot.hpp"
#include "flexiv/rdk/tool.hpp"

#include "flexiv_hardware/fault_recovery.hpp"

namespace flexiv_gripper {

class GripperActionServer : public rclcpp::Node
{

public:
    using Grasp = flexiv_msgs::action::Grasp;
    using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;

    using Move = flexiv_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

    using Trigger = std_srvs::srv::Trigger;

    explicit GripperActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // gripper actions
    enum class GripperAction
    {
        kGrasp,
        kMove,
        kGripperCommand
    };

    // How a gripper action ended, as observed from the gripper states
    enum class Completion
    {
        kReachedGoal, // stopped at the target width, or a grasp settled
        kStalled,     // stopped before reaching the target width
        kUnverified,  // states never changed (e.g. lite RDK), assumed done after an estimate
        kNoMotion,    // states are live but a grasp moved nothing
        kTimedOut,
        kCanceled,
        kPreempted, // superseded by a newer goal or a stop request
        kShutdown
    };

    // return string of the gripper action
    static std::string GetGripperActionName(GripperAction action);

    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    std::unique_ptr<flexiv::rdk::Gripper> gripper_;
    std::unique_ptr<flexiv::rdk::Tool> tool_;

    rclcpp_action::Server<Grasp>::SharedPtr grasp_action_server_;
    rclcpp_action::Server<Move>::SharedPtr move_action_server_;
    rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_action_server_;
    rclcpp::Service<Trigger>::SharedPtr stop_service_;
    rclcpp::TimerBase::SharedPtr state_publish_timer_;

    // Limits of the enabled gripper, read once after Enable()
    flexiv::rdk::GripperParams gripper_params_;

    std::mutex gripper_states_mutex_;
    flexiv::rdk::GripperStates current_gripper_states_;

    // Set once the gripper states are seen to change, which proves they are received
    std::atomic<bool> states_live_ {false};

    bool use_lite_rdk_ = false;
    double default_velocity_;
    double default_max_force_;

    /**
     * @brief Clamp a commanded velocity or force magnitude into the gripper's range, keeping its
     * sign, and warn when it changes. The defaults suit one gripper and may not suit another.
     */
    double ClampToGripperRange(double value, double min, double max, const char* what) const;
    double width_tolerance_;
    std::chrono::nanoseconds action_timeout_ {0};
    std::chrono::nanoseconds feedback_period_ {0};

    // Incremented by every new goal and stop request, so that older goals stop waiting
    std::atomic<std::uint64_t> active_goal_id_ {0};

    // Gripper joint states publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_ready_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_states_publisher_;
    std::vector<std::string> gripper_joint_names_;

    /**
     * @brief Publish the current gripper states.
     */
    void PublishGripperStates();

    /**
     * @brief Stop the gripper.
     * @param[out] response Success or failure of the service call.
     */
    void StopServiceCallback(const std::shared_ptr<Trigger::Response>& response);

    /**
     * @brief Handle the gripper action cancel request.
     * @param[in] action Gripper action to cancel.
     */
    rclcpp_action::CancelResponse HandleCancel(GripperAction action);

    /**
     * @brief Handle the gripper action goal request.
     * @param[in] action Gripper action to handle.
     */
    rclcpp_action::GoalResponse HandleGoal(GripperAction action);

    /**
     * @brief Perform the gripper move action.
     */
    void ExecuteMove(const std::shared_ptr<GoalHandleMove>& goal_handle);

    /**
     * @brief Perform the gripper grasp action.
     */
    void ExecuteGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);

    /**
     * @brief Perform the gripper command action.
     * @param[in] goal_handle The goal handle of the action.
     */
    void ExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

    /**
     * @brief Send a Move or Grasp command, wait for it to complete and report the result.
     * @tparam T Gripper action message type (Grasp or Move).
     * @param[in] goal_handle The goal handle of the action.
     * @param[in] action The gripper action to execute.
     * @param[in] command The RDK function to execute the gripper command.
     * @param[in] target_width Target width for position commands, empty for a grasp.
     * @param[in] velocity Finger velocity used to estimate the motion duration [m/s].
     */
    template <typename T>
    void ExecuteCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle,
        GripperAction action, const std::function<void()>& command,
        std::optional<double> target_width, double velocity);

    /**
     * @brief Poll the gripper states until the command just sent has finished.
     * @param[in] goal_id ID of the goal waiting, see active_goal_id_.
     * @param[in] target_width Target width for position commands, empty for a grasp.
     * @param[in] velocity Finger velocity used to estimate the motion duration [m/s].
     * @param[in] is_canceling Returns true when the goal is being canceled.
     * @param[in] publish_feedback Called with the latest gripper states at the feedback rate.
     * @param[out] final_states Gripper states when the wait ended.
     * @return How the command ended.
     */
    Completion WaitForCompletion(std::uint64_t goal_id, std::optional<double> target_width,
        double velocity, const std::function<bool()>& is_canceling,
        const std::function<void(const flexiv::rdk::GripperStates&)>& publish_feedback,
        flexiv::rdk::GripperStates& final_states);
};

} // namespace flexiv_gripper

#endif /* FLEXIV_GRIPPER__GRIPPER_ACTION_SERVER_HPP_ */

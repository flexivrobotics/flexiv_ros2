/**
 * @file recovery_node.hpp
 * @brief ROS node hosted by the hardware interface, exposing the error recovery action and the
 * operational status of the robot.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__RECOVERY_NODE_HPP_
#define FLEXIV_HARDWARE__RECOVERY_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "flexiv_msgs/action/error_recovery.hpp"
#include "flexiv_msgs/msg/operational_status.hpp"
#include "flexiv_msgs/msg/robot_event.hpp"
#include "flexiv_msgs/srv/get_operational_status.hpp"

#include "flexiv_hardware/fault_recovery.hpp"
#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

/**
 * @brief Node that supervises the robot outside the real-time control loop.
 *
 * All blocking RDK system control calls happen here, never in read() or write(). The node is added
 * to the controller manager's executor by the hardware interface, so it needs no thread of its
 * own; the recovery sequence itself runs on a detached worker so that the executor stays
 * responsive while a fault is being cleared.
 */
class RecoveryNode : public rclcpp::Node
{
public:
    using ErrorRecovery = flexiv_msgs::action::ErrorRecovery;
    using GoalHandleErrorRecovery = rclcpp_action::ServerGoalHandle<ErrorRecovery>;
    using GetOperationalStatus = flexiv_msgs::srv::GetOperationalStatus;

    /**
     * @brief Construct the recovery node.
     * @param[in] robot_sn Serial number of the robot, used as the node namespace so that dual-arm
     * setups do not collide.
     * @param[in] robot System control interface of the robot. Must outlive this node.
     * @param[in] status Status shared with the real-time control loop. Must outlive this node.
     */
    RecoveryNode(const std::string& robot_sn, RobotSystemControl& robot,
        std::shared_ptr<DriverStatus> status);

    ~RecoveryNode() override;

private:
    /** @brief Build the status message from the latched robot condition. */
    flexiv_msgs::msg::OperationalStatus BuildStatusMessage();

    /** @brief Publish the status message, and refresh the event log on a new fault. */
    void PublishStatus();

    rclcpp_action::GoalResponse HandleGoal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ErrorRecovery::Goal> goal);

    rclcpp_action::CancelResponse HandleCancel(
        const std::shared_ptr<GoalHandleErrorRecovery>& goal_handle);

    void HandleAccepted(const std::shared_ptr<GoalHandleErrorRecovery>& goal_handle);

    /** @brief Run the recovery sequence to completion. Runs on a detached worker thread. */
    void ExecuteRecovery(const std::shared_ptr<GoalHandleErrorRecovery>& goal_handle);

    /** @brief Refresh recent_events_ from the robot's event log. */
    void RefreshRecentEvents();

    RobotSystemControl& robot_;
    std::shared_ptr<DriverStatus> status_;

    rclcpp_action::Server<ErrorRecovery>::SharedPtr error_recovery_action_server_;
    rclcpp::Service<GetOperationalStatus>::SharedPtr get_operational_status_service_;
    rclcpp::Publisher<flexiv_msgs::msg::OperationalStatus>::SharedPtr operational_status_publisher_;
    rclcpp::TimerBase::SharedPtr status_publish_timer_;

    rclcpp::CallbackGroup::SharedPtr recovery_callback_group_;
    rclcpp::CallbackGroup::SharedPtr status_callback_group_;

    /** Guards against a second recovery starting while one is running. */
    std::atomic<bool> recovery_in_progress_ {false};

    std::mutex recent_events_mutex_;
    std::vector<flexiv_msgs::msg::RobotEvent> recent_events_;
    bool previous_fault_ = false;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__RECOVERY_NODE_HPP_ */

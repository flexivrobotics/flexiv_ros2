#ifndef FLEXIV_ROBOT_STATES_BROADCASTER__FLEXIV_ROBOT_STATES_BROADCASTER_HPP_
#define FLEXIV_ROBOT_STATES_BROADCASTER__FLEXIV_ROBOT_STATES_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "flexiv_robot_states_broadcaster/flexiv_robot_states.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"
#include "flexiv_robot_states_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace flexiv_robot_states_broadcaster {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FlexivRobotStatesBroadcaster : public controller_interface::ControllerInterface
{
public:
    FlexivRobotStatesBroadcaster();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::unique_ptr<semantic_components::FlexivRobotStates> flexiv_robot_states_;

    using StatePublisher = realtime_tools::RealtimePublisher<flexiv_msgs::msg::RobotStates>;
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr flexiv_robot_states_publisher_;
    std::unique_ptr<StatePublisher> realtime_publisher_;
};

} /* namespace flexiv_robot_states_broadcaster */

#endif /* FLEXIV_ROBOT_STATES_BROADCASTER__FLEXIV_ROBOT_STATES_BROADCASTER_HPP_ */

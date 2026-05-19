#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace flexiv_gripper {

class WaitForGripperReady : public rclcpp::Node
{
public:
    WaitForGripperReady()
    : Node("wait_for_gripper_ready")
    {
        this->declare_parameter("ready_topic", std::string("/flexiv_gripper_node/ready"));
        this->declare_parameter("timeout_sec", 30.0);

        ready_topic_ = this->get_parameter("ready_topic").as_string();
        const double timeout_sec = this->get_parameter("timeout_sec").as_double();
        if (timeout_sec <= 0.0) {
            throw std::invalid_argument("Parameter 'timeout_sec' must be positive");
        }

        timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(timeout_sec));

        readiness_subscription_ = this->create_subscription<std_msgs::msg::Bool>(ready_topic_,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (ready_ || !msg->data) {
                    return;
                }

                ready_ = true;
                exit_code_ = 0;
                RCLCPP_INFO(
                    this->get_logger(), "Received gripper readiness on %s", ready_topic_.c_str());
                rclcpp::shutdown();
            });

        timeout_timer_ = this->create_wall_timer(timeout_, [this]() {
            if (ready_) {
                return;
            }

            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for gripper readiness on %s",
                ready_topic_.c_str());
            rclcpp::shutdown();
        });

        RCLCPP_INFO(this->get_logger(), "Waiting up to %.1f seconds for gripper readiness on %s",
            std::chrono::duration<double>(timeout_).count(), ready_topic_.c_str());
    }

    int exit_code() const { return exit_code_; }

private:
    std::string ready_topic_;
    bool ready_ {false};
    int exit_code_ {1};
    std::chrono::nanoseconds timeout_ {std::chrono::seconds(30)};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr readiness_subscription_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
};

} // namespace flexiv_gripper

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<flexiv_gripper::WaitForGripperReady>();
    rclcpp::spin(node);

    const auto exit_code = node->exit_code();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return exit_code;
}

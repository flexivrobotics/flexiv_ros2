/**
 * @file test_cartesian_motion_force_config_node.cpp
 * @brief Unit tests for the Cartesian motion-force interface: validation, hold-or-deliver, and
 * re-application on mode entry. Needs no robot connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "flexiv_hardware/cartesian_motion_force_config_node.hpp"

using flexiv::rdk::Mode;
using flexiv_hardware::CartesianArray;
using flexiv_hardware::CartesianMotionForceBounds;
using flexiv_hardware::CartesianMotionForceConfigNode;
using flexiv_hardware::CartesianMotionForceSetters;
using flexiv_hardware::CartesianMotionLimits;
using flexiv_hardware::DriverState;
using flexiv_hardware::DriverStatus;
namespace srv = flexiv_msgs::srv;

namespace {

constexpr size_t kDoF = 7;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/**
 * @brief Spins a CartesianMotionForceConfigNode against stub setters that log every call by name,
 * so the handlers and the re-apply order are exercised without a robot.
 */
class CartesianConfigServiceTest : public ::testing::Test
{
protected:
    // Guarded, and deliberately not shut down per suite: re-initializing rclcpp in a process that
    // has already shut it down crashes.
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void StartNode(Mode mode, size_t num_robots = 1)
    {
        num_robots_ = num_robots;
        status_ = std::make_shared<DriverStatus>();
        status_->connected.store(true);
        status_->driver_state.store(DriverState::READY);
        status_->control_mode.store(mode);

        CartesianMotionForceBounds bounds;
        bounds.k_x_nom.assign(num_robots, CartesianArray {1000, 1000, 1000, 100, 100, 100});
        bounds.q_min.assign(kDoF * num_robots, -2.0);
        bounds.q_max.assign(kDoF * num_robots, 2.0);

        const auto log = [this](const std::string& name) {
            if (throw_on_ == name) {
                throw std::runtime_error("stub: failed to deliver the request");
            }
            if (logic_error_on_ == name) {
                throw std::logic_error("stub: wrong control mode");
            }
            if (invalid_argument_on_ == name) {
                throw std::invalid_argument("stub: value out of range");
            }
            calls_.push_back(name);
        };

        CartesianMotionForceSetters setters;
        setters.set_cartesian_impedance = [this, log](const std::vector<CartesianArray>& k_x,
                                              const std::vector<CartesianArray>&) {
            log("impedance");
            last_k_x_ = k_x;
        };
        setters.set_max_contact_wrench
            = [log](const std::vector<CartesianArray>&) { log("max_contact_wrench"); };
        setters.set_null_space_posture
            = [log](const std::vector<double>&) { log("null_space_posture"); };
        setters.set_null_space_objectives
            = [log](const std::vector<flexiv_hardware::NullSpaceObjectives>&) {
                  log("null_space_objectives");
              };
        setters.set_force_control_axis
            = [log](const std::vector<flexiv_hardware::CartesianFlags>&,
                  const std::vector<flexiv_hardware::LinearArray>&) { log("force_control_axis"); };
        setters.set_force_control_frame
            = [log](const std::vector<flexiv::rdk::CoordType>&,
                  const std::vector<flexiv_hardware::PoseArray>&) { log("force_control_frame"); };
        setters.set_passive_force_control
            = [log](const std::vector<bool>&) { log("passive_force_control"); };
        setters.set_motion_limits = [this, log](const std::vector<CartesianMotionLimits>& limits) {
            log("motion_limits");
            last_limits_ = limits;
        };

        // A serial number unique to this test, so no two tests share a node namespace and service
        // discovery cannot bind to a previous test's node.
        static int instance = 0;
        const std::string robot_sn = "Rizon4-91000" + std::to_string(++instance);
        namespace_ = flexiv_hardware::SanitizeNamespace(robot_sn);

        std::vector<std::string> joint_names;
        for (size_t i = 0; i < kDoF * num_robots; ++i) {
            joint_names.push_back("joint" + std::to_string(i + 1));
        }
        node_ = std::make_shared<CartesianMotionForceConfigNode>(
            robot_sn, joint_names, bounds, status_, setters);
        client_node_ = std::make_shared<rclcpp::Node>("cartesian_config_test_client");

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_->add_node(client_node_);

        running_.store(true);
        spin_thread_ = std::thread([this]() {
            while (running_.load()) {
                executor_->spin_some(std::chrono::milliseconds(5));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }

    void TearDown() override
    {
        running_.store(false);
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        if (executor_) {
            executor_->remove_node(node_);
            executor_->remove_node(client_node_);
        }
        executor_.reset();
        client_node_.reset();
        node_.reset();
    }

    template <typename Service>
    typename Service::Response::SharedPtr Call(
        const std::string& name, typename Service::Request::SharedPtr request)
    {
        const auto endpoint
            = "/" + namespace_ + "/flexiv_cartesian_motion_force_config_node/" + name;
        auto client = client_node_->create_client<Service>(endpoint);
        if (!client->wait_for_service(std::chrono::seconds(10))) {
            ADD_FAILURE() << "service " << endpoint << " never appeared";
            return nullptr;
        }
        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            ADD_FAILURE() << "service " << endpoint << " did not respond";
            return nullptr;
        }
        return future.get();
    }

    srv::SetMaxContactWrench::Response::SharedPtr SetMaxContactWrench(double value)
    {
        auto request = std::make_shared<srv::SetMaxContactWrench::Request>();
        request->max_wrench.assign(6 * num_robots_, value);
        return Call<srv::SetMaxContactWrench>("set_max_contact_wrench", request);
    }

    srv::SetForceControlAxis::Response::SharedPtr SetForceControlAxis()
    {
        auto request = std::make_shared<srv::SetForceControlAxis::Request>();
        request->enabled_axes.assign(6 * num_robots_, false);
        request->enabled_axes[2] = true;
        return Call<srv::SetForceControlAxis>("set_force_control_axis", request);
    }

    srv::SetPassiveForceControl::Response::SharedPtr SetPassiveForceControl()
    {
        auto request = std::make_shared<srv::SetPassiveForceControl::Request>();
        request->enabled.assign(num_robots_, true);
        return Call<srv::SetPassiveForceControl>("set_passive_force_control", request);
    }

    size_t num_robots_ = 1;
    std::string namespace_;
    std::shared_ptr<DriverStatus> status_;
    std::shared_ptr<CartesianMotionForceConfigNode> node_;
    std::shared_ptr<rclcpp::Node> client_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    std::atomic<bool> running_ {false};

    std::string throw_on_;
    std::string logic_error_on_;
    std::string invalid_argument_on_;
    std::vector<std::string> calls_;
    std::vector<CartesianArray> last_k_x_;
    std::vector<CartesianMotionLimits> last_limits_;
};

}

TEST_F(CartesianConfigServiceTest, AppliesWhileInCartesianMode)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);

    auto response = SetForceControlAxis();
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_EQ(calls_, std::vector<std::string> {"force_control_axis"});
}

TEST_F(CartesianConfigServiceTest, HoldsOutsideCartesianModeAndReappliesOnEntry)
{
    StartNode(Mode::NRT_JOINT_POSITION);

    auto response = SetForceControlAxis();
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_TRUE(calls_.empty());

    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(calls_, std::vector<std::string> {"force_control_axis"});
}

TEST_F(CartesianConfigServiceTest, HoldsOnAModeChangeButNotOnARejectedValue)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);

    // The robot left the Cartesian mode between the check and the call: held for the next start.
    logic_error_on_ = "force_control_axis";
    auto response = SetForceControlAxis();
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    logic_error_on_.clear();
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(calls_, std::vector<std::string> {"force_control_axis"});

    // std::invalid_argument derives from std::logic_error, but a refused value must not be held,
    // or it would fail every later controller start.
    calls_.clear();
    invalid_argument_on_ = "max_contact_wrench";
    auto rejected = SetMaxContactWrench(10.0);
    ASSERT_NE(rejected, nullptr);
    EXPECT_FALSE(rejected->success);
    invalid_argument_on_.clear();
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(calls_, std::vector<std::string> {"force_control_axis"});
}

TEST_F(CartesianConfigServiceTest, RefusesWhileTheDriverIsNotReady)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);
    status_->driver_state.store(DriverState::FAULT);

    auto response = SetForceControlAxis();
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_TRUE(calls_.empty());

    // A refused request is not held either.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_TRUE(calls_.empty());
}

TEST_F(CartesianConfigServiceTest, RejectsOutOfRangeAndNonFiniteValues)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);

    auto stiffness = std::make_shared<srv::SetCartesianImpedance::Request>();
    stiffness->k_x = {2000, 0, 0, 0, 0, 0};
    auto response = Call<srv::SetCartesianImpedance>("set_cartesian_impedance", stiffness);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);

    EXPECT_FALSE(SetMaxContactWrench(kNaN)->success);
    EXPECT_FALSE(SetMaxContactWrench(-1.0)->success);

    auto short_request = std::make_shared<srv::SetMaxContactWrench::Request>();
    short_request->max_wrench = {10.0};
    EXPECT_FALSE(Call<srv::SetMaxContactWrench>("set_max_contact_wrench", short_request)->success);

    auto posture = std::make_shared<srv::SetNullSpacePosture::Request>();
    posture->ref_positions.assign(kDoF, 0.0);
    posture->ref_positions[3] = 3.0;
    EXPECT_FALSE(Call<srv::SetNullSpacePosture>("set_null_space_posture", posture)->success);

    auto objectives = std::make_shared<srv::SetNullSpaceObjectives::Request>();
    objectives->linear_manipulability = {0.5};
    objectives->angular_manipulability = {0.5};
    objectives->ref_positions_tracking = {0.05};
    EXPECT_FALSE(
        Call<srv::SetNullSpaceObjectives>("set_null_space_objectives", objectives)->success);

    auto frame = std::make_shared<srv::SetForceControlFrame::Request>();
    frame->root_coord = {srv::SetForceControlFrame::Request::TCP};
    frame->t_in_root = {0, 0, 0, 0, 0, 0, 0};
    EXPECT_FALSE(Call<srv::SetForceControlFrame>("set_force_control_frame", frame)->success);

    // Neither delivered nor held.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_TRUE(calls_.empty());
}

TEST_F(CartesianConfigServiceTest, AcceptsInfinityToDisableMaxContactWrench)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);

    auto response = SetMaxContactWrench(kInf);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_EQ(calls_, std::vector<std::string> {"max_contact_wrench"});
}

TEST_F(CartesianConfigServiceTest, EmptyStiffnessMeansNominalAndReturnsIt)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);

    auto response = Call<srv::SetCartesianImpedance>(
        "set_cartesian_impedance", std::make_shared<srv::SetCartesianImpedance::Request>());
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    const std::vector<double> nominal {1000, 1000, 1000, 100, 100, 100};
    EXPECT_EQ(response->k_x_nom, nominal);
    ASSERT_EQ(last_k_x_.size(), 1u);
    EXPECT_EQ(std::vector<double>(last_k_x_[0].begin(), last_k_x_[0].end()), nominal);
}

TEST_F(CartesianConfigServiceTest, PassiveForceControlIsOnlyDeliveredInIdle)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE);

    ASSERT_TRUE(SetPassiveForceControl()->success);
    EXPECT_TRUE(calls_.empty());

    // Reapply() runs after SwitchMode(), where the robot rejects it; only the IDLE hook sends it.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_TRUE(calls_.empty());
    EXPECT_TRUE(node_->ApplyBeforeModeEntry());
    EXPECT_EQ(calls_, std::vector<std::string> {"passive_force_control"});

    calls_.clear();
    status_->control_mode.store(Mode::IDLE);
    ASSERT_TRUE(SetPassiveForceControl()->success);
    EXPECT_EQ(calls_, std::vector<std::string> {"passive_force_control"});
}

TEST_F(CartesianConfigServiceTest, MotionLimitsAreAlwaysTakenAndNeverReapplied)
{
    StartNode(Mode::NRT_JOINT_POSITION);

    auto request = std::make_shared<srv::SetCartesianMotionLimits::Request>();
    request->max_linear_vel = {0.02};
    request->max_angular_vel = {1.0};
    request->max_linear_acc = {2.0};
    request->max_angular_acc = {5.0};
    auto response = Call<srv::SetCartesianMotionLimits>("set_cartesian_motion_limits", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    ASSERT_EQ(last_limits_.size(), 1u);
    EXPECT_DOUBLE_EQ(last_limits_[0].max_linear_vel, 0.02);

    calls_.clear();
    EXPECT_TRUE(node_->Reapply());
    EXPECT_TRUE(calls_.empty());

    request->max_linear_acc = {0.0};
    EXPECT_FALSE(
        Call<srv::SetCartesianMotionLimits>("set_cartesian_motion_limits", request)->success);
}

TEST_F(CartesianConfigServiceTest, ReapplySendsEverySettingInTheRDKExampleOrder)
{
    StartNode(Mode::IDLE);

    auto posture = std::make_shared<srv::SetNullSpacePosture::Request>();
    posture->ref_positions.assign(kDoF, 0.5);
    ASSERT_TRUE(Call<srv::SetNullSpacePosture>("set_null_space_posture", posture)->success);
    auto objectives = std::make_shared<srv::SetNullSpaceObjectives::Request>();
    objectives->linear_manipulability = {0.2};
    objectives->angular_manipulability = {0.2};
    objectives->ref_positions_tracking = {0.5};
    ASSERT_TRUE(
        Call<srv::SetNullSpaceObjectives>("set_null_space_objectives", objectives)->success);
    ASSERT_TRUE(SetMaxContactWrench(10.0)->success);
    ASSERT_TRUE(Call<srv::SetCartesianImpedance>(
        "set_cartesian_impedance", std::make_shared<srv::SetCartesianImpedance::Request>())
                    ->success);
    ASSERT_TRUE(SetForceControlAxis()->success);
    auto frame = std::make_shared<srv::SetForceControlFrame::Request>();
    frame->root_coord = {srv::SetForceControlFrame::Request::TCP};
    ASSERT_TRUE(Call<srv::SetForceControlFrame>("set_force_control_frame", frame)->success);
    EXPECT_TRUE(calls_.empty());

    EXPECT_TRUE(node_->Reapply());
    const std::vector<std::string> expected {"force_control_frame", "force_control_axis",
        "impedance", "max_contact_wrench", "null_space_posture", "null_space_objectives"};
    EXPECT_EQ(calls_, expected);

    // Every setting survives any number of controller restarts.
    calls_.clear();
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(calls_, expected);
}

TEST_F(CartesianConfigServiceTest, ReapplyReportsFailureWhenTheRobotRejectsASetting)
{
    StartNode(Mode::IDLE);
    ASSERT_TRUE(SetForceControlAxis()->success);

    throw_on_ = "force_control_axis";
    EXPECT_FALSE(node_->Reapply());
}

TEST_F(CartesianConfigServiceTest, ARobotPairTakesBothHalves)
{
    StartNode(Mode::NRT_CARTESIAN_MOTION_FORCE, 2);

    ASSERT_TRUE(SetMaxContactWrench(10.0)->success);

    // One robot's worth is not enough for a pair.
    auto request = std::make_shared<srv::SetMaxContactWrench::Request>();
    request->max_wrench.assign(6, 10.0);
    EXPECT_FALSE(Call<srv::SetMaxContactWrench>("set_max_contact_wrench", request)->success);

    auto posture = std::make_shared<srv::SetNullSpacePosture::Request>();
    posture->ref_positions.assign(2 * kDoF, 0.0);
    EXPECT_TRUE(Call<srv::SetNullSpacePosture>("set_null_space_posture", posture)->success);
}

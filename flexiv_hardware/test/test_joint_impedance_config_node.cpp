/**
 * @file test_joint_impedance_config_node.cpp
 * @brief Unit tests for the joint order conversions and the range validation used by the joint
 * impedance interface. Needs no robot connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "flexiv_hardware/joint_impedance_config_node.hpp"

using flexiv_hardware::ConvertDRDKToROSOrder;
using flexiv_hardware::ConvertRDKToROSOrder;
using flexiv_hardware::ConvertROSToDRDKOrder;
using flexiv_hardware::ConvertROSToRDKOrder;
using flexiv_hardware::kMaxDampingRatio;
using flexiv_hardware::kMaxInertiaScale;
using flexiv_hardware::kMinDampingRatio;
using flexiv_hardware::kMinInertiaScale;
using flexiv_hardware::PairJointIndex;
using flexiv_hardware::ValidateJointValues;

namespace {

std::vector<std::string> JointNames(size_t count)
{
    std::vector<std::string> names;
    for (size_t i = 1; i <= count; ++i) {
        names.push_back("joint" + std::to_string(i));
    }
    return names;
}

/** @brief The map the hardware interface builds for a 7-DoF arm with no external axis. */
std::vector<size_t> IdentityMap(size_t count)
{
    std::vector<size_t> map(count);
    for (size_t i = 0; i < count; ++i) {
        map[i] = i;
    }
    return map;
}

/** @brief A contiguous left/right map: the first [left] ROS joints are the left robot's. */
std::vector<PairJointIndex> ContiguousPairMap(int left, int right)
{
    std::vector<PairJointIndex> map;
    for (int i = 0; i < left; ++i) {
        map.push_back({0, i});
    }
    for (int i = 0; i < right; ++i) {
        map.push_back({1, i});
    }
    return map;
}

}

//========================================== VALIDATION ============================================

TEST(JointImpedanceValidation, AcceptsValuesWithinPerJointBounds)
{
    // Deliberately non-uniform, so a per-joint bound is genuinely exercised rather than a single
    // maximum that happens to hold for every joint.
    const std::vector<double> bounds {3000.0, 3000.0, 800.0, 800.0, 100.0, 100.0, 100.0};
    std::string message;
    EXPECT_TRUE(ValidateJointValues({3000.0, 1500.0, 800.0, 0.0, 100.0, 50.0, 25.0}, JointNames(7),
        0.0, bounds, "k_q", message))
        << message;
}

TEST(JointImpedanceValidation, RejectsValueAbovePerJointBound)
{
    const std::vector<double> bounds {3000.0, 3000.0, 800.0, 800.0, 100.0, 100.0, 100.0};
    std::string message;
    // Legal for joints 1 and 2, but above the bound for joint 3.
    EXPECT_FALSE(ValidateJointValues(
        {0.0, 0.0, 900.0, 0.0, 0.0, 0.0, 0.0}, JointNames(7), 0.0, bounds, "k_q", message));
    EXPECT_NE(message.find("joint3"), std::string::npos) << message;
    EXPECT_NE(message.find("k_q"), std::string::npos) << message;
}

TEST(JointImpedanceValidation, RejectsNegativeStiffnessButAcceptsZero)
{
    const std::vector<double> bounds(7, 100.0);
    std::string message;
    EXPECT_FALSE(ValidateJointValues(
        {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, JointNames(7), 0.0, bounds, "k_q", message));

    // Zero is legal: the RDK documents it as making the axis free-floating.
    EXPECT_TRUE(ValidateJointValues(
        std::vector<double>(7, 0.0), JointNames(7), 0.0, bounds, "k_q", message))
        << message;
}

TEST(JointImpedanceValidation, RejectsWrongLengthNamingBothSizes)
{
    const std::vector<double> bounds(7, 100.0);
    std::string message;
    EXPECT_FALSE(ValidateJointValues({1.0, 2.0, 3.0}, JointNames(7), 0.0, bounds, "k_q", message));
    EXPECT_NE(message.find('3'), std::string::npos) << message;
    EXPECT_NE(message.find('7'), std::string::npos) << message;
}

TEST(JointImpedanceValidation, RejectsNonFiniteValues)
{
    const std::vector<double> bounds(7, 100.0);
    std::string message;

    auto with_value_at_index_two = [](double value) {
        std::vector<double> values(7, 1.0);
        values[2] = value;
        return values;
    };

    // A NaN passes both halves of a plain range comparison, so it needs its own check.
    EXPECT_FALSE(
        ValidateJointValues(with_value_at_index_two(std::numeric_limits<double>::quiet_NaN()),
            JointNames(7), 0.0, bounds, "k_q", message));
    EXPECT_NE(message.find("joint3"), std::string::npos) << message;

    EXPECT_FALSE(
        ValidateJointValues(with_value_at_index_two(std::numeric_limits<double>::infinity()),
            JointNames(7), 0.0, bounds, "k_q", message));
}

TEST(JointImpedanceValidation, DampingRatioBoundsAreInclusive)
{
    std::string message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMinDampingRatio), JointNames(7),
        kMinDampingRatio, kMaxDampingRatio, "z_q", message))
        << message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMaxDampingRatio), JointNames(7),
        kMinDampingRatio, kMaxDampingRatio, "z_q", message))
        << message;
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 0.29), JointNames(7), kMinDampingRatio,
        kMaxDampingRatio, "z_q", message));
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 0.81), JointNames(7), kMinDampingRatio,
        kMaxDampingRatio, "z_q", message));
}

TEST(JointImpedanceValidation, InertiaScaleBoundsAreInclusive)
{
    std::string message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMinInertiaScale), JointNames(7),
        kMinInertiaScale, kMaxInertiaScale, "inertia_scales", message))
        << message;
    EXPECT_TRUE(ValidateJointValues(std::vector<double>(7, kMaxInertiaScale), JointNames(7),
        kMinInertiaScale, kMaxInertiaScale, "inertia_scales", message))
        << message;
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 0.74), JointNames(7), kMinInertiaScale,
        kMaxInertiaScale, "inertia_scales", message));
    EXPECT_FALSE(ValidateJointValues(std::vector<double>(7, 1.01), JointNames(7), kMinInertiaScale,
        kMaxInertiaScale, "inertia_scales", message));
}

//===================================== SINGLE ROBOT ORDERING ======================================

TEST(JointOrderSingle, IdentityMapPassesValuesThrough)
{
    const std::vector<double> ros {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    EXPECT_EQ(ConvertROSToRDKOrder(ros, IdentityMap(7)), ros);
}

TEST(JointOrderSingle, ExternalAxisComesFirstInRdkOrder)
{
    // The hardware interface puts external axes at the front of the RDK vector, so a URDF that
    // lists the external axis last maps RDK index 0 to ROS index 7.
    const std::vector<size_t> map {7, 0, 1, 2, 3, 4, 5, 6};
    const std::vector<double> ros {10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 99.0};

    const auto rdk = ConvertROSToRDKOrder(ros, map);
    ASSERT_EQ(rdk.size(), 8u);
    EXPECT_DOUBLE_EQ(rdk[0], 99.0);
    EXPECT_DOUBLE_EQ(rdk[1], 10.0);
    EXPECT_DOUBLE_EQ(rdk[7], 16.0);
}

TEST(JointOrderSingle, PermuteAndGatherRoundTrip)
{
    const std::vector<size_t> map {7, 0, 1, 2, 3, 4, 5, 6};
    const std::vector<double> ros {10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 99.0};
    EXPECT_EQ(ConvertRDKToROSOrder(ConvertROSToRDKOrder(ros, map), map), ros);
}

TEST(JointOrderSingle, ThrowsWhenTheMapExceedsTheValues)
{
    EXPECT_THROW(ConvertROSToRDKOrder({1.0, 2.0}, IdentityMap(7)), std::invalid_argument);
    EXPECT_THROW(ConvertRDKToROSOrder({1.0, 2.0}, IdentityMap(7)), std::invalid_argument);
}

//====================================== ROBOT PAIR ORDERING =======================================

TEST(JointOrderPair, SplitsContiguousLeftRightMap)
{
    const auto map = ContiguousPairMap(7, 7);
    std::vector<double> ros(14);
    for (size_t i = 0; i < ros.size(); ++i) {
        ros[i] = static_cast<double>(i);
    }

    const auto split = ConvertROSToDRDKOrder(
        ros, map, std::vector<double>(7, -1.0), std::vector<double>(7, -1.0));
    ASSERT_EQ(split.first.size(), 7u);
    ASSERT_EQ(split.second.size(), 7u);
    EXPECT_DOUBLE_EQ(split.first[0], 0.0);
    EXPECT_DOUBLE_EQ(split.first[6], 6.0);
    EXPECT_DOUBLE_EQ(split.second[0], 7.0);
    EXPECT_DOUBLE_EQ(split.second[6], 13.0);
}

TEST(JointOrderPair, SplitFollowsTheMapNotThePosition)
{
    // Left and right joints alternating in URDF order: a split that assumed the first half belongs
    // to the left robot would silently swap values between the arms.
    std::vector<PairJointIndex> map;
    for (int i = 0; i < 3; ++i) {
        map.push_back({0, i});
        map.push_back({1, i});
    }
    const std::vector<double> ros {0.0, 100.0, 1.0, 101.0, 2.0, 102.0};

    const auto split = ConvertROSToDRDKOrder(
        ros, map, std::vector<double>(3, -1.0), std::vector<double>(3, -1.0));
    EXPECT_EQ(split.first, std::vector<double>({0.0, 1.0, 2.0}));
    EXPECT_EQ(split.second, std::vector<double>({100.0, 101.0, 102.0}));
}

TEST(JointOrderPair, UnmappedRobotJointsKeepTheirFillValue)
{
    // The AICO2 case: the right robot's two external axes are deliberately left unmapped, so no ROS
    // joint supplies a value for them. They must keep the robot's own nominal value -- a stiffness
    // of 0 there would make those axes free-floating and they would sag.
    std::vector<PairJointIndex> map;
    for (int i = 0; i < 9; ++i) {
        map.push_back({0, i}); // left: 2 external axes + 7 arm joints
    }
    for (int i = 2; i < 9; ++i) {
        map.push_back({1, i}); // right: arm joints only, external axes 0 and 1 unmapped
    }
    ASSERT_EQ(map.size(), 16u);

    const std::vector<double> nominal_left(9, 3000.0);
    const std::vector<double> nominal_right(9, 2500.0);
    const std::vector<double> ros(16, 10.0);

    const auto split = ConvertROSToDRDKOrder(ros, map, nominal_left, nominal_right);
    ASSERT_EQ(split.second.size(), 9u);
    EXPECT_DOUBLE_EQ(split.second[0], 2500.0);
    EXPECT_DOUBLE_EQ(split.second[1], 2500.0);
    EXPECT_DOUBLE_EQ(split.second[2], 10.0);
}

TEST(JointOrderPair, SplitAndGatherRoundTrip)
{
    const auto map = ContiguousPairMap(7, 7);
    std::vector<double> ros(14);
    for (size_t i = 0; i < ros.size(); ++i) {
        ros[i] = static_cast<double>(i) + 0.5;
    }

    const auto split
        = ConvertROSToDRDKOrder(ros, map, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0));
    EXPECT_EQ(ConvertDRDKToROSOrder(split.first, split.second, map), ros);
}

TEST(JointOrderPair, ThrowsOnSizeMismatch)
{
    const auto map = ContiguousPairMap(7, 7);
    EXPECT_THROW(ConvertROSToDRDKOrder(
                     {1.0, 2.0}, map, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0)),
        std::invalid_argument);

    // A joint mapped past the end of its robot's vector.
    const std::vector<PairJointIndex> bad_map {{0, 9}};
    EXPECT_THROW(ConvertROSToDRDKOrder(
                     {1.0}, bad_map, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0)),
        std::invalid_argument);
}

//======================================= SERVICE BEHAVIOUR ========================================

namespace {

using flexiv_hardware::DriverState;
using flexiv_hardware::DriverStatus;
using flexiv_hardware::JointImpedanceBounds;
using flexiv_hardware::JointImpedanceConfigNode;
using flexiv_hardware::JointImpedanceSetters;
using flexiv_hardware::SanitizeNamespace;

constexpr size_t kDoF = 7;

/**
 * @brief Spins a JointImpedanceConfigNode against stub setters, so the service handlers and
 * Reapply() are exercised without a robot. The stubs count calls, which is what the re-apply tests
 * assert on.
 */
class JointImpedanceServiceTest : public ::testing::Test
{
protected:
    // Guarded, and deliberately not shut down per suite: gtest runs these once per repetition, and
    // re-initializing rclcpp in a process that has already shut it down crashes.
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

    void StartNode(bool impedance_mode_configured, flexiv::rdk::Mode mode)
    {
        status_ = std::make_shared<DriverStatus>();
        status_->connected.store(true);
        status_->driver_state.store(DriverState::READY);
        status_->control_mode.store(mode);

        JointImpedanceBounds bounds;
        bounds.k_q_nom = std::vector<double>(kDoF, 100.0);
        bounds.tau_max = std::vector<double>(kDoF, 50.0);

        JointImpedanceSetters setters;
        setters.set_joint_impedance
            = [this](const std::vector<double>& k_q, const std::vector<double>& z_q) {
                  if (throw_on_impedance_) {
                      throw std::runtime_error("stub: failed to deliver the request");
                  }
                  ++impedance_calls_;
                  last_k_q_ = k_q;
                  last_z_q_ = z_q;
              };
        setters.set_max_contact_torque
            = [this](const std::vector<double>&) { ++contact_torque_calls_; };
        setters.set_joint_inertia_scale = [this](const std::vector<double>&) { ++inertia_calls_; };

        // A serial number unique to this test, so no two tests ever share a node namespace. Reusing
        // one name across nodes that are created and destroyed in quick succession makes service
        // discovery racy.
        static int instance = 0;
        const std::string robot_sn = "Rizon4-90000" + std::to_string(++instance);
        namespace_ = flexiv_hardware::SanitizeNamespace(robot_sn);

        node_ = std::make_shared<JointImpedanceConfigNode>(
            robot_sn, JointNames(kDoF), bounds, impedance_mode_configured, status_, setters);
        client_node_ = std::make_shared<rclcpp::Node>("joint_impedance_test_client");

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_->add_node(client_node_);

        // spin_some in a loop rather than spin(): a test whose body finishes before the thread
        // reaches spin() would cancel it first, and spin() would then never return.
        running_.store(true);
        spin_thread_ = std::thread([this]() {
            while (running_.load()) {
                executor_->spin_some(std::chrono::milliseconds(5));
                // Yields the core: a tight spin_some loop starves the discovery threads, which
                // shows up as a service that never appears.
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
        // The executor holds a reference to both nodes, so it has to go first. Leaving it would
        // keep the previous test's node -- and its identically named services -- alive, and the
        // next test's client could bind to those instead.
        if (executor_) {
            executor_->remove_node(node_);
            executor_->remove_node(client_node_);
        }
        executor_.reset();
        client_node_.reset();
        node_.reset();
    }

    std::string Endpoint(const std::string& name) const
    {
        return "/" + namespace_ + "/flexiv_joint_impedance_config_node/" + name;
    }

    /** @brief Call one of the services and return the response, failing the test on a timeout. */
    template <typename Service>
    typename Service::Response::SharedPtr Call(
        const std::string& name, typename Service::Request::SharedPtr request)
    {
        auto client = client_node_->create_client<Service>(Endpoint(name));
        if (!client->wait_for_service(std::chrono::seconds(10))) {
            ADD_FAILURE() << "service " << Endpoint(name) << " never appeared";
            return nullptr;
        }
        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            ADD_FAILURE() << "service " << Endpoint(name) << " did not respond";
            return nullptr;
        }
        return future.get();
    }

    std::string namespace_;
    std::shared_ptr<DriverStatus> status_;
    std::shared_ptr<JointImpedanceConfigNode> node_;
    std::shared_ptr<rclcpp::Node> client_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    std::atomic<bool> running_ {false};

    bool throw_on_impedance_ = false;
    int impedance_calls_ = 0;
    int contact_torque_calls_ = 0;
    int inertia_calls_ = 0;
    std::vector<double> last_k_q_;
    std::vector<double> last_z_q_;
};

flexiv_msgs::srv::SetJointImpedance::Request::SharedPtr StiffnessRequest(double value)
{
    auto request = std::make_shared<flexiv_msgs::srv::SetJointImpedance::Request>();
    request->k_q = std::vector<double>(kDoF, value);
    return request;
}

}

TEST_F(JointImpedanceServiceTest, AppliesWhileInAnImpedanceMode)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);

    auto response
        = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0));
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_TRUE(response->setting.in_effect);
    EXPECT_EQ(impedance_calls_, 1);
    EXPECT_EQ(last_k_q_, std::vector<double>(kDoF, 40.0));

    // An empty z_q must reach the RDK as the nominal ratio, not as an empty vector.
    EXPECT_EQ(last_z_q_, std::vector<double>(kDoF, flexiv_hardware::kNominalDampingRatio));
}

TEST_F(JointImpedanceServiceTest, RefusesWhenTheDriverIsInJointPositionMode)
{
    StartNode(false, flexiv::rdk::Mode::NRT_JOINT_POSITION);

    auto response
        = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0));
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("joint_impedance"), std::string::npos) << response->message;
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, HoldsTheRequestWhileTheRobotIsIdle)
{
    // Where the robot sits between activation and the first controller start, and after a fault.
    StartNode(true, flexiv::rdk::Mode::IDLE);

    auto response
        = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0));
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_FALSE(response->setting.in_effect);
    EXPECT_EQ(impedance_calls_, 0);
    EXPECT_EQ(response->setting.k_q, std::vector<double>(kDoF, 40.0));

    // The held request is delivered by the controller start that follows.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(impedance_calls_, 1);
}

TEST_F(JointImpedanceServiceTest, RefusesWhileTheRobotIsFaulted)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);
    status_->driver_state.store(DriverState::FAULT);

    auto response
        = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0));
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(impedance_calls_, 0);

    // Refused, not held: a later controller start must not apply it silently.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, RefusesWhileTheRobotIsInAReducedState)
{
    // Reduced state leaves the driver state at READY, so it needs its own check.
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);
    status_->reduced.store(true);

    auto response
        = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0));
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, RejectsAnOutOfRangeRequestWithoutTouchingTheRobot)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);

    // Above the k_q_nom of 100.0 the fixture reports.
    auto response = Call<flexiv_msgs::srv::SetJointImpedance>(
        "set_joint_impedance", StiffnessRequest(4000.0));
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(impedance_calls_, 0);
    // The held values are unchanged, so they still report the nominal the node started from.
    EXPECT_EQ(response->setting.k_q, std::vector<double>(kDoF, 100.0));
}

TEST_F(JointImpedanceServiceTest, ReapplySendsNothingWhenNothingWasEverSet)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);

    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(impedance_calls_, 0);
    EXPECT_EQ(contact_torque_calls_, 0);
    EXPECT_EQ(inertia_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, ReapplySendsOnlyThePropertiesThatWereSet)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointInertiaScale::Request>();
    request->inertia_scales = std::vector<double>(kDoF, 0.8);
    auto response
        = Call<flexiv_msgs::srv::SetJointInertiaScale>("set_joint_inertia_scale", request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_EQ(inertia_calls_, 1);

    // Mode entry resets every property to nominal, so the two nobody touched are already where they
    // need to be and must not cost a blocking call.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(inertia_calls_, 2);
    EXPECT_EQ(impedance_calls_, 0);
    EXPECT_EQ(contact_torque_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, ReapplySendsEveryPropertyThatWasEverSet)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);

    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0))
            ->success);

    auto torque_request = std::make_shared<flexiv_msgs::srv::SetMaxContactTorque::Request>();
    torque_request->max_contact_torques = std::vector<double>(kDoF, 10.0);
    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetMaxContactTorque>("set_max_contact_torque", torque_request)
            ->success);

    ASSERT_EQ(impedance_calls_, 1);
    ASSERT_EQ(contact_torque_calls_, 1);

    // Both come back, not just the one that was set most recently.
    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(impedance_calls_, 2);
    EXPECT_EQ(contact_torque_calls_, 2);
    EXPECT_EQ(inertia_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, ReapplyReportsFailureWhenTheRobotRejectsTheSetting)
{
    StartNode(true, flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE);
    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0))
            ->success);

    // The controller start that follows a failed re-apply has to be refused, so this must be false
    // rather than throwing into the control loop.
    node_->MarkNotInEffect();
    throw_on_impedance_ = true;
    EXPECT_FALSE(node_->Reapply());
}

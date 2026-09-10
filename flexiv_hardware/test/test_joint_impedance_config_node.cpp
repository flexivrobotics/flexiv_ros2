/**
 * @file test_joint_impedance_config_node.cpp
 * @brief Unit tests for the joint order conversions and the range validation used by the joint
 * impedance interface. Needs no robot connection.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <algorithm>
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

using flexiv_hardware::ConvertRDKToROSOrder;
using flexiv_hardware::ConvertROSToRDKOrder;
using flexiv_hardware::ResolveJointSelection;
using flexiv_hardware::kMaxDampingRatio;
using flexiv_hardware::kMaxInertiaScale;
using flexiv_hardware::kMinDampingRatio;
using flexiv_hardware::kMinInertiaScale;
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

TEST(JointOrderSingle, RdkOrderNeedNotMatchUrdfOrder)
{
    // The impedance joint list is in URDF order while the RDK vector follows the joint groups, so
    // a URDF that interleaves the two arms permutes rather than passes through. Here ROS index 0
    // is the first joint of the second RDK group.
    const std::vector<size_t> map {1, 3, 5, 0, 2, 4};
    const std::vector<double> ros {10.0, 11.0, 12.0, 13.0, 14.0, 15.0};

    const auto rdk = ConvertROSToRDKOrder(ros, map);
    ASSERT_EQ(rdk.size(), 6u);
    EXPECT_DOUBLE_EQ(rdk[0], 11.0);
    EXPECT_DOUBLE_EQ(rdk[3], 10.0);
    EXPECT_DOUBLE_EQ(rdk[5], 14.0);
}

TEST(JointOrderSingle, PermuteAndGatherRoundTrip)
{
    const std::vector<size_t> map {1, 3, 5, 0, 2, 4};
    const std::vector<double> ros {10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
    EXPECT_EQ(ConvertRDKToROSOrder(ConvertROSToRDKOrder(ros, map), map), ros);
}

TEST(JointOrderSingle, ThrowsWhenTheMapExceedsTheValues)
{
    EXPECT_THROW(ConvertROSToRDKOrder({1.0, 2.0}, IdentityMap(7)), std::invalid_argument);
    EXPECT_THROW(ConvertRDKToROSOrder({1.0, 2.0}, IdentityMap(7)), std::invalid_argument);
}

//======================================== JOINT SELECTION =========================================

TEST(JointSelection, AnEmptyRequestAddressesEveryJoint)
{
    std::vector<size_t> indices;
    std::string message;
    ASSERT_TRUE(ResolveJointSelection({}, JointNames(4), indices, message)) << message;
    EXPECT_EQ(indices, std::vector<size_t>({0, 1, 2, 3}));
}

TEST(JointSelection, ResolvesNamedJointsInRequestOrder)
{
    // The dual-arm case: one arm named, the other left out entirely.
    const std::vector<std::string> covered {
        "left_joint1", "left_joint2", "right_joint1", "right_joint2"};
    std::vector<size_t> indices;
    std::string message;
    ASSERT_TRUE(ResolveJointSelection({"right_joint2", "left_joint1"}, covered, indices, message))
        << message;
    EXPECT_EQ(indices, std::vector<size_t>({3, 0}));
}

TEST(JointSelection, RejectsAJointTheInterfaceDoesNotCover)
{
    // The MICO torso case: a real robot joint, but not one this interface can set.
    std::vector<size_t> indices;
    std::string message;
    EXPECT_FALSE(ResolveJointSelection({"torso_joint1"}, JointNames(4), indices, message));
    EXPECT_NE(message.find("torso_joint1"), std::string::npos) << message;
    EXPECT_NE(message.find("joint_names"), std::string::npos) << message;
}

TEST(JointSelection, RejectsADuplicatedJoint)
{
    // Two values for one joint: whichever won would be arbitrary, so refuse instead.
    std::vector<size_t> indices;
    std::string message;
    EXPECT_FALSE(ResolveJointSelection({"joint2", "joint2"}, JointNames(4), indices, message));
    EXPECT_NE(message.find("joint2"), std::string::npos) << message;
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
        setters.set_joint_impedance = [this](const std::vector<double>& k_q,
                                          const std::vector<double>& z_q,
                                          const flexiv_hardware::JointMask& touched) {
            if (throw_on_impedance_) {
                throw std::runtime_error("stub: failed to deliver the request");
            }
            ++impedance_calls_;
            last_k_q_ = k_q;
            last_z_q_ = z_q;
            last_touched_ = touched;
        };
        setters.set_max_contact_torque
            = [this](const std::vector<double>& max_torques,
                  const flexiv_hardware::JointMask& touched) {
                  ++contact_torque_calls_;
                  last_max_contact_torques_ = max_torques;
                  last_touched_ = touched;
              };
        setters.set_joint_inertia_scale
            = [this](const std::vector<double>& inertia_scales,
                  const flexiv_hardware::JointMask& touched) {
                  ++inertia_calls_;
                  last_inertia_scales_ = inertia_scales;
                  last_touched_ = touched;
              };

        // A serial number unique to this test, so no two tests ever share a node namespace. Reusing
        // one name across nodes that are created and destroyed in quick succession makes service
        // discovery racy.
        static int instance = 0;
        const std::string robot_sn = "Enlight-L-90000" + std::to_string(++instance);
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
    std::vector<double> last_max_contact_torques_;
    std::vector<double> last_inertia_scales_;
    flexiv_hardware::JointMask last_touched_;
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
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

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
    StartNode(false, flexiv::rdk::Mode::RT_JOINT_POSITION);

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
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);
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
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);
    status_->reduced.store(true);

    auto response
        = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0));
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, RejectsAnOutOfRangeRequestWithoutTouchingTheRobot)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

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
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    EXPECT_TRUE(node_->Reapply());
    EXPECT_EQ(impedance_calls_, 0);
    EXPECT_EQ(contact_torque_calls_, 0);
    EXPECT_EQ(inertia_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, ReapplySendsOnlyThePropertiesThatWereSet)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

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
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

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

TEST_F(JointImpedanceServiceTest, SetsOnlyTheNamedJointsAndLeavesTheRestAlone)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    // Deliver once first, so the robot already holds what the node holds and the partial delivery
    // below is not widened to every joint.
    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(90.0))
            ->success);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointImpedance::Request>();
    request->joint_names = {"joint1", "joint2"};
    request->k_q = {10.0, 20.0};

    auto response = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success) << response->message;

    // The response reports every covered joint: the two named ones changed, the rest did not.
    ASSERT_EQ(response->setting.k_q.size(), kDoF);
    EXPECT_DOUBLE_EQ(response->setting.k_q[0], 10.0);
    EXPECT_DOUBLE_EQ(response->setting.k_q[1], 20.0);
    EXPECT_DOUBLE_EQ(response->setting.k_q[2], 90.0);
    EXPECT_DOUBLE_EQ(response->setting.k_q[6], 90.0);

    // The RDK sets a whole joint group at a time, so the merged full-length vector is delivered.
    EXPECT_EQ(last_k_q_, response->setting.k_q);

    // Only the named joints are marked, which is what lets the hardware interface skip the joint
    // group the request did not touch.
    ASSERT_EQ(last_touched_.size(), kDoF);
    EXPECT_TRUE(last_touched_[0]);
    EXPECT_TRUE(last_touched_[1]);
    EXPECT_FALSE(last_touched_[2]);
    EXPECT_FALSE(last_touched_[6]);
}

TEST_F(JointImpedanceServiceTest, TheFirstDeliveryCoversEveryJointEvenWhenPartial)
{
    // Nothing has reached the robot yet, so a request naming one joint must still deliver the
    // values held for the others: skipping their joint group would leave it at nominal.
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointImpedance::Request>();
    request->joint_names = {"joint1"};
    request->k_q = {10.0};

    auto response = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;

    ASSERT_EQ(last_touched_.size(), kDoF);
    EXPECT_EQ(
        std::count(last_touched_.begin(), last_touched_.end(), true), static_cast<long>(kDoF));
}

TEST_F(JointImpedanceServiceTest, ValidatesNamedJointsAgainstTheirOwnBounds)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointImpedance::Request>();
    request->joint_names = {"joint3"};
    request->k_q = {4000.0}; // Above the k_q_nom of 100.0 the fixture reports.

    auto response = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", request);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("joint3"), std::string::npos) << response->message;
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, RejectsValuesThatDoNotMatchTheNamedJoints)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointImpedance::Request>();
    request->joint_names = {"joint1", "joint2"};
    request->k_q = {10.0};

    auto response = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", request);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, RejectsAJointNameTheInterfaceDoesNotCover)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointImpedance::Request>();
    request->joint_names = {"torso_joint1"};
    request->k_q = {10.0};

    auto response = Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", request);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("torso_joint1"), std::string::npos) << response->message;
    EXPECT_EQ(impedance_calls_, 0);
}

TEST_F(JointImpedanceServiceTest, PartialMaxContactTorqueMergesIntoTheHeldValues)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    auto full = std::make_shared<flexiv_msgs::srv::SetMaxContactTorque::Request>();
    full->max_contact_torques = std::vector<double>(kDoF, 20.0);
    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetMaxContactTorque>("set_max_contact_torque", full)->success);

    auto request = std::make_shared<flexiv_msgs::srv::SetMaxContactTorque::Request>();
    request->joint_names = {"joint7"};
    request->max_contact_torques = {5.0};

    auto response = Call<flexiv_msgs::srv::SetMaxContactTorque>("set_max_contact_torque", request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_EQ(response->setting.max_contact_torques.size(), kDoF);
    EXPECT_DOUBLE_EQ(response->setting.max_contact_torques[6], 5.0);
    EXPECT_DOUBLE_EQ(response->setting.max_contact_torques[0], 20.0);
    EXPECT_EQ(last_max_contact_torques_, response->setting.max_contact_torques);
}

TEST_F(JointImpedanceServiceTest, ReapplyAfterAPartialRequestCoversEveryJoint)
{
    // Mode entry resets every joint group, so a re-apply must not inherit the narrow mask of the
    // last partial request.
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);

    auto request = std::make_shared<flexiv_msgs::srv::SetJointInertiaScale::Request>();
    request->joint_names = {"joint1"};
    request->inertia_scales = {0.8};
    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetJointInertiaScale>("set_joint_inertia_scale", request)->success);

    ASSERT_TRUE(node_->Reapply());
    ASSERT_EQ(last_touched_.size(), kDoF);
    EXPECT_EQ(
        std::count(last_touched_.begin(), last_touched_.end(), true), static_cast<long>(kDoF));
    // The joint nobody named keeps the nominal scale the node started from.
    EXPECT_DOUBLE_EQ(last_inertia_scales_[0], 0.8);
    EXPECT_DOUBLE_EQ(last_inertia_scales_[1], flexiv_hardware::kNominalInertiaScale);
}

TEST_F(JointImpedanceServiceTest, ReapplyReportsFailureWhenTheRobotRejectsTheSetting)
{
    StartNode(true, flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);
    ASSERT_TRUE(
        Call<flexiv_msgs::srv::SetJointImpedance>("set_joint_impedance", StiffnessRequest(40.0))
            ->success);

    // The controller start that follows a failed re-apply has to be refused, so this must be false
    // rather than throwing into the control loop.
    node_->MarkNotInEffect();
    throw_on_impedance_ = true;
    EXPECT_FALSE(node_->Reapply());
}

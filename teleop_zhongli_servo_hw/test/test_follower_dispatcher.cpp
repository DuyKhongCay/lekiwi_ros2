/**
 * @file test_follower_dispatcher.cpp
 * @brief Unit tests for FollowerDispatcher (Streaming Teleop, LPF, Mode Dispatching).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "teleop_zhongli_servo_hw/follower_dispatcher.hpp"

using namespace teleop_zhongli_servo_hw;

class FollowerDispatcherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_follower_dispatcher_node");
  }

  rclcpp::Node::SharedPtr test_node_;
};

TEST_F(FollowerDispatcherTest, NullNodeThrowsInvalidArgument)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  EXPECT_THROW(dispatcher.init(nullptr, cfg), std::invalid_argument);
}

TEST_F(FollowerDispatcherTest, NonPositivePointDtClamping)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  cfg.point_dt_s = -0.05;

  dispatcher.init(test_node_.get(), cfg);
  EXPECT_DOUBLE_EQ(dispatcher.get_config().point_dt_s, 0.08);

  DispatcherConfig cfg_zero;
  cfg_zero.point_dt_s = 0.0;
  dispatcher.init(test_node_.get(), cfg_zero);
  EXPECT_DOUBLE_EQ(dispatcher.get_config().point_dt_s, 0.08);
}

TEST_F(FollowerDispatcherTest, StreamingTrajectoryTimestampZeroAndVelocities)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  cfg.arm_mode = "joint_trajectory";
  cfg.follower_jtc_topic = "/test/joint_trajectory_vel";
  cfg.follower_joint_names = {"joint_1", "joint_2"};
  cfg.point_dt_s = 0.08;
  cfg.lpf_alpha = 1.0;

  dispatcher.init(test_node_.get(), cfg);
  EXPECT_EQ(dispatcher.get_mode(), DispatchMode::JOINT_TRAJECTORY);

  trajectory_msgs::msg::JointTrajectory received_msg;
  bool message_received = false;

  auto sub = test_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      cfg.follower_jtc_topic, rclcpp::QoS(10).reliable(),
      [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        received_msg = *msg;
        message_received = true;
      });

  // Sample 1: Initial position -> initial velocity must be 0.0
  dispatcher.dispatch({0.0, 0.0});

  const auto start = std::chrono::steady_clock::now();
  while (!message_received && (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500))
  {
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(message_received);
  EXPECT_EQ(received_msg.header.stamp.sec, 0);
  EXPECT_EQ(received_msg.header.stamp.nanosec, 0u);
  ASSERT_EQ(received_msg.points.size(), 1u);
  EXPECT_EQ(received_msg.points[0].time_from_start.sec, 0);
  EXPECT_EQ(received_msg.points[0].time_from_start.nanosec, 80000000u); // 80ms lookahead
  ASSERT_EQ(received_msg.points[0].positions.size(), 2u);
  ASSERT_EQ(received_msg.points[0].velocities.size(), 2u);
  EXPECT_DOUBLE_EQ(received_msg.points[0].velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(received_msg.points[0].velocities[1], 0.0);

  // Sample 2: Movement after delay -> estimated velocity must be positive and non-zero
  message_received = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  dispatcher.dispatch({0.05, 0.10});

  const auto start2 = std::chrono::steady_clock::now();
  while (!message_received && (std::chrono::steady_clock::now() - start2) < std::chrono::milliseconds(500))
  {
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(message_received);
  ASSERT_EQ(received_msg.points.size(), 1u);
  ASSERT_EQ(received_msg.points[0].velocities.size(), 2u);
  EXPECT_GT(received_msg.points[0].velocities[0], 0.0);
  EXPECT_GT(received_msg.points[0].velocities[1], 0.0);
  EXPECT_NEAR(received_msg.points[0].velocities[1], 2.0 * received_msg.points[0].velocities[0], 0.1);

  // Reset clears velocity history: next sample should reset velocity to 0.0
  dispatcher.reset();
  message_received = false;
  dispatcher.dispatch({1.0, 2.0});

  const auto start3 = std::chrono::steady_clock::now();
  while (!message_received && (std::chrono::steady_clock::now() - start3) < std::chrono::milliseconds(500))
  {
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(message_received);
  ASSERT_EQ(received_msg.points[0].velocities.size(), 2u);
  EXPECT_DOUBLE_EQ(received_msg.points[0].velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(received_msg.points[0].velocities[1], 0.0);
}

TEST_F(FollowerDispatcherTest, ForwardPositionModeDispatching)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  cfg.arm_mode = "forward_position";
  cfg.follower_fwd_topic = "/test/forward_commands";
  cfg.follower_joint_names = {"joint_1"};
  cfg.lpf_alpha = 1.0;

  dispatcher.init(test_node_.get(), cfg);
  EXPECT_EQ(dispatcher.get_mode(), DispatchMode::FORWARD_POSITION);

  std_msgs::msg::Float64MultiArray received_msg;
  bool message_received = false;

  auto sub = test_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      cfg.follower_fwd_topic, rclcpp::QoS(10).reliable(),
      [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        received_msg = *msg;
        message_received = true;
      });

  std::vector<double> positions = {1.234};
  dispatcher.dispatch(positions);

  const auto start = std::chrono::steady_clock::now();
  while (!message_received && (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500))
  {
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(message_received);
  ASSERT_EQ(received_msg.data.size(), 1u);
  EXPECT_NEAR(received_msg.data[0], 1.234, 1e-6);
}

TEST_F(FollowerDispatcherTest, JointStatesOnlyModePublishesNothing)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  cfg.arm_mode = "joint_states_only";
  cfg.follower_jtc_topic = "/test/joint_states_only_jtc";
  cfg.follower_fwd_topic = "/test/joint_states_only_fwd";

  dispatcher.init(test_node_.get(), cfg);
  EXPECT_EQ(dispatcher.get_mode(), DispatchMode::JOINT_STATES_ONLY);

  bool received_any = false;
  auto sub_jtc = test_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      cfg.follower_jtc_topic, rclcpp::QoS(10).reliable(),
      [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr) { received_any = true; });

  auto sub_fwd = test_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      cfg.follower_fwd_topic, rclcpp::QoS(10).reliable(),
      [&](const std_msgs::msg::Float64MultiArray::SharedPtr) { received_any = true; });

  dispatcher.dispatch({0.1, 0.2});

  for (int i = 0; i < 5; ++i)
  {
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_FALSE(received_any);
}

TEST_F(FollowerDispatcherTest, LowPassFilterSmoothingAndReset)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  cfg.arm_mode = "forward_position";
  cfg.lpf_alpha = 0.5;

  dispatcher.init(test_node_.get(), cfg);
  EXPECT_FALSE(dispatcher.is_lpf_initialized());

  // Step 1: First sample must initialize LPF directly (no step lag)
  dispatcher.dispatch({1.0});
  EXPECT_TRUE(dispatcher.is_lpf_initialized());
  ASSERT_EQ(dispatcher.get_filtered_positions().size(), 1u);
  EXPECT_NEAR(dispatcher.get_filtered_positions()[0], 1.0, 1e-6);

  // Step 2: Second sample smoothed with alpha=0.5: 0.5 * 2.0 + 0.5 * 1.0 = 1.5
  dispatcher.dispatch({2.0});
  EXPECT_NEAR(dispatcher.get_filtered_positions()[0], 1.5, 1e-6);

  // Step 3: Third sample: 0.5 * 2.0 + 0.5 * 1.5 = 1.75
  dispatcher.dispatch({2.0});
  EXPECT_NEAR(dispatcher.get_filtered_positions()[0], 1.75, 1e-6);

  // Step 4: Safety reset on disconnect
  dispatcher.reset();
  EXPECT_FALSE(dispatcher.is_lpf_initialized());
  EXPECT_TRUE(dispatcher.get_filtered_positions().empty());

  // Step 5: Post-reset sample re-initializes cleanly to new position (no jump from 1.75)
  dispatcher.dispatch({0.0});
  EXPECT_TRUE(dispatcher.is_lpf_initialized());
  EXPECT_NEAR(dispatcher.get_filtered_positions()[0], 0.0, 1e-6);
}

TEST_F(FollowerDispatcherTest, EmptyInputSafeNoPublish)
{
  FollowerDispatcher dispatcher;
  DispatcherConfig cfg;
  cfg.arm_mode = "forward_position";
  dispatcher.init(test_node_.get(), cfg);

  // Empty input should safely return early
  dispatcher.dispatch({});
  EXPECT_TRUE(dispatcher.get_filtered_positions().empty());
}


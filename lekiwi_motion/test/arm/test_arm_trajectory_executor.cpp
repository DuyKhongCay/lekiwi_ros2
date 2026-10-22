// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test_arm_trajectory_executor.cpp
 * @brief Unit tests for ArmTrajectoryExecutor and TrajectoryExecutionGuard.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <chrono>
#include <fstream>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "lekiwi_motion/arm/arm_trajectory_executor.hpp"

namespace lekiwi_motion::arm::test
{

class ArmTrajectoryExecutorTest : public ::testing::Test
{
protected:
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

  static std::string load_test_urdf()
  {
#ifdef WORKSPACE_URDF_PATH
    std::ifstream file(WORKSPACE_URDF_PATH);
    if (file.is_open()) {
      std::stringstream buffer;
      buffer << file.rdbuf();
      return buffer.str();
    }
#endif
    return "";
  }
};

TEST_F(ArmTrajectoryExecutorTest, TrajectoryExecutionGuardEnforcesMutualExclusionAndMoveSemantics)
{
  std::atomic<bool> flag{false};

  {
    TrajectoryExecutionGuard g1(flag);
    EXPECT_TRUE(g1.acquired());
    EXPECT_TRUE(flag.load());

    // Concurrent acquisition attempt must fail
    TrajectoryExecutionGuard g2(flag);
    EXPECT_FALSE(g2.acquired());
    EXPECT_TRUE(flag.load());

    // Move construct: transfers ownership of lock
    TrajectoryExecutionGuard g3(std::move(g1));
    EXPECT_FALSE(g1.acquired());
    EXPECT_TRUE(g3.acquired());
    EXPECT_TRUE(flag.load());

    // Move assignment
    TrajectoryExecutionGuard g4(flag);
    EXPECT_FALSE(g4.acquired());
    g4 = std::move(g3);
    EXPECT_FALSE(g3.acquired());
    EXPECT_TRUE(g4.acquired());
    EXPECT_TRUE(flag.load());
  }

  // Once all guards out of scope, flag is unlocked
  EXPECT_FALSE(flag.load());

  TrajectoryExecutionGuard g5(flag);
  EXPECT_TRUE(g5.acquired());
  EXPECT_TRUE(flag.load());
}

TEST_F(ArmTrajectoryExecutorTest, DefaultAndCustomNamedPosesLoadedFromParameters)
{
  auto node = std::make_shared<rclcpp::Node>("test_executor_defaults");
  ArmTrajectoryExecutorConfig cfg;
  cfg.mock_execution = true;
  ArmTrajectoryExecutor executor(node.get(), cfg);

  EXPECT_TRUE(executor.get_named_pose("approach").has_value());
  EXPECT_TRUE(executor.get_named_pose("stow").has_value());
  EXPECT_TRUE(executor.get_named_pose("clear_left").has_value());
  EXPECT_TRUE(executor.get_named_pose("clear_right").has_value());
  EXPECT_FALSE(executor.get_named_pose("non_existent").has_value());

  // Test custom named pose override from Node parameter
  rclcpp::NodeOptions opts;
  const std::vector<double> custom_stow{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  opts.append_parameter_override("named_poses.stow", custom_stow);

  auto node_custom = std::make_shared<rclcpp::Node>("test_executor_custom", opts);
  ArmTrajectoryExecutor executor_custom(node_custom.get(), cfg);

  auto stow_opt = executor_custom.get_named_pose("stow");
  ASSERT_TRUE(stow_opt.has_value());
  EXPECT_EQ(*stow_opt, custom_stow);
}

TEST_F(ArmTrajectoryExecutorTest, ResolveIkUninitializedReturnsNullopt)
{
  auto node = std::make_shared<rclcpp::Node>("test_ik_uninitialized");
  ArmTrajectoryExecutorConfig cfg;
  ArmTrajectoryExecutor executor(node.get(), cfg);

  geometry_msgs::msg::Point pt;
  pt.x = 0.15;
  pt.y = 0.0;
  pt.z = 0.05;

  auto sol = executor.resolve_point_ik(pt, "base_footprint", 1.50);
  EXPECT_FALSE(sol.has_value());
}

TEST_F(ArmTrajectoryExecutorTest, ResolveIkWithKinematicsSolvedInBaseFrame)
{
  const std::string urdf = load_test_urdf();
  ASSERT_FALSE(urdf.empty()) << "Fixture URDF must be present";

  rclcpp::NodeOptions opts;
  opts.append_parameter_override("robot_description", urdf);
  auto node = std::make_shared<rclcpp::Node>("test_ik_solved", opts);

  ArmTrajectoryExecutorConfig cfg;
  ArmTrajectoryExecutor executor(node.get(), cfg);

  EXPECT_TRUE(executor.is_kinematics_initialized());

  geometry_msgs::msg::Point pt;
  pt.x = 0.20;
  pt.y = 0.0;
  pt.z = 0.10;

  auto sol = executor.resolve_point_ik(pt, "base_footprint", 0.0);
  ASSERT_TRUE(sol.has_value());
  EXPECT_EQ(sol->size(), 6u);
  EXPECT_DOUBLE_EQ((*sol)[5], 0.0);
}

TEST_F(ArmTrajectoryExecutorTest, TrajectoryExecutionGuardEnforcesMutualExclusion)
{
  std::atomic<bool> busy_flag{false};
  {
    TrajectoryExecutionGuard guard1(busy_flag);
    EXPECT_TRUE(guard1.acquired());
    EXPECT_TRUE(busy_flag.load());

    // Second guard attempts to acquire while first holds it
    TrajectoryExecutionGuard guard2(busy_flag);
    EXPECT_FALSE(guard2.acquired());
    EXPECT_TRUE(busy_flag.load());
  }
  // After guard1 goes out of scope, busy flag must be cleared
  EXPECT_FALSE(busy_flag.load());

  TrajectoryExecutionGuard guard3(busy_flag);
  EXPECT_TRUE(guard3.acquired());
  EXPECT_TRUE(busy_flag.load());
}

TEST_F(ArmTrajectoryExecutorTest, MockExecutionUpdatesJointPositionsAndRejectsConcurrentCalls)
{
  const std::string urdf = load_test_urdf();
  ASSERT_FALSE(urdf.empty()) << "Fixture URDF must be present";

  rclcpp::NodeOptions opts;
  opts.append_parameter_override("robot_description", urdf);
  opts.append_parameter_override("mock_execution", true);
  opts.append_parameter_override("default_velocity", 10.0);
  auto node = std::make_shared<rclcpp::Node>("test_mock_execution", opts);

  ArmTrajectoryExecutorConfig cfg;
  cfg.mock_execution = true;
  ArmTrajectoryExecutor executor(node.get(), cfg);

  const std::vector<double> target_joints{0.1, -0.2, 0.3, -0.4, 0.5, 0.0};
  auto [ok, msg] = executor.execute_joint_target(target_joints, "test_target");
  EXPECT_TRUE(ok) << "Mock trajectory execution failed: " << msg;

  const auto current = executor.get_current_positions_or_default();
  ASSERT_EQ(current.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(current[i], target_joints[i], 1e-4);
  }

  // Test mutual exclusion rejection on concurrent call
  const std::vector<double> slow_target{0.5, -0.5, 0.5, -0.5, 0.0, 0.0};
  auto future = std::async(std::launch::async, [&executor, &slow_target]() {
    return executor.execute_joint_target(slow_target, "slow_target", 0.01);
  });

  // Wait briefly until the background trajectory acquisition starts
  for (int i = 0; i < 20 && !executor.is_trajectory_busy(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(executor.is_trajectory_busy());

  // Concurrent execution must be rejected
  auto [ok_concurrent, msg_concurrent] = executor.execute_joint_target(target_joints, "concurrent_target", 1.0);
  EXPECT_FALSE(ok_concurrent);
  EXPECT_NE(msg_concurrent.find("busy"), std::string::npos);

  // Stop executor so slow mock trajectory terminates promptly
  executor.stop();
  auto [ok_bg, msg_bg] = future.get();
  EXPECT_FALSE(executor.is_trajectory_busy());
}

}  // namespace lekiwi_motion::arm::test


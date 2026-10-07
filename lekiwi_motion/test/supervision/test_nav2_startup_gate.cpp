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
 * @file test_nav2_startup_gate.cpp
 * @brief Unit tests for Nav2StartupGateNode parameter initialization and state checking.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "lekiwi_motion/supervision/nav2_startup_gate_node.hpp"

/**
 * @class Nav2StartupGateTest
 * @brief Test fixture initializing and shutting down rclcpp test context.
 */
class Nav2StartupGateTest : public ::testing::Test
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
};

/**
 * @brief Verifies default parameters and initial unstarted state.
 */
TEST_F(Nav2StartupGateTest, DefaultParametersAndInitialState)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lekiwi_motion::Nav2StartupGateNode>(options);

  EXPECT_FALSE(node->is_started());
  EXPECT_EQ(node->get_consecutive_count(), 0);

  EXPECT_EQ(node->get_parameter("map_frame").as_string(), "map");
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), "base_footprint");
  EXPECT_EQ(
      node->get_parameter("lifecycle_service").as_string(),
      "/lifecycle_manager_navigation/manage_nodes");
  EXPECT_DOUBLE_EQ(node->get_parameter("check_frequency_hz").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("consecutive_success_threshold").as_int(), 2);
  EXPECT_FALSE(node->get_parameter("exit_on_success").as_bool());
}

/**
 * @brief Tests custom parameter overrides for frames, thresholds, and auto-exit configuration.
 */
TEST_F(Nav2StartupGateTest, CustomParameters)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"map_frame", "custom_map"},
                               {"base_frame", "custom_base"},
                               {"check_frequency_hz", 5.0},
                               {"consecutive_success_threshold", 4},
                               {"exit_on_success", true}});

  auto node = std::make_shared<lekiwi_motion::Nav2StartupGateNode>(options);

  EXPECT_FALSE(node->is_started());
  EXPECT_EQ(node->get_consecutive_count(), 0);
  EXPECT_EQ(node->get_parameter("map_frame").as_string(), "custom_map");
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), "custom_base");
  EXPECT_DOUBLE_EQ(node->get_parameter("check_frequency_hz").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("consecutive_success_threshold").as_int(), 4);
  EXPECT_TRUE(node->get_parameter("exit_on_success").as_bool());
}

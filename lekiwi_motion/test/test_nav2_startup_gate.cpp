// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "lekiwi_motion/nav2_startup_gate_node.hpp"

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

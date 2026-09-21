/**
 * @file test_chess_engine_action.cpp
 * @brief Unit tests for ChessEngineActionComponent.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "lekiwi_chess_master/chess_engine_action_component.hpp"

using namespace lekiwi_chess_master;

TEST(ChessEngineActionTest, Instantiation)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.append_parameter_override("stockfish_path", "/usr/games/stockfish");
  options.append_parameter_override("think_time_ms", 200);

  auto node = std::make_shared<ChessEngineActionComponent>(options);
  EXPECT_NE(node, nullptr);

  rclcpp::shutdown();
}

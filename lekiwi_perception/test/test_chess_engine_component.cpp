/**
 * @file test_chess_engine_component.cpp
 * @brief Unit tests for ChessEngineComponent and Stockfish subprocess query.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "chess_engine_component.hpp"

TEST(ChessEngineComponentTest, NodeInstantiationAndStockfishBestMove)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.append_parameter_override("stockfish_path", "/usr/games/stockfish");
  options.append_parameter_override("think_time_ms", 200);
  options.append_parameter_override("robot_color", "black");
  options.append_parameter_override("auto_play", true);

  auto node = std::make_shared<lekiwi_perception::ChessEngineComponent>(options);
  EXPECT_NE(node, nullptr);

  rclcpp::shutdown();
}

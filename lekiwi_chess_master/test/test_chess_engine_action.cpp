/**
 * @file test_chess_engine_action.cpp
 * @brief Integration tests for ChessEngineActionComponent lifecycle and cancellation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include "lekiwi_chess_master/chess_engine_action_component.hpp"
#include "lekiwi_interfaces/action/compute_best_move.hpp"

using namespace lekiwi_chess_master;
using ComputeBestMove = lekiwi_interfaces::action::ComputeBestMove;

TEST(ChessEngineActionTest, ActionServerGoalLifecycleAndCancellation)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.append_parameter_override("stockfish_path", "/usr/games/stockfish");
  options.append_parameter_override("think_time_ms", 500);
  options.append_parameter_override("action_name", "/test_chess_engine_action");

  auto server_node = std::make_shared<ChessEngineActionComponent>(options);
  auto client_node = std::make_shared<rclcpp::Node>("test_action_client_node");

  auto action_client = rclcpp_action::create_client<ComputeBestMove>(
      client_node, "/test_chess_engine_action");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  // 1. Verify empty FEN goal rejection
  {
    ComputeBestMove::Goal empty_goal;
    empty_goal.fen = "";

    auto send_goal_options = rclcpp_action::Client<ComputeBestMove>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(empty_goal, send_goal_options);

    auto status = goal_handle_future.wait_for(std::chrono::seconds(2));
    ASSERT_EQ(status, std::future_status::ready);
    auto goal_handle = goal_handle_future.get();
    EXPECT_EQ(goal_handle, nullptr); // Rejected goals return nullptr goal handle
  }

  // 2. Verify goal cancellation without deadlock (Fixes P0)
  {
    ComputeBestMove::Goal long_goal;
    long_goal.fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
    long_goal.think_time_ms = 10000; // 10 seconds search

    auto goal_handle_future = action_client->async_send_goal(long_goal);
    ASSERT_EQ(goal_handle_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);

    // Sleep briefly to ensure engine has entered search loop
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Request cancellation
    auto cancel_future = action_client->async_cancel_goal(goal_handle);
    ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto cancel_response = cancel_future.get();
    EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);

    // Wait for result, which must be canceled
    auto result_future = action_client->async_get_result(goal_handle);
    ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_FALSE(wrapped_result.result->success);
  }

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }

  action_client.reset();
  client_node.reset();
  server_node.reset();
  rclcpp::shutdown();
}

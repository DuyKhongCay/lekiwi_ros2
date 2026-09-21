/**
 * @file test_game_state_tracker.cpp
 * @brief Unit tests for ChessGameStateTrackerComponent.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "lekiwi_chess_master/chess_game_state_tracker_component.hpp"
#include "lekiwi_chess_master/chessboard_visualizer_component.hpp"

using namespace lekiwi_chess_master;

class ChessGameStateTrackerTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(ChessGameStateTrackerTestFixture, InitialStateStartingPlacement)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 2);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

TEST_F(ChessGameStateTrackerTestFixture, ResetBoardGame)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  node->reset_game();
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

TEST_F(ChessGameStateTrackerTestFixture, LegalMoveProcessing)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 1);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR"; // White e2e4

  pub->publish(*msg);
  rclcpp::spin_some(node);

  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1");
}

TEST_F(ChessGameStateTrackerTestFixture, RejectIllegalMove)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 1);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "rnbqkbnr/pppppppp/8/8/4K3/8/PPPPPPPP/RNBQ1BNR"; // King to e4 (illegal)

  pub->publish(*msg);
  rclcpp::spin_some(node);

  // Stays at starting position
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

TEST_F(ChessGameStateTrackerTestFixture, RejectCorruptedPlacement)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 1);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "rnbqqnnr/pppppppp/8/8/8/8/PPPPPP1P/NBQQB1R1"; // Corrupted

  pub->publish(*msg);
  rclcpp::spin_some(node);

  // No crash, stays at starting position
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

TEST_F(ChessGameStateTrackerTestFixture, AutoTriggerParameters)
{
  // Test default parameter values
  {
    rclcpp::NodeOptions default_options;
    auto node = std::make_shared<ChessGameStateTrackerComponent>(default_options);
    EXPECT_FALSE(node->is_auto_trigger_enabled());
    EXPECT_EQ(node->get_robot_color(), "black");
    EXPECT_EQ(node->get_think_time_ms(), 1000);
  }

  // Test overridden parameter values
  {
    rclcpp::NodeOptions custom_options;
    custom_options.append_parameter_override("auto_trigger_engine", true);
    custom_options.append_parameter_override("robot_color", "white");
    custom_options.append_parameter_override("think_time_ms", 2500);

    auto node = std::make_shared<ChessGameStateTrackerComponent>(custom_options);
    EXPECT_TRUE(node->is_auto_trigger_enabled());
    EXPECT_EQ(node->get_robot_color(), "white");
    EXPECT_EQ(node->get_think_time_ms(), 2500);
  }
}

TEST_F(ChessGameStateTrackerTestFixture, InitialGameStatusProperties)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  EXPECT_EQ(node->get_game_phase(), lekiwi_interfaces::msg::ChessGameStatus::PHASE_WAITING_PLAYER);
  EXPECT_TRUE(node->get_best_move().empty());
}

TEST(BoardDisplayContextTest, DefaultInitialization)
{
  BoardDisplayContext ctx;
  EXPECT_EQ(ctx.fen, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_TRUE(ctx.last_move.empty());
  EXPECT_TRUE(ctx.best_move.empty());
  EXPECT_EQ(ctx.eval_cp, 0);
  EXPECT_EQ(ctx.game_phase, 0);
  EXPECT_FALSE(ctx.is_check);
  EXPECT_FALSE(ctx.is_checkmate);
  EXPECT_FALSE(ctx.is_draw);
  EXPECT_FALSE(ctx.is_raw_view);
}

TEST(BoardDisplayContextTest, FromGameStatusFactory)
{
  lekiwi_interfaces::msg::ChessGameStatus status;
  status.full_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1";
  status.last_move = "e2e4";
  status.best_move = "e7e5";
  status.eval_centipawns = 45;
  status.game_phase = lekiwi_interfaces::msg::ChessGameStatus::PHASE_ROBOT_READY;
  status.is_check = true;
  status.is_checkmate = false;
  status.is_draw = false;

  auto ctx = BoardDisplayContext::fromGameStatus(status);
  EXPECT_EQ(ctx.fen, status.full_fen);
  EXPECT_EQ(ctx.last_move, "e2e4");
  EXPECT_EQ(ctx.best_move, "e7e5");
  EXPECT_EQ(ctx.eval_cp, 45);
  EXPECT_EQ(ctx.game_phase, lekiwi_interfaces::msg::ChessGameStatus::PHASE_ROBOT_READY);
  EXPECT_TRUE(ctx.is_check);
  EXPECT_FALSE(ctx.is_checkmate);
  EXPECT_FALSE(ctx.is_draw);
  EXPECT_FALSE(ctx.is_raw_view);
}

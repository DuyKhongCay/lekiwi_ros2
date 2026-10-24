/**
 * @file test_game_state_tracker.cpp
 * @brief Unit tests for ChessGameStateTrackerComponent and BoardDisplayContext.
 *
 * Validates board state tracking, FEN string ingress validation, legal/illegal move
 * transitions, game reset service behavior, ROS 2 parameter bindings, and visualizer
 * display context transformation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <chrono>
#include <thread>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "lekiwi_chess_master/chess_game_state_tracker_component.hpp"
#include "lekiwi_chess_master/chessboard_2d_visualizer.hpp"

using namespace lekiwi_chess_master;

/**
 * @brief GoogleTest fixture managing ROS 2 context lifecycle for tracker component tests.
 */
class ChessGameStateTrackerTestFixture : public ::testing::Test
{
protected:
  /**
   * @brief Initializes global ROS 2 context before executing each test case.
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  /**
   * @brief Shuts down global ROS 2 context after each test case completes.
   */
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// Verifies newly initialized tracker starts with standard FIDE starting FEN
TEST_F(ChessGameStateTrackerTestFixture, InitialStateStartingPlacement)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 2);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

// Verifies explicit reset_game restores standard starting position and turn indicator
TEST_F(ChessGameStateTrackerTestFixture, ResetBoardGame)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  node->reset_game();
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

// Verifies incoming perception raw FEN matching a legal move updates internal board state
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

// Verifies incoming perception raw FEN representing an illegal transition is rejected without mutating state
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

// Verifies corrupted/malformed FEN strings are safely discarded without throwing or crashing
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

// Verifies initial board state properties upon node creation
TEST_F(ChessGameStateTrackerTestFixture, InitialGameStatusProperties)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  EXPECT_FALSE(node->is_awaiting_physical_sync());
  EXPECT_TRUE(node->get_last_move_details().uci.empty());
}

// Verifies default struct field initialization in BoardDisplayContext
TEST(BoardDisplayContextTest, DefaultInitialization)
{
  BoardDisplayContext ctx;
  EXPECT_EQ(ctx.fen, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_TRUE(ctx.last_move.empty());
  EXPECT_TRUE(ctx.candidate_move.empty());
  EXPECT_TRUE(ctx.is_board_synced);
  EXPECT_FALSE(ctx.is_check);
  EXPECT_FALSE(ctx.is_checkmate);
  EXPECT_FALSE(ctx.is_draw);
}

// Verifies set_fen validates, imports, and sets tracker to awaiting physical sync
TEST_F(ChessGameStateTrackerTestFixture, SetFenValidAndAwaitingBoardSync)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 2);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  std::string target_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2";
  std::string target_placement = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR";
  std::string canonical_fen, active_color, message;

  EXPECT_TRUE(node->set_fen(target_fen, canonical_fen, active_color, message));
  EXPECT_EQ(canonical_fen, target_fen);
  EXPECT_EQ(active_color, "w");
  EXPECT_TRUE(node->is_awaiting_physical_sync());

  // Incoming mismatched vision frames do NOT cancel or mutate awaiting sync state
  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());
  auto raw_msg = std::make_shared<std_msgs::msg::String>();
  raw_msg->data = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"; // standard starting placement
  pub->publish(*raw_msg);
  pub->publish(*raw_msg);
  rclcpp::spin_some(node);

  EXPECT_TRUE(node->is_awaiting_physical_sync());
  EXPECT_EQ(node->get_board().getFen(), target_fen);

  // When vision frames match target placement for debounce_frames, sync completes!
  raw_msg->data = target_placement;
  for (int i = 0; i < 10 && node->is_awaiting_physical_sync(); ++i)
  {
    pub->publish(*raw_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(node->is_awaiting_physical_sync());
}

// Verifies set_fen rejects malformed FEN and preserves existing state
TEST_F(ChessGameStateTrackerTestFixture, SetFenRejectsMalformed)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  std::string canonical_fen, active_color, message;
  EXPECT_FALSE(node->set_fen("invalid/fen/string", canonical_fen, active_color, message));
  EXPECT_FALSE(message.empty());
  EXPECT_FALSE(node->is_awaiting_physical_sync());
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

// Verifies factory helper correctly converts ChessGameStatus ROS message into BoardDisplayContext
TEST(BoardDisplayContextTest, FromGameStatusFactory)
{
  lekiwi_interfaces::msg::ChessGameStatus status;
  status.full_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1";
  status.last_move_details.uci = "e2e4";
  status.is_board_synced = true;
  status.is_check = true;
  status.is_checkmate = false;
  status.is_draw = false;

  auto ctx = BoardDisplayContext::fromGameStatus(status, "e7e5");
  EXPECT_EQ(ctx.fen, status.full_fen);
  EXPECT_EQ(ctx.last_move, "e2e4");
  EXPECT_EQ(ctx.candidate_move, "e7e5");
  EXPECT_TRUE(ctx.is_board_synced);
  EXPECT_TRUE(ctx.is_check);
  EXPECT_FALSE(ctx.is_checkmate);
  EXPECT_FALSE(ctx.is_draw);
}


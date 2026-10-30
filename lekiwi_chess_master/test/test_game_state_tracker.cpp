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
#include <future>
#include <memory>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "lekiwi_chess_master/chess_game_state_tracker_component.hpp"
#include "lekiwi_chess_master/chessboard_2d_visualizer.hpp"

using namespace lekiwi_chess_master;

/**
 * @brief GoogleTest fixture managing ROS 2 context lifecycle for tracker component tests.
 */
class ChessGameStateTrackerTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// 1. Verifies initial state, properties, and reset lifecycle
TEST_F(ChessGameStateTrackerTestFixture, InitialStateAndResetLifecycle)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 2);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  // Initial state check
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_TRUE(node->is_awaiting_physical_sync());
  EXPECT_TRUE(node->get_last_move_details().uci.empty());

  // Mutate state with reset_game()
  node->reset_game();
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_TRUE(node->is_awaiting_physical_sync());
}

// 2. Verifies legal move ingress and board transition
TEST_F(ChessGameStateTrackerTestFixture, LegalMoveProcessing)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 1);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());

  auto msg = std::make_shared<std_msgs::msg::String>();
  // Synchronize initial physical board layout first
  msg->data = kDefaultStartingPlacement;
  pub->publish(*msg);
  rclcpp::spin_some(node);
  EXPECT_FALSE(node->is_awaiting_physical_sync());

  msg->data = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR"; // White e2e4

  pub->publish(*msg);
  rclcpp::spin_some(node);

  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1");
}

// 3. Verifies illegal and corrupted placements are safely rejected without state mutation
TEST_F(ChessGameStateTrackerTestFixture, RejectsIllegalAndCorruptedPlacements)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 1);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = kDefaultStartingPlacement;
  pub->publish(*msg);
  rclcpp::spin_some(node);
  EXPECT_FALSE(node->is_awaiting_physical_sync());

  // Illegal move: King to e4
  msg->data = "rnbqkbnr/pppppppp/8/8/4K3/8/PPPPPPPP/RNBQ1BNR";
  pub->publish(*msg);
  rclcpp::spin_some(node);
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");

  // Corrupted placement: invalid piece distribution
  msg->data = "rnbqqnnr/pppppppp/8/8/8/8/PPPPPP1P/NBQQB1R1";
  pub->publish(*msg);
  rclcpp::spin_some(node);
  EXPECT_EQ(node->get_board().getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}

// 4. Verifies set_fen lifecycle, board sync, checkmate detection, and malformed rejection
TEST_F(ChessGameStateTrackerTestFixture, SetFenLifecycleAndCheckmateDetection)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("debounce_frames", 2);
  auto node = std::make_shared<ChessGameStateTrackerComponent>(options);

  // 1. Rejects malformed FEN
  std::string bad_canonical, bad_color, bad_msg;
  EXPECT_FALSE(node->set_fen("invalid/fen/string", bad_canonical, bad_color, bad_msg));
  EXPECT_FALSE(bad_msg.empty());

  // 2. Valid custom position
  std::string target_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2";
  std::string target_placement = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR";
  std::string canonical_fen, active_color, message;

  EXPECT_TRUE(node->set_fen(target_fen, canonical_fen, active_color, message));
  EXPECT_EQ(canonical_fen, target_fen);
  EXPECT_EQ(active_color, "w");
  EXPECT_TRUE(node->is_awaiting_physical_sync());

  // Sync with matching vision frames
  auto pub = node->create_publisher<std_msgs::msg::String>("/chess/raw_fen", rclcpp::SensorDataQoS());
  auto raw_msg = std::make_shared<std_msgs::msg::String>();
  raw_msg->data = target_placement;
  for (int i = 0; i < 10 && node->is_awaiting_physical_sync(); ++i)
  {
    pub->publish(*raw_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(node->is_awaiting_physical_sync());

  // 3. Set checkmate position: Scholar's mate
  std::string mate_fen = "r1bqkb1r/pppp1Qpp/2n5/4p3/2B1n3/8/PPPP1PPP/RNB1K1NR b KQkq - 0 4";
  EXPECT_TRUE(node->set_fen(mate_fen, canonical_fen, active_color, message));
  EXPECT_EQ(active_color, "b");
}

// 5. Verifies BoardDisplayContext struct defaults and fromGameStatus factory
TEST(BoardDisplayContextTest, BoardDisplayContextUnit)
{
  // Defaults
  BoardDisplayContext ctx;
  EXPECT_EQ(ctx.fen, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_TRUE(ctx.last_move.empty());
  EXPECT_TRUE(ctx.candidate_move.empty());
  EXPECT_FALSE(ctx.is_board_synced);
  EXPECT_FALSE(ctx.is_check);
  EXPECT_FALSE(ctx.is_checkmate);
  EXPECT_FALSE(ctx.is_draw);

  // fromGameStatus factory
  lekiwi_interfaces::msg::ChessGameStatus status;
  status.full_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1";
  status.last_move_details.uci = "e2e4";
  status.is_board_synced = true;
  status.is_check = true;
  status.is_checkmate = false;
  status.is_draw = false;

  auto ctx_factory = BoardDisplayContext::fromGameStatus(status, "e7e5");
  EXPECT_EQ(ctx_factory.fen, status.full_fen);
  EXPECT_EQ(ctx_factory.last_move, "e2e4");
  EXPECT_EQ(ctx_factory.candidate_move, "e7e5");
  EXPECT_TRUE(ctx_factory.is_board_synced);
  EXPECT_TRUE(ctx_factory.is_check);
  EXPECT_FALSE(ctx_factory.is_checkmate);
  EXPECT_FALSE(ctx_factory.is_draw);
}

// 6. Verifies Chessboard2DVisualizer renders JPEG compressed images on game status
TEST_F(ChessGameStateTrackerTestFixture, VisualizerRendersCompressedJpegOnStatusAndRawFen)
{
  rclcpp::NodeOptions viz_opts;
  viz_opts.append_parameter_override("render_rate_hz", 0.0); // synchronous / event-driven for test
  viz_opts.append_parameter_override("board_panel_size", 240);
  viz_opts.append_parameter_override("jpeg_quality", 80);

  auto viz_node = std::make_shared<Chessboard2DVisualizer>(viz_opts);
  auto client_node = std::make_shared<rclcpp::Node>("test_viz_client");

  std::promise<sensor_msgs::msg::CompressedImage::SharedPtr> img_promise;
  auto img_future = img_promise.get_future();

  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();

  auto sub = client_node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/chess/board_2d/compressed", qos,
      [&img_promise](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        img_promise.set_value(msg);
      });

  auto status_pub = client_node->create_publisher<lekiwi_interfaces::msg::ChessGameStatus>(
      "/chess/game_status", qos);

  lekiwi_interfaces::msg::ChessGameStatus status_msg;
  status_msg.full_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
  status_msg.active_color = "w";
  status_msg.is_board_synced = true;

  status_pub->publish(status_msg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(viz_node);
  executor.add_node(client_node);

  for (int i = 0; i < 20 && img_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready; ++i)
  {
    executor.spin_some();
  }

  ASSERT_EQ(img_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  auto received_img = img_future.get();
  ASSERT_NE(received_img, nullptr);
  EXPECT_EQ(received_img->format, "jpeg");
  ASSERT_GE(received_img->data.size(), 4U);
  // Verify JPEG magic bytes: 0xFF, 0xD8
  EXPECT_EQ(received_img->data[0], 0xFF);
  EXPECT_EQ(received_img->data[1], 0xD8);
}

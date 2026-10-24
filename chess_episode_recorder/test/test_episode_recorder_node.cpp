/**
 * @file test_episode_recorder_node.cpp
 * @brief Comprehensive ROS 2 unit & integration tests for EpisodeRecorderNode.
 *
 * Validates episode lifecycle transitions (Start -> Stop -> Save) and verifies that
 * discarding an active recording (/discard_episode) retains the cached task instruction
 * from "/manipulation/execute_chess_move/_action/feedback" for subsequent retries in episode_info.yaml.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "chess_episode_recorder/episode_recorder_node.hpp"

namespace chess_episode_recorder
{
namespace test
{

/**
 * @brief Test fixture for EpisodeRecorderNode managing lifecycle, temporary filesystem,
 *        and asynchronous multithreaded ROS 2 execution.
 */
class EpisodeRecorderNodeTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Set up dedicated temporary directory for test storage isolation
    const auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    test_dir_ = std::filesystem::temp_directory_path() /
                ("lekiwi_test_episodes_" + std::to_string(timestamp));
    std::filesystem::create_directories(test_dir_);

    client_node_ = std::make_shared<rclcpp::Node>("test_episode_recorder_client");

    // Initialize mock orchestrator services on client_node first
    mock_import_fen_srv_ = client_node_->create_service<lekiwi_interfaces::srv::SetFen>(
      "/orchestrator/import_fen",
      [](const std::shared_ptr<lekiwi_interfaces::srv::SetFen::Request>,
         std::shared_ptr<lekiwi_interfaces::srv::SetFen::Response> res) {
        res->success = true;
        res->message = "OK";
      });
    mock_feasibility_srv_ = client_node_->create_service<lekiwi_interfaces::srv::CheckMoveFeasibility>(
      "/workspace/check_move_feasibility",
      [](const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request>,
         std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> res) {
        res->feasible = true;
        res->plan_type = 0;
      });
    mock_execute_task_srv_ = client_node_->create_service<lekiwi_interfaces::srv::ExecuteCollectionTask>(
      "/orchestrator/execute_collection_task",
      [](const std::shared_ptr<lekiwi_interfaces::srv::ExecuteCollectionTask::Request>,
         std::shared_ptr<lekiwi_interfaces::srv::ExecuteCollectionTask::Response> res) {
        res->success = true;
        res->message = "OK";
      });

    // Initialize service clients for Trigger endpoints
    start_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/start_episode");
    save_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/save_episode");
    discard_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/discard_episode");
    status_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/get_campaign_status");

    // Initialize feedback publisher to emulate ManipulationActionServer
    feedback_pub_ =
      client_node_->create_publisher<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>(
        "/manipulation/execute_chess_move/_action/feedback", rclcpp::SystemDefaultsQoS());

    // Initialize system readiness publisher to emulate Navigation/Localization convergence
    nav_ready_pub_ =
      client_node_->create_publisher<std_msgs::msg::Bool>("/system/nav_ready", rclcpp::SystemDefaultsQoS());

    // Initialize mock board game status publisher with reliable volatile QoS
    rclcpp::QoS game_status_qos(1);
    game_status_qos.reliable();
    game_status_qos.durability_volatile();
    mock_game_status_pub_ =
      client_node_->create_publisher<lekiwi_interfaces::msg::ChessGameStatus>("/chess/game_status", game_status_qos);

    // Configure EpisodeRecorderNode parameters
    rclcpp::NodeOptions options;
    options.append_parameter_override("root_dir", test_dir_.string());
    options.append_parameter_override("experiment_name", "test_experiment");
    options.append_parameter_override("task", "chess_pick_and_place");
    options.append_parameter_override("storage_id", "mcap");
    options.append_parameter_override(
      "feedback_topic", "/manipulation/execute_chess_move/_action/feedback");
    options.append_parameter_override("topics", std::vector<std::string>{});

    options.append_parameter_override("enable_collection_planner", true);
    options.append_parameter_override("collection_config_path", "");

    recorder_node_ = std::make_shared<EpisodeRecorderNode>(options);

    // Run MultiThreadedExecutor on a background thread for concurrent ROS 2 execution
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(client_node_);
    executor_->add_node(recorder_node_);

    spinning_ = true;
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    // Ensure services are ready
    ASSERT_TRUE(start_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(save_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(discard_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(status_client_->wait_for_service(std::chrono::seconds(5)));

    // Confirm system readiness (/system/nav_ready) to transition from WAITING_FOR_SYSTEM_READY to IDLE_READY
    publish_nav_ready(true);

    // Confirm initial board synchronization (/chess/game_status)
    publish_game_status(true, true);
  }

  void TearDown() override
  {
    spinning_ = false;
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    // Reset recorder node first to stop internal worker thread cleanly
    recorder_node_.reset();

    // Release ROS resources
    mock_import_fen_srv_.reset();
    mock_feasibility_srv_.reset();
    mock_execute_task_srv_.reset();
    feedback_pub_.reset();
    nav_ready_pub_.reset();
    mock_game_status_pub_.reset();
    start_client_.reset();
    save_client_.reset();
    discard_client_.reset();
    status_client_.reset();
    client_node_.reset();
    executor_.reset();

    // Clean up temporary files
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Helper to invoke a Trigger service client asynchronously with a timeout.
   */
  bool call_trigger(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    std::string &response_message,
    std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return false;
    }
    auto response = future.get();
    response_message = response->message;
    return response->success;
  }

  /**
   * @brief Helper to publish mock move execution feedback message.
   */
  void publish_feedback(
    const std::string &instruction,
    const std::string &phase = "APPROACH_PICK",
    float progress = 25.0f)
  {
    lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage msg;
    msg.feedback.instruction = instruction;
    msg.feedback.current_phase = phase;
    msg.feedback.progress_percent = progress;
    feedback_pub_->publish(msg);

    // Yield brief execution time to let ROS executor dispatch the subscription callback
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  /**
   * @brief Helper to publish system readiness status (/system/nav_ready).
   */
  void publish_nav_ready(bool ready)
  {
    std_msgs::msg::Bool msg;
    msg.data = ready;
    nav_ready_pub_->publish(msg);

    // Yield brief execution time to let ROS executor dispatch the subscription callback
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  /**
   * @brief Helper to publish chess game status (/chess/game_status).
   */
  void publish_game_status(bool is_board_stable, bool is_board_synced)
  {
    lekiwi_interfaces::msg::ChessGameStatus msg;
    msg.is_board_stable = is_board_stable;
    msg.is_board_synced = is_board_synced;
    mock_game_status_pub_->publish(msg);

    // Yield brief execution time to let ROS executor dispatch the subscription callback
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::filesystem::path test_dir_;
  std::shared_ptr<EpisodeRecorderNode> recorder_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr discard_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_client_;
  rclcpp::Publisher<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_ready_pub_;
  rclcpp::Publisher<lekiwi_interfaces::msg::ChessGameStatus>::SharedPtr mock_game_status_pub_;
  rclcpp::Service<lekiwi_interfaces::srv::SetFen>::SharedPtr mock_import_fen_srv_;
  rclcpp::Service<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr mock_feasibility_srv_;
  rclcpp::Service<lekiwi_interfaces::srv::ExecuteCollectionTask>::SharedPtr mock_execute_task_srv_;

  std::atomic<bool> spinning_{false};
  std::thread spin_thread_;
};

/**
 * @brief Verifies standard recording workflow: Start -> Stop -> Save.
 *        Checks directory creation, state transitions, and generated episode_info.yaml metadata.
 */
TEST_F(EpisodeRecorderNodeTestFixture, NormalLifecycle_StartStopSave)
{
  std::string response_msg;

  // 1. Initial State
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::IDLE);

  // 2. Start Episode
  bool started = call_trigger(start_client_, response_msg);
  EXPECT_TRUE(started);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::RECORDING);

  const auto exp_dir = test_dir_ / "test_experiment";
  const auto ep0_dir = exp_dir / "episode_000000";
  EXPECT_TRUE(std::filesystem::exists(ep0_dir));

  // 3. Stop Episode
  bool stopped = recorder_node_->stop_episode();
  EXPECT_TRUE(stopped);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::STOPPED);

  // 4. Save Episode
  bool saved = call_trigger(save_client_, response_msg);
  EXPECT_TRUE(saved);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::IDLE);

  // 5. Verify YAML metadata
  const auto yaml_path = ep0_dir / "episode_info.yaml";
  ASSERT_TRUE(std::filesystem::exists(yaml_path));

  YAML::Node info = YAML::LoadFile(yaml_path.string());
  EXPECT_EQ(info["episode_id"].as<uint32_t>(), 0u);
  EXPECT_EQ(info["dataset_format"].as<std::string>(), "lerobot_3.0");
  EXPECT_EQ(info["experiment_name"].as<std::string>(), "test_experiment");
  EXPECT_EQ(info["task"].as<std::string>(), "chess_pick_and_place");
  EXPECT_TRUE(info["success"].as<bool>());
}

/**
 * @brief Core Test: Verifies that calling /discard_episode purges the current recording attempt,
 *        but retains the active task instruction from action feedback so that subsequent retries
 *        persist the exact instruction in episode_info.yaml.
 */
TEST_F(EpisodeRecorderNodeTestFixture, DiscardEpisode_RetainsInstructionOnRetry)
{
  std::string response_msg;
  const std::string move_instruction = "e2e4: move white pawn from e2 to e4";

  // 1. Emulate ManipulationActionServer sending action feedback
  publish_feedback(move_instruction, "APPROACH_PICK", 30.0f);

  // 2. Start initial attempt (episode_000000)
  bool started = call_trigger(start_client_, response_msg);
  ASSERT_TRUE(started);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::RECORDING);

  const auto exp_dir = test_dir_ / "test_experiment";
  const auto ep0_dir = exp_dir / "episode_000000";
  EXPECT_TRUE(std::filesystem::exists(ep0_dir));

  // 3. Flawed trajectory -> Operator triggers Discard
  bool discarded = call_trigger(discard_client_, response_msg);
  EXPECT_TRUE(discarded);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::IDLE);

  // Directory for discarded episode must be cleaned up
  EXPECT_FALSE(std::filesystem::exists(ep0_dir));

  // 4. Operator retries the same chess move -> Start recording again
  bool retry_started = call_trigger(start_client_, response_msg);
  ASSERT_TRUE(retry_started);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::RECORDING);
  EXPECT_TRUE(std::filesystem::exists(ep0_dir));

  // 5. Move succeeds -> Save episode
  bool saved = call_trigger(save_client_, response_msg);
  ASSERT_TRUE(saved);
  EXPECT_EQ(recorder_node_->get_state(), EpisodeState::IDLE);

  // 6. Verify that episode_info.yaml still contains the move instruction from feedback
  const auto yaml_path = ep0_dir / "episode_info.yaml";
  ASSERT_TRUE(std::filesystem::exists(yaml_path));

  YAML::Node info = YAML::LoadFile(yaml_path.string());
  EXPECT_EQ(info["episode_id"].as<uint32_t>(), 0u);
  EXPECT_EQ(info["dataset_format"].as<std::string>(), "lerobot_3.0");
  EXPECT_EQ(info["experiment_name"].as<std::string>(), "test_experiment");
  EXPECT_TRUE(info["success"].as<bool>());

  // Crucial assertion: task field maintained instruction from feedback across discard
  EXPECT_EQ(info["task"].as<std::string>(), move_instruction);
}

/**
 * @brief Campaign Conductor Test: Verifies that saving an episode updates the QuotaTracker
 *        and reflects in the ~/get_campaign_status service response.
 */
TEST_F(EpisodeRecorderNodeTestFixture, CampaignConductor_TracksProgressOnSaveEpisode)
{
  std::string response_msg;

  // 1. Check initial campaign status service
  bool status_ok = call_trigger(status_client_, response_msg);
  EXPECT_TRUE(status_ok);
  EXPECT_NE(response_msg.find("Campaign: 0/600"), std::string::npos);

  // 2. Publish feedback with valid SmolVLA instruction
  const std::string move_instruction = "Move white knight to target";
  publish_feedback(move_instruction, "APPROACH_PICK", 50.0f);

  // 3. Start and Save an episode
  EXPECT_TRUE(call_trigger(start_client_, response_msg));
  EXPECT_TRUE(call_trigger(save_client_, response_msg));

  // 4. Verify that QuotaTracker received the episode
  domain::PieceKey wn{chess::Color::WHITE, chess::PieceType::KNIGHT};
  EXPECT_EQ(recorder_node_->get_quota_tracker().get_completed(domain::StepKind::MOVE, wn), 1u);

  // 5. Query campaign status service again
  EXPECT_TRUE(call_trigger(status_client_, response_msg));
  EXPECT_NE(response_msg.find("Campaign: 1/600"), std::string::npos);
  EXPECT_NE(response_msg.find("MOVE: 1"), std::string::npos);
}

TEST_F(EpisodeRecorderNodeTestFixture, DiagnosticUpdaterProducesValidTelemetry)
{
  diagnostic_updater::DiagnosticStatusWrapper recorder_stat;
  recorder_node_->produce_recorder_diagnostics(recorder_stat);

  EXPECT_EQ(recorder_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(recorder_stat.message, "Episode recorder nominal");

  bool found_episode_state = false;
  bool found_topic_sub = false;
  for (const auto &kv : recorder_stat.values) {
    if (kv.key == "Episode State") {
      found_episode_state = true;
    }
    if (kv.key == "Topic Subscriptions") {
      found_topic_sub = true;
    }
  }
  EXPECT_TRUE(found_episode_state);
  EXPECT_FALSE(found_topic_sub);

  diagnostic_updater::DiagnosticStatusWrapper campaign_stat;
  recorder_node_->produce_campaign_diagnostics(campaign_stat);

  EXPECT_EQ(campaign_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  bool found_campaign_mode = false;
  bool found_last_fen = false;
  for (const auto &kv : campaign_stat.values) {
    if (kv.key == "Campaign Mode") {
      found_campaign_mode = true;
    }
    if (kv.key == "Last Seen FEN") {
      found_last_fen = true;
    }
  }
  EXPECT_TRUE(found_campaign_mode);
  EXPECT_FALSE(found_last_fen);
}

/**
 * @brief Gating Contract Test: Verifies that /system/nav_ready controls campaign lifecycle.
 *        Transitioning to false suspends planning and sets state to WAITING_FOR_SYSTEM_READY.
 *        Transitioning to true recovers state to IDLE_READY.
 */
TEST_F(EpisodeRecorderNodeTestFixture, CampaignGating_SystemNavReadyLifecycle)
{
  // 1. Initial State: Ready confirmed in SetUp, initial boot sync completed
  EXPECT_TRUE(recorder_node_->is_system_ready());
  EXPECT_TRUE(recorder_node_->has_initial_boot_synced());
  EXPECT_NE(recorder_node_->get_campaign_state(), CampaignState::WAITING_FOR_SYSTEM_READY);

  // 2. Lost Localization/TF: Publish /system/nav_ready = false
  publish_nav_ready(false);
  EXPECT_FALSE(recorder_node_->is_system_ready());
  EXPECT_TRUE(recorder_node_->has_initial_boot_synced());
  EXPECT_EQ(recorder_node_->get_campaign_state(), CampaignState::WAITING_FOR_SYSTEM_READY);

  // Verify diagnostic updater reports WAITING_FOR_SYSTEM_READY and WARN level
  diagnostic_updater::DiagnosticStatusWrapper campaign_stat;
  recorder_node_->produce_campaign_diagnostics(campaign_stat);
  EXPECT_EQ(campaign_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(campaign_stat.message.find("Waiting for system readiness") != std::string::npos);

  // 3. Regained Localization/TF: Publish /system/nav_ready = true
  publish_nav_ready(true);
  EXPECT_TRUE(recorder_node_->is_system_ready());
  EXPECT_TRUE(recorder_node_->has_initial_boot_synced());
  EXPECT_NE(recorder_node_->get_campaign_state(), CampaignState::WAITING_FOR_SYSTEM_READY);
}

/**
 * @brief Board Synchronization Gate Test: Verifies that /chess/game_status controls board readiness.
 *        Transitioning to PHASE_AWAITING_BOARD_SYNC blocks planning until physical board is stable and confirmed.
 */
TEST_F(EpisodeRecorderNodeTestFixture, CampaignGating_BoardSyncGate)
{
  // 1. Initial State: Synced confirmed in SetUp
  EXPECT_TRUE(recorder_node_->is_board_synced());
  EXPECT_TRUE(recorder_node_->is_board_stable());

  // 2. Physical board desynced: Tracker awaits board sync
  publish_game_status(false, false);
  EXPECT_FALSE(recorder_node_->is_board_synced());
  EXPECT_FALSE(recorder_node_->is_board_stable());

  // Verify diagnostic updater reports board synchronization warning
  diagnostic_updater::DiagnosticStatusWrapper campaign_stat;
  recorder_node_->produce_campaign_diagnostics(campaign_stat);
  EXPECT_EQ(campaign_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(campaign_stat.message.find("Waiting for physical board synchronization") != std::string::npos);

  // 3. Physical board reset and verified: Tracker synced with is_board_stable = true
  publish_game_status(true, true);
  EXPECT_TRUE(recorder_node_->is_board_synced());
  EXPECT_TRUE(recorder_node_->is_board_stable());
}

/**
 * @brief Verifies that completing a move locks the Board Sync Gate (is_board_synced = false)
 *        to await robot observation standoff and camera verification, and that planning
 *        is only reactivated when /chess/game_status confirms stability.
 */
TEST_F(EpisodeRecorderNodeTestFixture, MoveCompletion_LocksBoardSyncGateUntilVerified)
{
  std::string response_msg;

  // 1. Initial State: board synced and stable
  EXPECT_TRUE(recorder_node_->is_board_synced());

  // Wait briefly for worker thread to plan and dispatch initial move
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // 2. Start and complete all episodes for the active move
  publish_feedback("Move white pawn to target");
  EXPECT_TRUE(call_trigger(start_client_, response_msg));
  EXPECT_TRUE(call_trigger(save_client_, response_msg));

  // If the dispatched move requires 2 episodes, complete the 2nd episode to finalize the move
  if (recorder_node_->is_board_synced()) {
    publish_feedback("Move white pawn to target");
    EXPECT_TRUE(call_trigger(start_client_, response_msg));
    EXPECT_TRUE(call_trigger(save_client_, response_msg));
  }

  // 3. Verify Board Sync Gate is now locked (is_board_synced == false)
  EXPECT_FALSE(recorder_node_->is_board_synced());

  // Diagnostics should report waiting for physical board synchronization
  diagnostic_updater::DiagnosticStatusWrapper campaign_stat;
  recorder_node_->produce_campaign_diagnostics(campaign_stat);
  EXPECT_EQ(campaign_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(campaign_stat.message.find("Waiting for physical board synchronization") != std::string::npos);

  // 4. Standoff reached & camera verifies board: publish game_status confirmed
  publish_game_status(true, true);
  EXPECT_TRUE(recorder_node_->is_board_synced());
}

}  // namespace test
}  // namespace chess_episode_recorder


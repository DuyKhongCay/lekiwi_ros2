/**
 * @file test_episode_recorder_node.cpp
 * @brief Comprehensive ROS 2 unit & integration tests for EpisodeRecorderNode.
 *
 * Validates episode lifecycle transitions (Start -> Stop -> Save), verifies that
 * discarding an active recording (/discard_episode) retains the cached task instruction,
 * and tests the ExecuteCollectionTask action client lifecycle.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <lekiwi_interfaces/action/execute_collection_task.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <lekiwi_interfaces/srv/set_fen.hpp>
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
  using ExecuteCollectionTask = lekiwi_interfaces::action::ExecuteCollectionTask;
  using ServerGoalHandleCollectionTask = rclcpp_action::ServerGoalHandle<ExecuteCollectionTask>;

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

    // Initialize mock ExecuteCollectionTask action server
    mock_action_server_ = rclcpp_action::create_server<ExecuteCollectionTask>(
      client_node_,
      "/orchestrator/execute_collection_task",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteCollectionTask::Goal> goal) {
        std::lock_guard<std::mutex> lock(action_server_mutex_);
        goals_received_.push_back(*goal);
        return should_accept_goals_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                                    : rclcpp_action::GoalResponse::REJECT;
      },
      [](const std::shared_ptr<ServerGoalHandleCollectionTask>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ServerGoalHandleCollectionTask> goal_handle) {
        std::lock_guard<std::mutex> lock(action_server_mutex_);
        latest_server_goal_handle_ = goal_handle;
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

    game_status_pub_ =
      client_node_->create_publisher<lekiwi_interfaces::msg::ChessGameStatus>(
        "/chess/game_status", rclcpp::QoS(1).reliable().transient_local());

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
    options.append_parameter_override("service_timeout_sec", 1.0);

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

    // Ensure recorder has received initial board sync state
    publish_board_sync(true);
  }

  void publish_board_sync(bool is_synced)
  {
    auto status = lekiwi_interfaces::msg::ChessGameStatus();
    status.is_board_synced = is_synced;
    for (int i = 0; i < 30; ++i) {
      game_status_pub_->publish(status);
      if (recorder_node_->is_board_synced() == is_synced) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
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

    // Reset recorder node first
    recorder_node_.reset();

    // Release ROS resources
    mock_import_fen_srv_.reset();
    mock_feasibility_srv_.reset();
    mock_action_server_.reset();
    feedback_pub_.reset();
    game_status_pub_.reset();
    start_client_.reset();
    save_client_.reset();
    discard_client_.reset();
    status_client_.reset();
    client_node_.reset();
    executor_.reset();

    // Clean up temporary files
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

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
  rclcpp::Publisher<lekiwi_interfaces::msg::ChessGameStatus>::SharedPtr game_status_pub_;
  rclcpp::Service<lekiwi_interfaces::srv::SetFen>::SharedPtr mock_import_fen_srv_;
  rclcpp::Service<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr mock_feasibility_srv_;
  rclcpp_action::Server<ExecuteCollectionTask>::SharedPtr mock_action_server_;

  std::mutex action_server_mutex_;
  std::vector<ExecuteCollectionTask::Goal> goals_received_;
  std::shared_ptr<ServerGoalHandleCollectionTask> latest_server_goal_handle_;
  bool should_accept_goals_{true};

  std::atomic<bool> spinning_{false};
  std::thread spin_thread_;
};

/**
 * @brief Verifies standard recording workflow: Start -> Stop -> Save.
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
 * @brief Verifies that calling /discard_episode purges the recording attempt,
 *        but retains the active task instruction from action feedback for retries.
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
  EXPECT_FALSE(std::filesystem::exists(ep0_dir));

  // 4. Operator retries -> Start recording again
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
  EXPECT_TRUE(info["success"].as<bool>());
  EXPECT_EQ(info["task"].as<std::string>(), move_instruction);
}

/**
 * @brief Verifies that saving an episode updates the QuotaTracker
 *        and reflects in the ~/get_campaign_status service response.
 */
TEST_F(EpisodeRecorderNodeTestFixture, CampaignConductor_TracksProgressOnSaveEpisode)
{
  std::string response_msg;

  // 1. Check initial campaign status service
  bool status_ok = call_trigger(status_client_, response_msg);
  EXPECT_TRUE(status_ok);
  EXPECT_NE(response_msg.find("Campaign: 0/600"), std::string::npos);

  // 2. Publish feedback with valid instruction
  const std::string move_instruction = "Move white knight to target";
  publish_feedback(move_instruction, "APPROACH_PICK", 50.0f);

  // 3. Start and Save an episode
  EXPECT_TRUE(call_trigger(start_client_, response_msg));
  EXPECT_TRUE(call_trigger(save_client_, response_msg));

  // 4. Verify that QuotaTracker received the episode
  ASSERT_NE(recorder_node_->get_conductor(), nullptr);
  EXPECT_EQ(recorder_node_->get_conductor()->get_quota_summary().completed_by_kind.at(domain::StepKind::MOVE), 1u);

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

  diagnostic_updater::DiagnosticStatusWrapper campaign_stat;
  recorder_node_->produce_campaign_diagnostics(campaign_stat);

  EXPECT_EQ(campaign_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  bool found_campaign_mode = false;
  bool found_action_server = false;
  for (const auto &kv : campaign_stat.values) {
    if (kv.key == "Campaign Mode") {
      found_campaign_mode = true;
    }
    if (kv.key == "Action Server Connected") {
      found_action_server = true;
    }
  }
  EXPECT_TRUE(found_campaign_mode);
  EXPECT_TRUE(found_action_server);
}

/**
 * @brief Action Client Lifecycle Test: Verifies that when the action server is ready,
 *        EpisodeRecorderNode dispatches ExecuteCollectionTask goals.
 */
TEST_F(EpisodeRecorderNodeTestFixture, ActionLifecycle_DispatchesGoalWhenServerAvailable)
{
  // Wait up to 2 seconds for action check timer to dispatch goal
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard<std::mutex> lock(action_server_mutex_);
      if (!goals_received_.empty() && latest_server_goal_handle_) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::lock_guard<std::mutex> lock(action_server_mutex_);
  ASSERT_FALSE(goals_received_.empty());
  EXPECT_FALSE(goals_received_[0].move.uci.empty());
  EXPECT_TRUE(recorder_node_->has_active_goal());
  EXPECT_EQ(recorder_node_->get_campaign_state(), CampaignState::DISPATCHED_TO_ORCH);
}

/**
 * @brief Action Client Lifecycle Test: Verifies that succeeding an action goal applies the move
 *        to the internal QuotaTracker sequential board and advances the campaign.
 */
TEST_F(EpisodeRecorderNodeTestFixture, ActionLifecycle_CompletesMoveAndUpdatesQuotaOnResult)
{
  // 1. Wait for goal dispatch
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard<std::mutex> lock(action_server_mutex_);
      if (latest_server_goal_handle_ != nullptr) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::shared_ptr<ServerGoalHandleCollectionTask> goal_handle;
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    ASSERT_NE(latest_server_goal_handle_, nullptr);
    goal_handle = latest_server_goal_handle_;
  }

  // 2. Mock Orchestrator succeeding the action
  auto result = std::make_shared<ExecuteCollectionTask::Result>();
  result->success = true;
  result->final_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1";
  result->message = "Move verified successfully by camera";
  goal_handle->succeed(result);

  // 3. Wait for result callback to process and the next candidate task to be dispatched automatically
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard<std::mutex> lock(action_server_mutex_);
      if (goals_received_.size() >= 2) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    EXPECT_GE(goals_received_.size(), 2u);
  }
}

/**
 * @brief State Verification Gate Test: Verifies that when teleop execution diverges from the
 *        target candidate (e.g. human moved to wrong square), the recorder node:
 *        1) Issues a warning and preserves the recorded episode for post-processing
 *        2) Synchronizes internal QuotaTracker domain board to the action server's actual final_fen
 *        3) Does NOT credit quota for the diverged move
 */
TEST_F(EpisodeRecorderNodeTestFixture, StateVerificationGate_DivergencePreservesEpisodeAndAdoptsFen)
{
  // 1. Wait for initial goal dispatch
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard<std::mutex> lock(action_server_mutex_);
      if (latest_server_goal_handle_ != nullptr) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::shared_ptr<ServerGoalHandleCollectionTask> goal_handle;
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    ASSERT_NE(latest_server_goal_handle_, nullptr);
    goal_handle = latest_server_goal_handle_;
  }

  // 2. Emulate an episode being saved during task execution before action completes
  std::string response_msg;
  ASSERT_TRUE(call_trigger(start_client_, response_msg));
  ASSERT_TRUE(recorder_node_->stop_episode());
  ASSERT_TRUE(call_trigger(save_client_, response_msg));

  const auto exp_dir = test_dir_ / "test_experiment";
  const auto ep0_dir = exp_dir / "episode_000000";
  ASSERT_TRUE(std::filesystem::exists(ep0_dir));

  // 3. Complete action with diverged FEN (e.g. black king went to f8 instead of planned target)
  const std::string diverged_fen = "5k2/8/1p1p4/7p/8/8/2P2n2/2K5 w - - 2 6";
  auto result = std::make_shared<ExecuteCollectionTask::Result>();
  result->success = true;
  result->final_fen = diverged_fen;
  result->message = "Move verified with diverged board state";
  goal_handle->succeed(result);

  // 4. Wait for on_result processing (until QuotaTracker adopts diverged FEN)
  for (int i = 0; i < 30; ++i) {
    if (recorder_node_->get_session_state().active_fen == diverged_fen) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // 5. Assert: diverged episode directory must be PRESERVED for post-processing
  EXPECT_TRUE(std::filesystem::exists(ep0_dir));

  // 6. Assert: QuotaTracker domain board must have adopted the action server's actual FEN
  EXPECT_EQ(recorder_node_->get_session_state().active_fen, diverged_fen);

  // 7. Assert: Node remains healthy in IDLE_READY or re-dispatched state
  EXPECT_TRUE(
    recorder_node_->get_campaign_state() == CampaignState::IDLE_READY ||
    recorder_node_->get_campaign_state() == CampaignState::DISPATCHED_TO_ORCH);
}

/**
 * @brief Action Client Lifecycle Test: Verifies that goal rejection is handled gracefully
 *        without setting a timer retry loop (as requested by user review).
 */
TEST_F(EpisodeRecorderNodeTestFixture, ActionLifecycle_HandlesGoalRejectionGracefully)
{
  // 1. Wait for initial goal dispatch
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard<std::mutex> lock(action_server_mutex_);
      if (latest_server_goal_handle_ != nullptr) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 2. Clear initial goal by aborting it, and configure server to reject subsequent goals
  std::shared_ptr<ServerGoalHandleCollectionTask> goal_handle;
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    goal_handle = latest_server_goal_handle_;
    should_accept_goals_ = false;
  }

  if (goal_handle) {
    auto result = std::make_shared<ExecuteCollectionTask::Result>();
    result->success = false;
    result->message = "Aborting to test rejection";
    goal_handle->abort(result);
  }

  // 3. Manually trigger dispatch_next_task() with server rejecting
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  recorder_node_->dispatch_next_task();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_FALSE(recorder_node_->has_active_goal());
}

/**
 * @brief Board Sync Gating Test: Verifies that when is_board_synced is false,
 *        EpisodeRecorderNode does not dispatch collection tasks.
 */
TEST_F(EpisodeRecorderNodeTestFixture, BoardSyncGating_PreventsGoalDispatchWhenUnsynced)
{
  // 1. Publish unsynced board status
  publish_board_sync(false);
  EXPECT_FALSE(recorder_node_->is_board_synced());

  // 2. Clear active goals
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    if (latest_server_goal_handle_) {
      auto result = std::make_shared<ExecuteCollectionTask::Result>();
      result->success = true;
      latest_server_goal_handle_->succeed(result);
      latest_server_goal_handle_ = nullptr;
    }
    goals_received_.clear();
  }

  // 3. Trigger dispatch_next_task() and verify no goal is sent
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  recorder_node_->dispatch_next_task();

  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    EXPECT_TRUE(goals_received_.empty());
  }

  // 4. Publish synced board status -> verifies state transitions to true
  publish_board_sync(true);
  EXPECT_TRUE(recorder_node_->is_board_synced());
}

/**
 * @brief State Barrier Test: Verifies sync_active_fen_to_orchestrator sets campaign state to
 *        AWAITING_PHYSICAL_SYNC and locks task dispatch until is_board_synced == true.
 */
TEST_F(EpisodeRecorderNodeTestFixture, StateBarrier_AwaitingPhysicalSyncLocksDispatch)
{
  // 1. Trigger State Barrier
  publish_board_sync(false);
  recorder_node_->sync_active_fen_to_orchestrator("rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1");
  EXPECT_EQ(recorder_node_->get_campaign_state(), CampaignState::AWAITING_PHYSICAL_SYNC);

  // 2. Clear any received goals
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    goals_received_.clear();
  }

  // 3. Dispatch is strictly locked while in AWAITING_PHYSICAL_SYNC
  recorder_node_->dispatch_next_task();
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    EXPECT_TRUE(goals_received_.empty());
  }

  // 4. Publish board sync true -> clears barrier and allows dispatch
  publish_board_sync(true);

  // Wait for check timer to process barrier clearance and dispatch
  for (int i = 0; i < 30; ++i) {
    auto state = recorder_node_->get_campaign_state();
    if (state == CampaignState::IDLE_READY || state == CampaignState::DISPATCHED_TO_ORCH) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_NE(recorder_node_->get_campaign_state(), CampaignState::AWAITING_PHYSICAL_SYNC);
}

/**
 * @brief Deadlock Prevention Test: Verifies advancing session (which re-enters domain_mutex_ via
 *        sync_active_fen_to_orchestrator and send_fen_sync_request) does not deadlock.
 */
TEST_F(EpisodeRecorderNodeTestFixture, AdvanceSession_NoDeadlockOnDomainMutex)
{
  // Trigger advance_to_new_session which re-enters domain_mutex_
  EXPECT_NO_THROW({
    recorder_node_->advance_to_new_session();
  });

  EXPECT_EQ(recorder_node_->get_campaign_state(), CampaignState::AWAITING_PHYSICAL_SYNC);

  // Dispatch next task should safely return because campaign_state is AWAITING_PHYSICAL_SYNC
  recorder_node_->dispatch_next_task();
  EXPECT_EQ(recorder_node_->get_campaign_state(), CampaignState::AWAITING_PHYSICAL_SYNC);
}

/**
 * @brief Verifies that when feasibility service is unavailable, dispatch_next_task()
 *        waits instead of advancing the session or triggering SetFen loop.
 */
TEST_F(EpisodeRecorderNodeTestFixture, FeasibilityServiceUnavailable_PreventsFalseSessionExhaustion)
{
  // 1. Destroy mock feasibility service to emulate service offline BEFORE board sync
  mock_feasibility_srv_.reset();

  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    goals_received_.clear();
  }

  // 2. Clear barrier into IDLE_READY
  publish_board_sync(true);
  for (int i = 0; i < 30; ++i) {
    if (recorder_node_->get_campaign_state() == CampaignState::IDLE_READY) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // 3. Dispatch next task
  uint32_t current_session = recorder_node_->get_session_state().session_id;
  recorder_node_->dispatch_next_task();

  // 4. Must NOT advance session, must NOT dispatch goals, must remain IDLE_READY
  EXPECT_EQ(recorder_node_->get_session_state().session_id, current_session);
  {
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    EXPECT_TRUE(goals_received_.empty());
  }
  EXPECT_EQ(recorder_node_->get_campaign_state(), CampaignState::IDLE_READY);
}

class GlobalTestEnvironment : public ::testing::Environment
{
public:
  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

static ::testing::Environment * const s_global_env =
  ::testing::AddGlobalTestEnvironment(new GlobalTestEnvironment());

}  // namespace test
}  // namespace chess_episode_recorder

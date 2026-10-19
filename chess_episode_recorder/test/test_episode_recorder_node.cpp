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

    // Configure EpisodeRecorderNode parameters
    rclcpp::NodeOptions options;
    options.append_parameter_override("root_dir", test_dir_.string());
    options.append_parameter_override("experiment_name", "test_experiment");
    options.append_parameter_override("task", "chess_pick_and_place");
    options.append_parameter_override("storage_id", "mcap");
    options.append_parameter_override(
      "feedback_topic", "/manipulation/execute_chess_move/_action/feedback");
    options.append_parameter_override("topics", std::vector<std::string>{});

    recorder_node_ = std::make_shared<EpisodeRecorderNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("test_episode_recorder_client");

    // Initialize service clients for Trigger endpoints
    start_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/start_episode");
    save_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/save_episode");
    discard_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/episode_recorder_node/discard_episode");

    // Initialize feedback publisher to emulate ManipulationActionServer
    feedback_pub_ =
      client_node_->create_publisher<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>(
        "/manipulation/execute_chess_move/_action/feedback", rclcpp::SystemDefaultsQoS());

    // Run SingleThreadedExecutor on a background thread for non-blocking asynchronous coordination
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(recorder_node_);
    executor_->add_node(client_node_);

    spinning_ = true;
    spin_thread_ = std::thread([this]() {
      while (spinning_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });

    // Ensure services are ready
    ASSERT_TRUE(start_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(save_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(discard_client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override
  {
    spinning_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    if (executor_) {
      executor_->cancel();
    }

    // Release ROS resources
    feedback_pub_.reset();
    start_client_.reset();
    save_client_.reset();
    discard_client_.reset();
    client_node_.reset();
    recorder_node_.reset();
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

  std::filesystem::path test_dir_;
  std::shared_ptr<EpisodeRecorderNode> recorder_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr discard_client_;
  rclcpp::Publisher<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>::SharedPtr feedback_pub_;

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

}  // namespace test
}  // namespace chess_episode_recorder


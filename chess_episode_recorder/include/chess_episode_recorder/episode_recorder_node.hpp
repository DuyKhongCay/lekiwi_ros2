// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <lekiwi_interfaces/action/execute_collection_task.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <lekiwi_interfaces/srv/set_fen.hpp>

#include "chess_episode_recorder/collection_planner_domain.hpp"
#include "chess_episode_recorder/recorder_diagnostics.hpp"

namespace chess_episode_recorder
{

class EpisodeRecorderNode : public rclcpp::Node
{
public:
  using ExecuteCollectionTask = lekiwi_interfaces::action::ExecuteCollectionTask;
  using GoalHandleCollectionTask = rclcpp_action::ClientGoalHandle<ExecuteCollectionTask>;

  explicit EpisodeRecorderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~EpisodeRecorderNode() override;

  // Manual & Teleop Episode Transitions
  bool start_episode();
  bool stop_episode();
  bool save_episode();
  bool discard_episode();

  [[nodiscard]] EpisodeState get_state() const { return current_state_.load(); }
  [[nodiscard]] CampaignState get_campaign_state() const { return campaign_state_.load(); }
  [[nodiscard]] bool is_action_server_connected() const;
  [[nodiscard]] bool is_board_synced() const { return is_board_synced_.load(); }
  [[nodiscard]] bool has_active_goal() const;
  [[nodiscard]] const domain::QuotaTracker &get_quota_tracker() const { return *quota_tracker_; }

  // Diagnostics callbacks & snapshot
  void produce_recorder_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void produce_campaign_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  [[nodiscard]] RecorderDiagnosticsSnapshot capture_diagnostics_snapshot() const;

  // Campaign Dispatch Entrypoint & FEN Synchronization
  void dispatch_next_task();
  bool sync_active_fen_to_orchestrator(const std::string &fen);
  void advance_to_new_session();

private:
  void init_parameters();
  void init_services();
  void init_campaign_conductor();
  void init_diagnostics();

  // Bag writing & discovery
  void discover_and_subscribe_topics();
  void on_feedback_received(
    const lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage::ConstSharedPtr msg);
  void write_serialized_message(
    const std::string &topic_name,
    const std::string &topic_type,
    std::shared_ptr<const rclcpp::SerializedMessage> msg);

  uint32_t scan_next_episode_index(const std::filesystem::path &dir);
  std::filesystem::path make_episode_dir(uint32_t index);
  bool write_episode_info_yaml(
    const std::filesystem::path &episode_dir,
    uint32_t episode_index,
    double duration_sec,
    const rclcpp::Time &start_time,
    const rclcpp::Time &end_time);

  // Campaign Conductor Logic
  bool send_fen_sync_request();
  uint32_t calculate_expected_episodes(const domain::MoveCandidate &cand) const;
  bool query_move_feasibility_sync(
    const domain::MoveCandidate &cand,
    const chess::Board &board,
    uint8_t &out_plan_type);
  void check_action_server_and_dispatch();
  void on_game_status_received(const lekiwi_interfaces::msg::ChessGameStatus::ConstSharedPtr msg);

  // Action client callbacks
  void on_goal_response(const GoalHandleCollectionTask::SharedPtr & goal_handle);
  void on_feedback(
    GoalHandleCollectionTask::SharedPtr goal_handle,
    const std::shared_ptr<const ExecuteCollectionTask::Feedback> feedback);
  void on_result(const GoalHandleCollectionTask::WrappedResult & result);
  void trigger_deferred_dispatch();

  // Service callbacks
  void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_save(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_discard(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_campaign_status(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Storage and configuration parameters
  std::string root_dir_;
  std::string experiment_name_;
  std::string task_;
  std::string storage_id_;
  std::string feedback_topic_{"/manipulation/execute_chess_move/_action/feedback"};
  std::vector<std::string> requested_topics_;
  std::filesystem::path base_output_dir_;

  // Campaign parameters
  bool enable_collection_planner_{false};
  domain::CampaignConfig campaign_config_{};
  double service_timeout_sec_{5.0};
  std::string game_status_topic_{"/chess/game_status"};
  std::string import_fen_srv_name_{"/orchestrator/import_fen"};
  std::string check_feasibility_srv_name_{"/workspace/check_move_feasibility"};
  std::string execute_collection_task_action_name_{"/orchestrator/execute_collection_task"};

  // Episode state & storage
  std::atomic<EpisodeState> current_state_{EpisodeState::IDLE};
  std::atomic<CampaignState> campaign_state_{CampaignState::DISABLED};
  std::atomic<bool> is_board_synced_{false};
  std::atomic<bool> fen_sync_acknowledged_{false};
  std::atomic<bool> fen_sync_in_flight_{false};
  std::atomic<bool> last_feasibility_query_succeeded_{true};
  std::string target_session_fen_;
  uint32_t current_episode_index_{0};
  std::filesystem::path current_episode_dir_;
  rclcpp::Time episode_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time episode_stop_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::mutex writer_mutex_;

  // Topic discovery and subscription maps
  std::map<std::string, std::string> topic_type_map_;
  std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  std::map<std::string, uint64_t> message_counts_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  // Feedback Context
  rclcpp::Subscription<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>::SharedPtr feedback_sub_;
  rclcpp::Subscription<lekiwi_interfaces::msg::ChessGameStatus>::SharedPtr game_status_sub_;
  std::mutex instruction_mutex_;
  std::string current_instruction_{""};

  // ROS 2 Trigger services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_discard_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_campaign_status_;

  // Campaign Conductor Domain & Clients
  std::unique_ptr<domain::QuotaTracker> quota_tracker_;
  std::unique_ptr<domain::FenSynthesizer> fen_synthesizer_;
  std::unique_ptr<domain::MoveCandidateSelector> move_selector_;
  mutable std::recursive_mutex domain_mutex_;

  // Active Move Tracking (for multi-episode atomic chess moves)
  std::optional<domain::MoveCandidate> active_move_candidate_;
  uint32_t active_move_expected_episodes_{0};
  uint32_t active_move_recorded_episodes_{0};

  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;
  rclcpp::Client<lekiwi_interfaces::srv::SetFen>::SharedPtr import_fen_client_;
  rclcpp::Client<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr check_feasibility_client_;
  rclcpp_action::Client<ExecuteCollectionTask>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleCollectionTask> active_goal_handle_;
  std::mutex goal_mutex_;

  rclcpp::TimerBase::SharedPtr action_server_check_timer_;
  rclcpp::TimerBase::SharedPtr defer_dispatch_timer_;
  std::atomic<bool> is_running_{true};

  // Diagnostics Updater
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
};

} // namespace chess_episode_recorder

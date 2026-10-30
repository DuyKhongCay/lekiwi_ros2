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
#include <std_srvs/srv/trigger.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <lekiwi_interfaces/action/execute_collection_task.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <lekiwi_interfaces/srv/set_fen.hpp>

#include "chess_episode_recorder/campaign_quota_domain.hpp"
#include "chess_episode_recorder/move_planning_domain.hpp"
#include "chess_episode_recorder/dataset_storage.hpp"
#include "chess_episode_recorder/campaign_conductor.hpp"

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
  [[nodiscard]] CampaignState get_campaign_state() const;
  [[nodiscard]] bool is_action_server_connected() const;
  [[nodiscard]] bool is_board_synced() const;
  [[nodiscard]] bool has_active_goal() const;
  [[nodiscard]] const CampaignConductor * get_conductor() const { return conductor_.get(); }
  [[nodiscard]] domain::SessionState get_session_state() const;

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

  // Campaign Conductor Logic & Dispatch
  bool send_fen_sync_request();
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

  // Storage & Writer
  storage::McapEpisodeWriter writer_;
  std::atomic<EpisodeState> current_state_{EpisodeState::IDLE};
  uint32_t current_episode_index_{0};
  std::filesystem::path current_episode_dir_;
  rclcpp::Time episode_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time episode_stop_time_{0, 0, RCL_ROS_TIME};

  // Topic discovery and subscription maps
  std::map<std::string, std::string> topic_type_map_;
  std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  // Feedback Context
  rclcpp::Subscription<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>::SharedPtr feedback_sub_;
  rclcpp::Subscription<lekiwi_interfaces::msg::ChessGameStatus>::SharedPtr game_status_sub_;
  mutable std::mutex instruction_mutex_;
  std::string current_instruction_{""};

  // ROS 2 Trigger services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_discard_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_campaign_status_;

  // Campaign Conductor
  std::unique_ptr<CampaignConductor> conductor_;

  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;
  rclcpp::Client<lekiwi_interfaces::srv::SetFen>::SharedPtr import_fen_client_;
  rclcpp::Client<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr check_feasibility_client_;
  rclcpp_action::Client<ExecuteCollectionTask>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleCollectionTask> active_goal_handle_;
  mutable std::mutex goal_mutex_;

  std::atomic<bool> fen_sync_in_flight_{false};
  rclcpp::TimerBase::SharedPtr action_server_check_timer_;
  rclcpp::TimerBase::SharedPtr defer_dispatch_timer_;
  std::atomic<bool> is_running_{true};

  // Diagnostics Updater
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
};

} // namespace chess_episode_recorder

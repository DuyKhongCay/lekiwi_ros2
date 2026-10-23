// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <lekiwi_interfaces/srv/execute_collection_task.hpp>
#include <lekiwi_interfaces/srv/set_fen.hpp>

#include "chess_episode_recorder/collection_planner_domain.hpp"
#include "chess_episode_recorder/recorder_diagnostics.hpp"

namespace chess_episode_recorder
{

class EpisodeRecorderNode : public rclcpp::Node
{
public:
  explicit EpisodeRecorderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~EpisodeRecorderNode() override;

  // Manual & Teleop Episode Transitions
  bool start_episode();
  bool stop_episode();
  bool save_episode();
  bool discard_episode();

  [[nodiscard]] EpisodeState get_state() const { return current_state_.load(); }
  [[nodiscard]] CampaignState get_campaign_state() const { return campaign_state_.load(); }
  [[nodiscard]] const domain::QuotaTracker &get_quota_tracker() const { return *quota_tracker_; }

  // Diagnostics callbacks & snapshot
  void produce_recorder_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void produce_campaign_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  [[nodiscard]] RecorderDiagnosticsSnapshot capture_diagnostics_snapshot() const;

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
  void on_game_status_received(const lekiwi_interfaces::msg::ChessGameStatus::ConstSharedPtr msg);
  void trigger_campaign_step_async(const std::string &fen);
  bool query_move_feasibility_sync(
    const domain::MoveCandidate &cand,
    const chess::Board &board,
    uint8_t &out_plan_type);
  void dispatch_task_to_orchestrator(
    const domain::MoveCandidate &cand,
    const chess::Board &board);
  void advance_to_new_session();

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
  std::string collection_config_path_;
  std::string game_status_topic_{"/chess/game_status"};
  std::string import_fen_srv_name_{"/orchestrator/import_fen"};
  std::string export_fen_srv_name_{"/orchestrator/export_fen"};
  std::string check_feasibility_srv_name_{"/workspace/check_move_feasibility"};
  std::string execute_collection_task_srv_name_{"/orchestrator/execute_collection_task"};

  // Episode state & storage
  std::atomic<EpisodeState> current_state_{EpisodeState::IDLE};
  std::atomic<CampaignState> campaign_state_{CampaignState::DISABLED};
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
  std::mutex domain_mutex_;

  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;
  rclcpp::Subscription<lekiwi_interfaces::msg::ChessGameStatus>::SharedPtr game_status_sub_;
  rclcpp::Client<lekiwi_interfaces::srv::SetFen>::SharedPtr import_fen_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr export_fen_client_;
  rclcpp::Client<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr check_feasibility_client_;
  rclcpp::Client<lekiwi_interfaces::srv::ExecuteCollectionTask>::SharedPtr execute_collection_task_client_;

  std::string last_seen_fen_{""};
  bool board_is_stable_{false};
  std::atomic<bool> is_planning_in_progress_{false};

  // Diagnostics Updater
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
};

} // namespace chess_episode_recorder

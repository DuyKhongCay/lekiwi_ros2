// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "chess_episode_recorder/episode_recorder_node.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <rmw/rmw.h>

namespace chess_episode_recorder
{

namespace
{
lekiwi_interfaces::msg::ChessMoveDetails to_ros_move_details(
  const domain::MoveCandidate &cand,
  const chess::Board &board)
{
  lekiwi_interfaces::msg::ChessMoveDetails details;
  details.uci = cand.uci;

  auto moving = cand.moving_piece(board);
  details.piece_color = (moving.color == chess::Color::WHITE) ? "white" : "black";
  details.piece_type = moving.piece_name();

  details.is_capture = cand.is_capture(board);
  details.is_en_passant = cand.is_en_passant();
  details.is_castling = cand.is_castling();
  details.promotion_piece = cand.promotion_piece_name();

  if (details.is_capture) {
    auto cap = cand.captured_piece(board);
    if (cap) {
      details.piece_type += ":" + cap->piece_name();
    }
  }

  return details;
}
} // namespace

EpisodeRecorderNode::EpisodeRecorderNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("episode_recorder_node", options)
{
  init_parameters();
  init_services();
  init_campaign_conductor();
  init_diagnostics();

  discovery_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EpisodeRecorderNode::discover_and_subscribe_topics, this));

  if (!feedback_topic_.empty()) {
    feedback_sub_ = this->create_subscription<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>(
      feedback_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&EpisodeRecorderNode::on_feedback_received, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening for move execution feedback on '%s'", feedback_topic_.c_str());
  }
}

EpisodeRecorderNode::~EpisodeRecorderNode()
{
  is_running_.store(false);
  if (current_state_.load() == EpisodeState::RECORDING) {
    stop_episode();
    save_episode();
  }
}

void EpisodeRecorderNode::init_parameters()
{
  root_dir_ = this->declare_parameter<std::string>("root_dir", "~/lekiwi_dataset");
  experiment_name_ = this->declare_parameter<std::string>("experiment_name", "chess_smolvla_v1");
  task_ = this->declare_parameter<std::string>("task", "");
  storage_id_ = this->declare_parameter<std::string>("storage_id", "mcap");
  feedback_topic_ = this->declare_parameter<std::string>(
    "feedback_topic", "/manipulation/execute_chess_move/_action/feedback");

  requested_topics_ = this->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>{
      "/camera/wrist/image_raw/compressed",
      "/camera/overhead/image_raw/compressed",
      "/joint_states",
      "/dynamixel/gripper_status",
      "/cmd_vel",
      "/arm/cartesian_jog/command",
      "/gamepad/cmd_jog"
    });

  enable_collection_planner_ = this->declare_parameter<bool>("enable_collection_planner", false);
  campaign_config_.total_episodes = static_cast<uint32_t>(this->declare_parameter<int64_t>("campaign.total_episodes", 600));
  campaign_config_.episodes_per_session = static_cast<uint32_t>(this->declare_parameter<int64_t>("campaign.episodes_per_session", 20));
  campaign_config_.move_total = static_cast<uint32_t>(this->declare_parameter<int64_t>("quotas.move.total", 240));
  campaign_config_.clear_total = static_cast<uint32_t>(this->declare_parameter<int64_t>("quotas.clear.total", 120));
  campaign_config_.pick_total = static_cast<uint32_t>(this->declare_parameter<int64_t>("quotas.pick.total", 120));
  campaign_config_.place_total = static_cast<uint32_t>(this->declare_parameter<int64_t>("quotas.place.total", 120));

  service_timeout_sec_ = this->declare_parameter<double>("service_timeout_sec", 5.0);
  game_status_topic_ = this->declare_parameter<std::string>("game_status_topic", "/chess/game_status");
  import_fen_srv_name_ = this->declare_parameter<std::string>("import_fen_service", "/orchestrator/import_fen");
  check_feasibility_srv_name_ = this->declare_parameter<std::string>("check_feasibility_service", "/workspace/check_move_feasibility");
  execute_collection_task_action_name_ = this->declare_parameter<std::string>("execute_collection_task_action", "/orchestrator/execute_collection_task");

  if (!root_dir_.empty() && root_dir_[0] == '~') {
    const char *home = std::getenv("HOME");
    if (home) {
      root_dir_.replace(0, 1, home);
    }
  }

  base_output_dir_ = std::filesystem::path(root_dir_);
  if (!experiment_name_.empty()) {
    base_output_dir_ /= experiment_name_;
  }

  std::error_code ec;
  std::filesystem::create_directories(base_output_dir_, ec);

  current_episode_index_ = storage::DatasetDirectoryScanner::scan_next_episode_index(base_output_dir_);
  RCLCPP_INFO(get_logger(), "Ready. Next episode index will be: %06u", current_episode_index_);
}

void EpisodeRecorderNode::init_services()
{
  srv_start_ = this->create_service<std_srvs::srv::Trigger>(
    "~/start_episode",
    std::bind(&EpisodeRecorderNode::handle_start, this, std::placeholders::_1, std::placeholders::_2));

  srv_save_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_episode",
    std::bind(&EpisodeRecorderNode::handle_save, this, std::placeholders::_1, std::placeholders::_2));

  srv_discard_ = this->create_service<std_srvs::srv::Trigger>(
    "~/discard_episode",
    std::bind(&EpisodeRecorderNode::handle_discard, this, std::placeholders::_1, std::placeholders::_2));

  srv_campaign_status_ = this->create_service<std_srvs::srv::Trigger>(
    "~/get_campaign_status",
    std::bind(&EpisodeRecorderNode::handle_campaign_status, this, std::placeholders::_1, std::placeholders::_2));
}

void EpisodeRecorderNode::init_campaign_conductor()
{
  if (!enable_collection_planner_) {
    RCLCPP_INFO(get_logger(), "Automated collection campaign planner is DISABLED (Manual Teleop Mode).");
    return;
  }

  conductor_ = std::make_unique<CampaignConductor>(
    campaign_config_, base_output_dir_, experiment_name_, true);

  cb_group_clients_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  import_fen_client_ = this->create_client<lekiwi_interfaces::srv::SetFen>(
    import_fen_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);

  check_feasibility_client_ = this->create_client<lekiwi_interfaces::srv::CheckMoveFeasibility>(
    check_feasibility_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);

  action_client_ = rclcpp_action::create_client<ExecuteCollectionTask>(
    this, execute_collection_task_action_name_, cb_group_clients_);

  game_status_sub_ = this->create_subscription<lekiwi_interfaces::msg::ChessGameStatus>(
    game_status_topic_,
    rclcpp::QoS(1).transient_local(),
    std::bind(&EpisodeRecorderNode::on_game_status_received, this, std::placeholders::_1));

  uint32_t recovered = conductor_->initialize_from_storage(false);
  auto summary = conductor_->get_quota_summary();

  RCLCPP_INFO(
    get_logger(),
    "🏆 [CAMPAIGN CONDUCTOR] Initialized. Recovered %u episodes from disk. Total: %u/%u (MOVE: %u, CLEAR: %u, PICK: %u, PLACE: %u)",
    recovered, summary.total_completed, summary.total_target,
    summary.completed_by_kind.at(domain::StepKind::MOVE),
    summary.completed_by_kind.at(domain::StepKind::CLEAR),
    summary.completed_by_kind.at(domain::StepKind::PICK),
    summary.completed_by_kind.at(domain::StepKind::PLACE));

  const auto target_fen = conductor_->get_target_session_fen();
  if (!target_fen.empty()) {
    sync_active_fen_to_orchestrator(target_fen);
  }

  action_server_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EpisodeRecorderNode::check_action_server_and_dispatch, this));
}

void EpisodeRecorderNode::init_diagnostics()
{
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID(experiment_name_.empty() ? "lekiwi_episode_recorder" : experiment_name_);

  diag_updater_->add("Episode Recorder Status", this, &EpisodeRecorderNode::produce_recorder_diagnostics);
  diag_updater_->add("Collection Campaign Status", this, &EpisodeRecorderNode::produce_campaign_diagnostics);
}

void EpisodeRecorderNode::discover_and_subscribe_topics()
{
  auto topic_names_and_types = this->get_topic_names_and_types();

  for (const auto &req_topic : requested_topics_) {
    if (subscriptions_.find(req_topic) != subscriptions_.end()) {
      continue;
    }

    auto it = topic_names_and_types.find(req_topic);
    if (it != topic_names_and_types.end() && !it->second.empty()) {
      const std::string topic_type = it->second[0];
      topic_type_map_[req_topic] = topic_type;

      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.ignore_local_publications = true;

      auto sub = this->create_generic_subscription(
        req_topic,
        topic_type,
        rclcpp::SensorDataQoS(),
        [this, req_topic, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
          write_serialized_message(req_topic, topic_type, msg);
        },
        sub_opts);

      subscriptions_[req_topic] = sub;
      RCLCPP_INFO(get_logger(), "Subscribed to recording topic: %s [%s]",
                  req_topic.c_str(), topic_type.c_str());
    }
  }

  if (subscriptions_.size() == requested_topics_.size()) {
    discovery_timer_->cancel();
  }
}

void EpisodeRecorderNode::write_serialized_message(
  const std::string &topic_name,
  const std::string &topic_type,
  std::shared_ptr<const rclcpp::SerializedMessage> msg)
{
  if (current_state_.load() != EpisodeState::RECORDING) {
    return;
  }
  writer_.write_serialized_message(topic_name, topic_type, msg, this->now());
}

bool EpisodeRecorderNode::start_episode()
{
  if (current_state_.load() == EpisodeState::RECORDING) {
    RCLCPP_WARN(get_logger(), "Already recording episode %06u. Stop first!", current_episode_index_);
    return false;
  }

  if (current_state_.load() == EpisodeState::STOPPED) {
    RCLCPP_INFO(get_logger(), "Auto-saving previous episode before starting new one...");
    save_episode();
  }

  current_episode_index_ = storage::DatasetDirectoryScanner::scan_next_episode_index(base_output_dir_);
  current_episode_dir_ = storage::DatasetDirectoryScanner::make_episode_dir(base_output_dir_, current_episode_index_);

  std::string task_str = task_;
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      task_str = current_instruction_;
    }
  }

  if (!writer_.open_episode(
        current_episode_dir_, current_episode_index_, experiment_name_, task_str,
        topic_type_map_, rmw_get_serialization_format()))
  {
    RCLCPP_ERROR(get_logger(), "Failed to open MCAP writer for episode %06u", current_episode_index_);
    return false;
  }

  episode_start_time_ = this->now();
  current_state_.store(EpisodeState::RECORDING);

  RCLCPP_INFO(get_logger(), "▶ [EPISODE RECORDER] Started Episode %06u -> %s",
              current_episode_index_, current_episode_dir_.c_str());
  return true;
}

bool EpisodeRecorderNode::stop_episode()
{
  if (current_state_.load() != EpisodeState::RECORDING) {
    RCLCPP_WARN(get_logger(), "Cannot stop: Not currently in RECORDING state.");
    return false;
  }

  current_state_.store(EpisodeState::STOPPED);
  episode_stop_time_ = this->now();
  writer_.close();

  double duration = (episode_stop_time_ - episode_start_time_).seconds();
  RCLCPP_INFO(get_logger(),
              "⏸ [EPISODE RECORDER] Stopped Episode %06u (Duration: %.2fs). "
              "Press [RIGHT: Save] or [LEFT: Discard].",
              current_episode_index_, duration);
  return true;
}

bool EpisodeRecorderNode::save_episode()
{
  if (current_state_.load() != EpisodeState::STOPPED) {
    if (current_state_.load() == EpisodeState::RECORDING) {
      stop_episode();
    } else {
      RCLCPP_WARN(get_logger(), "Cannot save: No episode has been recorded yet.");
      return false;
    }
  }

  double duration = (episode_stop_time_ - episode_start_time_).seconds();
  std::string task_saved = task_;
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      task_saved = current_instruction_;
    }
  }

  storage::EpisodeMetadata meta;
  meta.episode_id = current_episode_index_;
  meta.experiment_name = experiment_name_;
  meta.task = task_saved;
  meta.duration_sec = duration;
  meta.timestamp_start = episode_start_time_.seconds();
  meta.timestamp_end = episode_stop_time_.seconds();
  meta.message_counts = writer_.get_message_counts();

  bool ok = storage::DatasetDirectoryScanner::write_episode_info(current_episode_dir_, meta);
  if (ok) {
    if (conductor_) {
      conductor_->on_episode_recorded(task_saved);
      auto sum = conductor_->get_quota_summary();
      auto sess = conductor_->get_session_state();
      RCLCPP_INFO(
        get_logger(),
        "✔ [EPISODE SAVED] Episode %06u! Campaign: %u/%u (MOVE: %u, CLEAR: %u, PICK: %u, PLACE: %u). Session: %u/20. Move: %u/%u",
        current_episode_index_, sum.total_completed, sum.total_target,
        sum.completed_by_kind.at(domain::StepKind::MOVE),
        sum.completed_by_kind.at(domain::StepKind::CLEAR),
        sum.completed_by_kind.at(domain::StepKind::PICK),
        sum.completed_by_kind.at(domain::StepKind::PLACE),
        sess.episodes_in_session,
        conductor_->get_active_move_recorded_episodes(),
        conductor_->get_active_move_expected_episodes());
    } else {
      RCLCPP_INFO(get_logger(), "✔ [EPISODE RECORDER] Saved Episode %06u successfully!",
                  current_episode_index_);
    }

    current_episode_index_++;
    current_state_.store(EpisodeState::IDLE);
    {
      std::lock_guard<std::mutex> lock(instruction_mutex_);
      current_instruction_.clear();
    }
    return true;
  }

  RCLCPP_ERROR(get_logger(), "Failed to write episode metadata for %06u", current_episode_index_);
  return false;
}

bool EpisodeRecorderNode::discard_episode()
{
  if (current_state_.load() == EpisodeState::RECORDING) {
    current_state_.store(EpisodeState::IDLE);
    writer_.close();
  }

  std::error_code ec;
  if (std::filesystem::exists(current_episode_dir_, ec)) {
    std::filesystem::remove_all(current_episode_dir_, ec);
    RCLCPP_WARN(get_logger(), "🗑 [EPISODE RECORDER] Discarded & deleted Episode %06u.",
                current_episode_index_);
  }
  current_state_.store(EpisodeState::IDLE);
  return true;
}

CampaignState EpisodeRecorderNode::get_campaign_state() const
{
  if (!conductor_) {
    return CampaignState::DISABLED;
  }
  return conductor_->get_campaign_state();
}

bool EpisodeRecorderNode::is_action_server_connected() const
{
  return action_client_ && action_client_->action_server_is_ready();
}

bool EpisodeRecorderNode::is_board_synced() const
{
  return conductor_ && conductor_->is_board_synced();
}

bool EpisodeRecorderNode::has_active_goal() const
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  return active_goal_handle_ != nullptr;
}

domain::SessionState EpisodeRecorderNode::get_session_state() const
{
  if (!conductor_) {
    return domain::SessionState{};
  }
  return conductor_->get_session_state();
}

void EpisodeRecorderNode::produce_recorder_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  const auto snapshot = capture_diagnostics_snapshot();
  RecorderDiagnostics::evaluate_recorder_status(snapshot, stat);
}

void EpisodeRecorderNode::produce_campaign_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  const auto snapshot = capture_diagnostics_snapshot();
  RecorderDiagnostics::evaluate_campaign_status(snapshot, stat);
}

RecorderDiagnosticsSnapshot EpisodeRecorderNode::capture_diagnostics_snapshot() const
{
  const auto disk_info = storage::query_disk_space(base_output_dir_);
  std::string active_ins;
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    active_ins = current_instruction_;
  }

  if (conductor_) {
    return conductor_->capture_diagnostics_snapshot(
      disk_info,
      current_state_.load(),
      current_episode_index_,
      writer_.get_total_messages(),
      active_ins,
      is_action_server_connected());
  }

  RecorderDiagnosticsSnapshot snap;
  snap.episode_state = current_state_.load();
  snap.campaign_state = CampaignState::DISABLED;
  snap.enable_collection_planner = false;
  snap.is_action_server_connected = false;
  snap.has_active_goal = false;
  snap.current_episode_index = current_episode_index_;
  snap.base_output_dir = base_output_dir_.string();
  snap.active_instruction = active_ins;
  snap.total_recorded_messages = writer_.get_total_messages();
  snap.available_disk_space_bytes = disk_info.available_bytes;
  snap.disk_query_success = disk_info.success;
  return snap;
}

bool EpisodeRecorderNode::sync_active_fen_to_orchestrator(const std::string &fen)
{
  if (conductor_) {
    conductor_->sync_active_fen(fen);
  }
  fen_sync_in_flight_.store(false);
  if (import_fen_client_ && import_fen_client_->service_is_ready()) {
    return send_fen_sync_request();
  }
  return false;
}

bool EpisodeRecorderNode::send_fen_sync_request()
{
  if (!conductor_ || !import_fen_client_ || !import_fen_client_->service_is_ready()) {
    return false;
  }
  if (fen_sync_in_flight_.exchange(true)) {
    return false;
  }

  const std::string fen = conductor_->get_target_session_fen();
  if (fen.empty()) {
    fen_sync_in_flight_.store(false);
    return false;
  }

  auto req = std::make_shared<lekiwi_interfaces::srv::SetFen::Request>();
  req->fen = fen;

  import_fen_client_->async_send_request(
    req,
    [this, fen](rclcpp::Client<lekiwi_interfaces::srv::SetFen>::SharedFuture fut) {
      try {
        auto resp = fut.get();
        if (resp->success) {
          if (conductor_) {
            conductor_->set_fen_sync_acknowledged(true);
          }
          RCLCPP_INFO(
            get_logger(),
            "✔ [STATE BARRIER] Orchestrator acknowledged FEN: %s. Awaiting physical board sync.",
            fen.c_str());
        } else {
          if (conductor_) {
            conductor_->set_fen_sync_acknowledged(false);
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "⚠ [STATE BARRIER] Orchestrator SetFen rejected: %s (Will retry)",
            resp->message.c_str());
        }
      } catch (const std::exception &e) {
        if (conductor_) {
          conductor_->set_fen_sync_acknowledged(false);
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "⚠ [STATE BARRIER] Exception in SetFen response: %s (Will retry)", e.what());
      }
      fen_sync_in_flight_.store(false);
    });

  return true;
}

void EpisodeRecorderNode::advance_to_new_session()
{
  if (!conductor_) {
    return;
  }

  conductor_->advance_to_new_session();
  const auto sess = conductor_->get_session_state();
  const auto new_fen = conductor_->get_target_session_fen();

  RCLCPP_INFO(
    get_logger(),
    "\n================================================================================\n"
    "📢 [NEW COLLECTION SESSION] Session #%u Initialized (Target: 20 eps)\n"
    "Active FEN: %s\n"
    "Operator: Please arrange the physical chessboard pieces to match the FEN,\n"
    "or observe the 2D visualizer on /chess/board_2d/compressed!\n"
    "================================================================================",
    sess.session_id, new_fen.c_str());

  send_fen_sync_request();
}

void EpisodeRecorderNode::on_game_status_received(
  const lekiwi_interfaces::msg::ChessGameStatus::ConstSharedPtr msg)
{
  if (!msg || !conductor_) {
    return;
  }
  conductor_->set_board_synced(msg->is_board_synced);
}

void EpisodeRecorderNode::check_action_server_and_dispatch()
{
  if (!enable_collection_planner_ || !conductor_ || !is_running_.load()) {
    return;
  }

  if (conductor_->get_campaign_state() == CampaignState::AWAITING_PHYSICAL_SYNC) {
    if (!conductor_->is_fen_sync_acknowledged() && !fen_sync_in_flight_.load()) {
      send_fen_sync_request();
    }
    return;
  }

  if (conductor_->get_campaign_state() != CampaignState::IDLE_READY || has_active_goal()) {
    return;
  }

  if (!action_client_ || !action_client_->action_server_is_ready()) {
    return;
  }

  dispatch_next_task();
}

void EpisodeRecorderNode::dispatch_next_task()
{
  if (!conductor_ || !is_running_.load()) {
    return;
  }

  // Phase 1: Snapshot candidate generation without blocking lock during query
  auto snapshot = conductor_->prepare_planning_snapshot();
  if (!snapshot) {
    return;
  }

  // Feasibility query checker evaluated outside conductor state mutex lock
  domain::MoveCandidateSelector::FeasibilityChecker checker =
    [this, &snapshot](const domain::MoveCandidate &cand) -> std::pair<bool, uint8_t> {
      if (!check_feasibility_client_ || !check_feasibility_client_->service_is_ready()) {
        return {false, 0};
      }

      auto req = std::make_shared<lekiwi_interfaces::srv::CheckMoveFeasibility::Request>();
      req->move = to_ros_move_details(cand, snapshot->current_board);

      auto future = check_feasibility_client_->async_send_request(req);
      auto timeout = std::chrono::duration<double>(service_timeout_sec_);
      if (future.wait_for(timeout) != std::future_status::ready) {
        return {false, 0};
      }

      try {
        auto resp = future.get();
        return {resp->feasible, resp->plan_type};
      } catch (...) {
        return {false, 0};
      }
    };

  domain::MoveCandidateSelector selector;
  auto best_move = selector.select_best_move(
    snapshot->current_board, snapshot->tracker, checker);

  if (!best_move.has_value()) {
    conductor_->abort_planning();
    return;
  }

  // Phase 2: Commit selected candidate and transition to DISPATCHED_TO_ORCH
  auto dispatched = conductor_->commit_selected_candidate(*best_move);
  if (!dispatched.has_value()) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "🎯 [CANDIDATE SELECTED] Move: %s (Score: %.1f, Feasible: %s, Plan: %u, Expected Episodes: %u)",
    best_move->uci.c_str(), best_move->score, best_move->is_feasible ? "YES" : "NO",
    best_move->plan_type, dispatched->expected_episodes);

  ExecuteCollectionTask::Goal goal;
  goal.move = to_ros_move_details(*best_move, snapshot->current_board);
  goal.force_pick_place = best_move->force_pick_place;
  goal.clear_bin_side = best_move->clear_bin_side;
  goal.bypass_steps = best_move->bypass_steps;

  rclcpp_action::Client<ExecuteCollectionTask>::SendGoalOptions options;
  options.goal_response_callback =
    std::bind(&EpisodeRecorderNode::on_goal_response, this, std::placeholders::_1);
  options.feedback_callback =
    std::bind(&EpisodeRecorderNode::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback =
    std::bind(&EpisodeRecorderNode::on_result, this, std::placeholders::_1);

  action_client_->async_send_goal(goal, options);
}

void EpisodeRecorderNode::on_goal_response(
  const GoalHandleCollectionTask::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "⚠ [COLLECTION TASK] Goal rejected by Orchestrator.");
    if (conductor_) {
      conductor_->on_goal_rejected();
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_ = goal_handle;
  }
  RCLCPP_INFO(get_logger(), "✔ [COLLECTION TASK] Goal accepted by Orchestrator. Execution in progress...");
}

void EpisodeRecorderNode::on_feedback(
  GoalHandleCollectionTask::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const ExecuteCollectionTask::Feedback> feedback)
{
  if (!feedback) {
    return;
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "📊 [COLLECTION FEEDBACK] Stage: %s, Progress: %.1f%%",
    feedback->current_stage.c_str(), feedback->progress_percent);
}

void EpisodeRecorderNode::on_result(const GoalHandleCollectionTask::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
  }

  if (!conductor_) {
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      const std::string actual_fen =
        (result.result && !result.result->final_fen.empty()) ? result.result->final_fen : "";

      RCLCPP_INFO(
        get_logger(),
        "🎉 [COLLECTION TASK] Succeeded! Final FEN: %s",
        actual_fen.empty() ? "(empty)" : actual_fen.c_str());

      auto outcome = conductor_->on_collection_task_finished(true, actual_fen);
      if (outcome == domain::BoardTransitionOutcome::DIVERGED_RESYNCED) {
        RCLCPP_WARN(
          get_logger(),
          "⚠ [STATE GATE] Teleop move divergence detected! Re-syncing Orchestrator to actual FEN: %s",
          actual_fen.c_str());
        send_fen_sync_request();
      }
      trigger_deferred_dispatch();
      break;
    }
    case rclcpp_action::ResultCode::ABORTED: {
      RCLCPP_ERROR(
        get_logger(),
        "❌ [COLLECTION TASK] Aborted by Orchestrator: %s",
        result.result ? result.result->message.c_str() : "No message");
      conductor_->on_goal_aborted_or_canceled();
      trigger_deferred_dispatch();
      break;
    }
    case rclcpp_action::ResultCode::CANCELED: {
      RCLCPP_WARN(get_logger(), "⚠ [COLLECTION TASK] Canceled by Orchestrator or client request.");
      conductor_->on_goal_aborted_or_canceled();
      trigger_deferred_dispatch();
      break;
    }
    default:
      RCLCPP_WARN(get_logger(), "Unknown action result code: %d", static_cast<int>(result.code));
      conductor_->on_goal_aborted_or_canceled();
      break;
  }
}

void EpisodeRecorderNode::trigger_deferred_dispatch()
{
  defer_dispatch_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() {
      defer_dispatch_timer_->cancel();
      check_action_server_and_dispatch();
    });
}

void EpisodeRecorderNode::on_feedback_received(
  const lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(instruction_mutex_);
  if (!msg->feedback.instruction.empty()) {
    current_instruction_ = msg->feedback.instruction;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "📥 [FEEDBACK CACHED] Task instruction: '%s' (Phase: %s, Progress: %.1f%%)",
      current_instruction_.c_str(),
      msg->feedback.current_phase.c_str(),
      msg->feedback.progress_percent);
  }
}

void EpisodeRecorderNode::handle_start(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = start_episode();
  res->message = res->success ? "Episode recording started." : "Failed to start episode.";
}

void EpisodeRecorderNode::handle_save(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = save_episode();
  res->message = res->success ? "Episode saved successfully." : "Failed to save episode.";
}

void EpisodeRecorderNode::handle_discard(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = discard_episode();
  res->message = res->success ? "Episode discarded." : "Failed to discard episode.";
}

void EpisodeRecorderNode::handle_campaign_status(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!enable_collection_planner_ || !conductor_) {
    res->success = false;
    res->message = "Collection planner is disabled.";
    return;
  }

  const auto summary = conductor_->get_quota_summary();
  const auto session = conductor_->get_session_state();

  std::ostringstream ss;
  ss << "Campaign: " << summary.total_completed << "/" << summary.total_target
     << " (MOVE: " << summary.completed_by_kind.at(domain::StepKind::MOVE)
     << ", CLEAR: " << summary.completed_by_kind.at(domain::StepKind::CLEAR)
     << ", PICK: " << summary.completed_by_kind.at(domain::StepKind::PICK)
     << ", PLACE: " << summary.completed_by_kind.at(domain::StepKind::PLACE) << ")"
     << " | Session " << session.session_id << ": " << session.episodes_in_session << "/20"
     << " | State: " << to_string(conductor_->get_campaign_state())
     << " | FEN: " << session.active_fen;

  res->success = true;
  res->message = ss.str();
}

} // namespace chess_episode_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(chess_episode_recorder::EpisodeRecorderNode)

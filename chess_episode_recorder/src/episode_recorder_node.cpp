// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "chess_episode_recorder/episode_recorder_node.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <rmw/rmw.h>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

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
  details.piece_color = (moving.color == chess::Color::WHITE) ? "w" : "b";
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

  // Create timer to periodically resolve topic types and create subscriptions
  discovery_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EpisodeRecorderNode::discover_and_subscribe_topics, this));

  RCLCPP_INFO(
    get_logger(),
    "EpisodeRecorderNode initialized. Base output dir: %s, Experiment: %s",
    base_output_dir_.c_str(), experiment_name_.c_str());
}

EpisodeRecorderNode::~EpisodeRecorderNode()
{
  is_running_.store(false);

  if (action_server_check_timer_) {
    action_server_check_timer_->cancel();
  }

  if (current_state_.load() == EpisodeState::RECORDING) {
    RCLCPP_WARN(get_logger(), "Shutting down while recording. Stopping active episode...");
    stop_episode();
  }
}

void EpisodeRecorderNode::init_parameters()
{
  this->declare_parameter<std::string>("root_dir", "/tmp/lekiwi_episodes");
  this->declare_parameter<std::string>("experiment_name", "chess_teleop_v1");
  this->declare_parameter<std::string>("task", "chess_pick_and_place");
  this->declare_parameter<std::string>("storage_id", "mcap");
  this->declare_parameter<std::string>(
    "feedback_topic", "/manipulation/execute_chess_move/_action/feedback");
  this->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>{
      "/cameras/usb_wrist/image_raw/compressed",
      "/cameras/usb_side/image_raw/compressed",
      "/cameras/stereo_right/image_raw/compressed",
      "/joint_states",
      "/arm_trajectory_controller/joint_trajectory",
      "/tf",
      "/tf_static",
      "/manipulation/execute_chess_move/_action/feedback"});

  // Campaign Conductor Parameters
  this->declare_parameter<bool>("enable_collection_planner", false);
  this->declare_parameter<int64_t>("campaign.total_episodes", 600);
  this->declare_parameter<int64_t>("campaign.episodes_per_session", 20);
  this->declare_parameter<int64_t>("quotas.move.total", 240);
  this->declare_parameter<int64_t>("quotas.clear.total", 120);
  this->declare_parameter<int64_t>("quotas.pick.total", 120);
  this->declare_parameter<int64_t>("quotas.place.total", 120);

  this->declare_parameter<std::string>("game_status_topic", "/chess/game_status");
  this->declare_parameter<std::string>("import_fen_service", "/orchestrator/import_fen");
  this->declare_parameter<std::string>(
    "check_feasibility_service", "/workspace/check_move_feasibility");
  this->declare_parameter<std::string>(
    "execute_collection_task_action", "/orchestrator/execute_collection_task");
  this->declare_parameter<double>("service_timeout_sec", 5.0);

  root_dir_ = this->get_parameter("root_dir").as_string();
  experiment_name_ = this->get_parameter("experiment_name").as_string();
  task_ = this->get_parameter("task").as_string();
  storage_id_ = this->get_parameter("storage_id").as_string();
  feedback_topic_ = this->get_parameter("feedback_topic").as_string();
  requested_topics_ = this->get_parameter("topics").as_string_array();

  enable_collection_planner_ = this->get_parameter("enable_collection_planner").as_bool();
  campaign_config_.total_episodes =
    static_cast<uint32_t>(this->get_parameter("campaign.total_episodes").as_int());
  campaign_config_.episodes_per_session =
    static_cast<uint32_t>(this->get_parameter("campaign.episodes_per_session").as_int());
  campaign_config_.move_total =
    static_cast<uint32_t>(this->get_parameter("quotas.move.total").as_int());
  campaign_config_.clear_total =
    static_cast<uint32_t>(this->get_parameter("quotas.clear.total").as_int());
  campaign_config_.pick_total =
    static_cast<uint32_t>(this->get_parameter("quotas.pick.total").as_int());
  campaign_config_.place_total =
    static_cast<uint32_t>(this->get_parameter("quotas.place.total").as_int());

  game_status_topic_ = this->get_parameter("game_status_topic").as_string();
  import_fen_srv_name_ = this->get_parameter("import_fen_service").as_string();
  check_feasibility_srv_name_ = this->get_parameter("check_feasibility_service").as_string();
  execute_collection_task_action_name_ = this->get_parameter("execute_collection_task_action").as_string();
  service_timeout_sec_ = this->get_parameter("service_timeout_sec").as_double();

  base_output_dir_ = std::filesystem::path(root_dir_);
  if (!experiment_name_.empty()) {
    base_output_dir_ /= experiment_name_;
  }

  try {
    std::filesystem::create_directories(base_output_dir_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to create output directory '%s': %s",
                 base_output_dir_.c_str(), e.what());
  }

  current_episode_index_ = scan_next_episode_index(base_output_dir_);
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

  feedback_sub_ = this->create_subscription<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>(
    feedback_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&EpisodeRecorderNode::on_feedback_received, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Listening for move instruction on feedback topic: %s", feedback_topic_.c_str());
}

void EpisodeRecorderNode::init_campaign_conductor()
{
  if (!enable_collection_planner_) {
    campaign_state_.store(CampaignState::DISABLED);
    RCLCPP_INFO(get_logger(), "Automated Collection Planner is DISABLED. Manual teleop mode active.");
    return;
  }

  cb_group_clients_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  quota_tracker_ = std::make_unique<domain::QuotaTracker>(campaign_config_);
  fen_synthesizer_ = std::make_unique<domain::FenSynthesizer>();
  move_selector_ = std::make_unique<domain::MoveCandidateSelector>();

  // Recover progress from existing episodes
  uint32_t recovered = quota_tracker_->scan_and_recover(base_output_dir_);
  auto summary = quota_tracker_->get_summary();
  RCLCPP_INFO(
    get_logger(),
    "🏆 [CAMPAIGN CONDUCTOR] Initialized. Recovered %u episodes from disk. Total: %u/%u (MOVE: %u, CLEAR: %u, PICK: %u, PLACE: %u)",
    recovered, summary.total_completed, summary.total_target,
    summary.completed_by_kind[domain::StepKind::MOVE],
    summary.completed_by_kind[domain::StepKind::CLEAR],
    summary.completed_by_kind[domain::StepKind::PICK],
    summary.completed_by_kind[domain::StepKind::PLACE]);

  // Load session state if it exists
  const auto session_file = base_output_dir_ / "session_state.yaml";
  bool session_loaded = quota_tracker_->load_session_state(session_file);
  auto sess = quota_tracker_->get_session_state();

  if (session_loaded && sess.is_active) {
    RCLCPP_INFO(
      get_logger(),
      "💾 [CAMPAIGN CONDUCTOR] Resumed active session #%u (Active FEN: %s, %u/%u eps, Executed Moves: %zu)",
      sess.session_id, sess.active_fen.c_str(), sess.episodes_in_session,
      campaign_config_.episodes_per_session, sess.executed_moves.size());
  }

  // Create ROS 2 service, action clients, and game status subscription
  import_fen_client_ = this->create_client<lekiwi_interfaces::srv::SetFen>(
    import_fen_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);
  check_feasibility_client_ = this->create_client<lekiwi_interfaces::srv::CheckMoveFeasibility>(
    check_feasibility_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);
  action_client_ = rclcpp_action::create_client<ExecuteCollectionTask>(
    this, execute_collection_task_action_name_, cb_group_clients_);

  game_status_sub_ = this->create_subscription<lekiwi_interfaces::msg::ChessGameStatus>(
    game_status_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&EpisodeRecorderNode::on_game_status_received, this, std::placeholders::_1));

  if (!sess.is_active || sess.episodes_in_session >= campaign_config_.episodes_per_session) {
    advance_to_new_session();
  } else {
    sync_active_fen_to_orchestrator(sess.active_fen);
  }

  // Periodic check timer to dispatch task when action server is online
  action_server_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EpisodeRecorderNode::check_action_server_and_dispatch, this));

  RCLCPP_INFO(
    get_logger(),
    "⏳ [CAMPAIGN CONDUCTOR] Initialized Action Client on '%s'.",
    execute_collection_task_action_name_.c_str());
}

bool EpisodeRecorderNode::is_action_server_connected() const
{
  return action_client_ && action_client_->action_server_is_ready();
}

bool EpisodeRecorderNode::has_active_goal() const
{
  std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(goal_mutex_));
  return active_goal_handle_ != nullptr;
}

void EpisodeRecorderNode::on_game_status_received(
  const lekiwi_interfaces::msg::ChessGameStatus::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  is_board_synced_.store(msg->is_board_synced);
}

void EpisodeRecorderNode::check_action_server_and_dispatch()
{
  if (!enable_collection_planner_ || !action_client_) {
    return;
  }

  // Handle State Barrier
  if (campaign_state_.load() == CampaignState::AWAITING_PHYSICAL_SYNC) {
    if (!fen_sync_acknowledged_.load()) {
      if (!import_fen_client_ || !import_fen_client_->service_is_ready()) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "⏳ [STATE BARRIER] Waiting for /orchestrator/import_fen service to become available...");
        return;
      }
      if (!fen_sync_in_flight_.load()) {
        send_fen_sync_request();
      }
      return; // Locked: Orchestrator has not acknowledged FEN yet
    }

    // Orchestrator accepted FEN; now waiting for physical board sync from vision
    std::string current_target_fen;
    {
      std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
      current_target_fen = target_session_fen_;
    }
    if (!is_board_synced_.load()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "⏳ [STATE BARRIER] Awaiting physical chessboard arrangement for session FEN: %s",
        current_target_fen.c_str());
      return; // Locked: Physical board not yet synced
    }

    // Both Orchestrator acknowledged FEN AND physical board is synced!
    RCLCPP_INFO(
      get_logger(),
      "✔ [STATE BARRIER CLEARED] Physical board and Orchestrator synced with FEN: %s. Campaign ready.",
      current_target_fen.c_str());
    campaign_state_.store(CampaignState::IDLE_READY);
  }

  if (campaign_state_.load() != CampaignState::IDLE_READY || has_active_goal()) {
    return;
  }

  if (!action_client_->action_server_is_ready()) {
    return;
  }
  if (!is_board_synced_.load()) {
    return;
  }

  dispatch_next_task();
}

bool EpisodeRecorderNode::query_move_feasibility_sync(
  const domain::MoveCandidate &cand,
  const chess::Board &board,
  uint8_t &out_plan_type)
{
  if (!is_running_.load()) {
    return false;
  }

  if (!check_feasibility_client_ || !check_feasibility_client_->service_is_ready()) {
    last_feasibility_query_succeeded_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Feasibility service '%s' is not ready yet.", check_feasibility_srv_name_.c_str());
    return false;
  }

  auto req = std::make_shared<lekiwi_interfaces::srv::CheckMoveFeasibility::Request>();
  req->move = to_ros_move_details(cand, board);

  auto future = check_feasibility_client_->async_send_request(req);
  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (future.wait_for(timeout) != std::future_status::ready) {
    last_feasibility_query_succeeded_.store(false);
    return false;
  }

  try {
    auto resp = future.get();
    out_plan_type = resp->plan_type;
    return resp->feasible;
  } catch (const std::exception &e) {
    last_feasibility_query_succeeded_.store(false);
    RCLCPP_WARN(get_logger(), "Feasibility query failed with exception: %s", e.what());
    return false;
  }
}

uint32_t EpisodeRecorderNode::calculate_expected_episodes(const domain::MoveCandidate &cand) const
{
  uint32_t expected = 0;
  auto is_bypassed = [&](const std::string &step) {
    return std::find(cand.bypass_steps.begin(), cand.bypass_steps.end(), step) != cand.bypass_steps.end();
  };

  if (quota_tracker_) {
    const auto &board = quota_tracker_->get_active_board();
    if (cand.is_capture(board)) {
      if (!is_bypassed("CLEAR")) {
        expected++;
      }
    }
  }

  if (cand.force_pick_place) {
    if (!is_bypassed("PICK")) {
      expected++;
    }
    if (!is_bypassed("PLACE")) {
      expected++;
    }
  } else {
    if (!is_bypassed("MOVE")) {
      expected++;
    }
  }

  return expected;
}

bool EpisodeRecorderNode::sync_active_fen_to_orchestrator(const std::string &fen)
{
  {
    std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
    target_session_fen_ = fen;
  }
  fen_sync_acknowledged_.store(false);
  fen_sync_in_flight_.store(false);
  is_board_synced_.store(false);
  campaign_state_.store(CampaignState::AWAITING_PHYSICAL_SYNC);

  if (import_fen_client_ && import_fen_client_->service_is_ready()) {
    return send_fen_sync_request();
  }
  return false;
}

bool EpisodeRecorderNode::send_fen_sync_request()
{
  if (!import_fen_client_ || !import_fen_client_->service_is_ready()) {
    return false;
  }
  if (fen_sync_in_flight_.exchange(true)) {
    return false;
  }

  std::string fen;
  {
    std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
    fen = target_session_fen_;
  }
  if (fen.empty()) {
    fen_sync_in_flight_.store(false);
    return false;
  }

  auto req = std::make_shared<lekiwi_interfaces::srv::SetFen::Request>();
  req->fen = fen;
  req->robot_color = ""; // Deduce color automatically

  import_fen_client_->async_send_request(
    req,
    [this, fen](rclcpp::Client<lekiwi_interfaces::srv::SetFen>::SharedFuture fut) {
      try {
        auto resp = fut.get();
        if (resp->success) {
          fen_sync_acknowledged_.store(true);
          RCLCPP_INFO(
            get_logger(),
            "✔ [STATE BARRIER] Orchestrator acknowledged FEN: %s. Awaiting physical board sync.",
            fen.c_str());
        } else {
          fen_sync_acknowledged_.store(false);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "⚠ [STATE BARRIER] Orchestrator SetFen rejected: %s (Will retry)",
            resp->message.c_str());
        }
      } catch (const std::exception &e) {
        fen_sync_acknowledged_.store(false);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "⚠ [STATE BARRIER] Exception in SetFen response: %s (Will retry)", e.what());
      }
      fen_sync_in_flight_.store(false);
    });
  return true;
}

void EpisodeRecorderNode::dispatch_next_task()
{
  if (!enable_collection_planner_ || !quota_tracker_ || !is_running_.load()) {
    return;
  }

  if (campaign_state_.load() != CampaignState::IDLE_READY) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cannot dispatch next task: Campaign state is %s",
      to_string(campaign_state_.load()).data());
    return;
  }

  if (!action_client_ || !action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ExecuteCollectionTask action server '%s' is not ready yet.",
      execute_collection_task_action_name_.c_str());
    return;
  }

  if (!check_feasibility_client_ || !check_feasibility_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cannot dispatch next task: Feasibility service '%s' is not ready yet.",
      check_feasibility_srv_name_.c_str());
    return;
  }

  if (!is_board_synced_.load()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cannot dispatch next task: Waiting for physical chessboard synchronization with FEN.");
    return;
  }

  if (has_active_goal()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot dispatch next task: A task is already actively being executed by Orchestrator.");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(domain_mutex_);

  if (quota_tracker_->is_campaign_complete()) {
    campaign_state_.store(CampaignState::CAMPAIGN_FINISHED);
    RCLCPP_INFO(
      get_logger(),
      "🎉 [CAMPAIGN COMPLETE] All %u target episodes achieved successfully!",
      campaign_config_.total_episodes);
    return;
  }

  auto sess = quota_tracker_->get_session_state();
  if (!sess.is_active || sess.episodes_in_session >= campaign_config_.episodes_per_session) {
    advance_to_new_session();
    return;
  }

  const auto &board = quota_tracker_->get_active_board();
  campaign_state_.store(CampaignState::PLANNING_OR_SYNTHESIS);
  last_feasibility_query_succeeded_.store(true);

  domain::MoveCandidateSelector::FeasibilityChecker checker =
    [this, &board](const domain::MoveCandidate &cand) -> std::pair<bool, uint8_t> {
      uint8_t plan_type = 0;
      bool feasible = query_move_feasibility_sync(cand, board, plan_type);
      return {feasible, plan_type};
    };

  auto best_move = move_selector_->select_best_move(board, *quota_tracker_, checker);
  if (best_move.has_value()) {
    active_move_candidate_ = best_move;
    active_move_expected_episodes_ = calculate_expected_episodes(*best_move);
    active_move_recorded_episodes_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "🎯 [CANDIDATE SELECTED] Move: %s (Score: %.1f, Feasible: %s, Plan: %u, Expected Episodes: %u)",
      best_move->uci.c_str(), best_move->score, best_move->is_feasible ? "YES" : "NO",
      best_move->plan_type, active_move_expected_episodes_);

    ExecuteCollectionTask::Goal goal;
    goal.move = to_ros_move_details(*best_move, board);
    goal.force_pick_place = best_move->force_pick_place;
    goal.clear_bin_side = best_move->clear_bin_side;
    goal.bypass_steps = best_move->bypass_steps;

    campaign_state_.store(CampaignState::DISPATCHED_TO_ORCH);

    rclcpp_action::Client<ExecuteCollectionTask>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&EpisodeRecorderNode::on_goal_response, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(&EpisodeRecorderNode::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&EpisodeRecorderNode::on_result, this, std::placeholders::_1);

    action_client_->async_send_goal(goal, options);
  } else {
    if (!last_feasibility_query_succeeded_.load()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Feasibility service timed out or failed during candidate evaluation. Retrying later without advancing session.");
      campaign_state_.store(CampaignState::IDLE_READY);
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Board exhausted: No contributing feasible moves found on active turn (%s). Advancing session...",
      (board.sideToMove() == chess::Color::WHITE ? "White" : "Black"));
    advance_to_new_session();
  }
}

void EpisodeRecorderNode::on_goal_response(const GoalHandleCollectionTask::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "⚠ [COLLECTION TASK] Goal rejected by Orchestrator.");
    {
      std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
      active_move_candidate_.reset();
      active_move_expected_episodes_ = 0;
      active_move_recorded_episodes_ = 0;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_handle_.reset();
    }
    campaign_state_.store(CampaignState::IDLE_READY);
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

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      const std::string actual_fen =
        (result.result && !result.result->final_fen.empty()) ? result.result->final_fen : "";

      RCLCPP_INFO(
        get_logger(),
        "🎉 [COLLECTION TASK] Succeeded! Final FEN: %s",
        actual_fen.empty() ? "(empty)" : actual_fen.c_str());

      std::string finished_uci;
      {
        std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
        if (active_move_candidate_.has_value()) {
          finished_uci = active_move_candidate_->uci;

          chess::Board expected_board = quota_tracker_->get_active_board();
          chess::Move expected_move = chess::uci::uciToMove(expected_board, finished_uci);
          bool move_valid = (expected_move != chess::Move::NO_MOVE);
          std::string expected_fen;
          if (move_valid) {
            expected_board.makeMove(expected_move);
            expected_fen = expected_board.getFen();
          }

          chess::Board actual_board;
          bool actual_valid = !actual_fen.empty() && actual_board.setFen(actual_fen);

          bool mismatch = !actual_valid || !move_valid || !(actual_board == expected_board);
          if (mismatch) {
            RCLCPP_WARN(
              get_logger(),
              "⚠ [STATE GATE] Teleop move divergence detected! Planned target: '%s' (Expected FEN: %s), "
              "Action Server reported FEN: %s. Preserving recorded episodes for post-processing and adopting actual board state.",
              finished_uci.c_str(), expected_fen.c_str(), actual_fen.c_str());

            if (actual_valid) {
              quota_tracker_->set_active_fen(actual_fen);
              quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");
              RCLCPP_INFO(
                get_logger(),
                "♟ [STATE RE-SYNC] Synchronized domain board to action server FEN: %s",
                actual_fen.c_str());
            }
          } else {
            bool applied = quota_tracker_->apply_move(finished_uci);
            if (applied) {
              RCLCPP_INFO(
                get_logger(),
                "♟ [MOVE COMPLETED] Applied UCI: %s. New Sequential FEN: %s",
                finished_uci.c_str(), quota_tracker_->get_active_fen().c_str());
              quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");
            } else {
              RCLCPP_ERROR(get_logger(), "❌ Failed to apply completed move '%s' to active board!", finished_uci.c_str());
            }
          }

          active_move_candidate_.reset();
          active_move_expected_episodes_ = 0;
          active_move_recorded_episodes_ = 0;
        }

        if (quota_tracker_->is_campaign_complete()) {
          campaign_state_.store(CampaignState::CAMPAIGN_FINISHED);
          RCLCPP_INFO(
            get_logger(),
            "🏆 [CAMPAIGN COMPLETE] All %u target episodes achieved successfully!",
            campaign_config_.total_episodes);
          return;
        }

        auto sess = quota_tracker_->get_session_state();
        if (sess.episodes_in_session >= campaign_config_.episodes_per_session) {
          advance_to_new_session();
          return;
        }
      }

      campaign_state_.store(CampaignState::IDLE_READY);
      trigger_deferred_dispatch();
      break;
    }
    case rclcpp_action::ResultCode::ABORTED: {
      RCLCPP_ERROR(
        get_logger(),
        "❌ [COLLECTION TASK] Aborted by Orchestrator: %s",
        result.result ? result.result->message.c_str() : "No message");
      {
        std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
        active_move_candidate_.reset();
        active_move_expected_episodes_ = 0;
        active_move_recorded_episodes_ = 0;
      }
      campaign_state_.store(CampaignState::IDLE_READY);
      trigger_deferred_dispatch();
      break;
    }
    case rclcpp_action::ResultCode::CANCELED: {
      RCLCPP_WARN(get_logger(), "⚠ [COLLECTION TASK] Canceled by Orchestrator or client request.");
      {
        std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
        active_move_candidate_.reset();
        active_move_expected_episodes_ = 0;
        active_move_recorded_episodes_ = 0;
      }
      campaign_state_.store(CampaignState::IDLE_READY);
      trigger_deferred_dispatch();
      break;
    }
    default:
      RCLCPP_WARN(get_logger(), "Unknown action result code: %d", static_cast<int>(result.code));
      campaign_state_.store(CampaignState::IDLE_READY);
      break;
  }
}

void EpisodeRecorderNode::trigger_deferred_dispatch()
{
  defer_dispatch_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5),
    [this]() {
      if (defer_dispatch_timer_) {
        defer_dispatch_timer_->cancel();
      }
      if (campaign_state_.load() == CampaignState::IDLE_READY && !has_active_goal()) {
        dispatch_next_task();
      }
    });
}

void EpisodeRecorderNode::advance_to_new_session()
{
  std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
  std::string new_fen = fen_synthesizer_->synthesize_fen(*quota_tracker_);

  quota_tracker_->advance_session();
  quota_tracker_->reset_session(new_fen);
  quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");

  active_move_candidate_.reset();
  active_move_expected_episodes_ = 0;
  active_move_recorded_episodes_ = 0;

  auto sess = quota_tracker_->get_session_state();
  RCLCPP_INFO(
    get_logger(),
    "\n================================================================================\n"
    "📢 [NEW COLLECTION SESSION] Session #%u Initialized (Target: 20 eps)\n"
    "Active FEN: %s\n"
    "Operator: Please arrange the physical chessboard pieces to match the FEN,\n"
    "or observe the 2D visualizer on /chess/board_2d/compressed!\n"
    "================================================================================",
    sess.session_id, new_fen.c_str());

  // Activate state barrier: Orchestrator SetFen sync & physical board alignment
  sync_active_fen_to_orchestrator(new_fen);
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

  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!writer_) {
    return;
  }

  writer_->write(std::const_pointer_cast<rclcpp::SerializedMessage>(msg),
                 topic_name, topic_type, this->now());
  message_counts_[topic_name]++;
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

  current_episode_index_ = scan_next_episode_index(base_output_dir_);
  current_episode_dir_ = make_episode_dir(current_episode_index_);

  try {
    std::filesystem::create_directories(base_output_dir_);
    if (std::filesystem::exists(current_episode_dir_)) {
      std::filesystem::remove_all(current_episode_dir_);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to prepare directories for '%s': %s",
                 current_episode_dir_.c_str(), e.what());
    return false;
  }

  std::lock_guard<std::mutex> lock(writer_mutex_);
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = current_episode_dir_.string();
  storage_opts.storage_id = storage_id_;
  storage_opts.max_cache_size = 50u * 1024u * 1024u;

#ifdef HAS_ROSBAG2_CUSTOM_DATA
  std::string task_str = task_;
  {
    std::lock_guard<std::mutex> lock_ins(instruction_mutex_);
    if (!current_instruction_.empty()) {
      task_str = current_instruction_;
    }
  }
  storage_opts.custom_data["experiment_name"] = experiment_name_;
  storage_opts.custom_data["episode_index"] = std::to_string(current_episode_index_);
  storage_opts.custom_data["task"] = task_str;
#endif

  try {
    writer_->open(storage_opts, {rmw_get_serialization_format(), rmw_get_serialization_format()});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to open MCAP writer: %s", e.what());
    writer_.reset();
    std::filesystem::remove_all(current_episode_dir_);
    return false;
  }

  for (const auto &[topic, type] : topic_type_map_) {
    rosbag2_storage::TopicMetadata meta;
    meta.name = topic;
    meta.type = type;
    meta.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(meta);
  }

  message_counts_.clear();
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

  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (writer_) {
      writer_.reset();
    }
  }

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
  bool ok = write_episode_info_yaml(
    current_episode_dir_, current_episode_index_, duration,
    episode_start_time_, episode_stop_time_);

  if (ok) {
    std::string task_saved = task_;
    {
      std::lock_guard<std::mutex> lock(instruction_mutex_);
      if (!current_instruction_.empty()) {
        task_saved = current_instruction_;
      }
    }

    if (enable_collection_planner_ && quota_tracker_) {
      std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
      quota_tracker_->record_episode(task_saved);
      auto sess = quota_tracker_->get_session_state();
      sess.episodes_in_session++;
      quota_tracker_->set_session_state(sess);
      quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");

      active_move_recorded_episodes_++;

      auto sum = quota_tracker_->get_summary();
      RCLCPP_INFO(
        get_logger(),
        "✔ [EPISODE SAVED] Episode %06u! Campaign: %u/%u (MOVE: %u, CLEAR: %u, PICK: %u, PLACE: %u). Session: %u/20. Move: %u/%u",
        current_episode_index_, sum.total_completed, sum.total_target,
        sum.completed_by_kind[domain::StepKind::MOVE],
        sum.completed_by_kind[domain::StepKind::CLEAR],
        sum.completed_by_kind[domain::StepKind::PICK],
        sum.completed_by_kind[domain::StepKind::PLACE],
        sess.episodes_in_session,
        active_move_recorded_episodes_, active_move_expected_episodes_);
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
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (writer_) {
      writer_.reset();
    }
  }

  if (std::filesystem::exists(current_episode_dir_)) {
    try {
      std::filesystem::remove_all(current_episode_dir_);
      RCLCPP_WARN(get_logger(), "🗑 [EPISODE RECORDER] Discarded & deleted Episode %06u.",
                  current_episode_index_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to delete discarded episode: %s", e.what());
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "🗑 [EPISODE RECORDER] Retaining active task instruction for retry: '%s'",
        current_instruction_.c_str());
    }
  }

  current_state_.store(EpisodeState::IDLE);
  return true;
}

uint32_t EpisodeRecorderNode::scan_next_episode_index(const std::filesystem::path &dir)
{
  uint32_t max_index = 0;
  bool found = false;

  if (!std::filesystem::exists(dir)) {
    return 0;
  }

  const std::regex pattern(R"(episode_(\d{6}))");
  for (const auto &entry : std::filesystem::directory_iterator(dir)) {
    if (!entry.is_directory()) continue;
    std::smatch match;
    std::string name = entry.path().filename().string();
    if (std::regex_match(name, match, pattern)) {
      uint32_t idx = static_cast<uint32_t>(std::stoul(match[1].str()));
      if (!found || idx >= max_index) {
        max_index = idx + 1;
        found = true;
      }
    }
  }
  return max_index;
}

std::filesystem::path EpisodeRecorderNode::make_episode_dir(uint32_t index)
{
  std::ostringstream ss;
  ss << "episode_" << std::setfill('0') << std::setw(6) << index;
  return base_output_dir_ / ss.str();
}

bool EpisodeRecorderNode::write_episode_info_yaml(
  const std::filesystem::path &episode_dir,
  uint32_t episode_index,
  double duration_sec,
  const rclcpp::Time &start_time,
  const rclcpp::Time &end_time)
{
  std::filesystem::path file_path = episode_dir / "episode_info.yaml";

  std::string task_str = task_;
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      task_str = current_instruction_;
    }
  }

  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "episode_id" << YAML::Value << episode_index;
    out << YAML::Key << "dataset_format" << YAML::Value << "lerobot_3.0";
    out << YAML::Key << "experiment_name" << YAML::Value << experiment_name_;
    out << YAML::Key << "task" << YAML::Value << task_str;
    out << YAML::Key << "teleop_mode" << YAML::Value << "gamepad_manual";
    out << YAML::Key << "success" << YAML::Value << true;
    out << YAML::Key << "duration_sec" << YAML::Value << duration_sec;
    out << YAML::Key << "timestamp_start" << YAML::Value << start_time.seconds();
    out << YAML::Key << "timestamp_end" << YAML::Value << end_time.seconds();

    out << YAML::Key << "message_counts" << YAML::Value << YAML::BeginMap;
    for (const auto &[topic, count] : message_counts_) {
      out << YAML::Key << topic << YAML::Value << count;
    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(file_path);
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(get_logger(), "📝 [METADATA] Generated episode_info.yaml (Task: '%s')", task_str.c_str());
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to emit YAML metadata: %s", e.what());
    return false;
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
  if (!enable_collection_planner_ || !quota_tracker_) {
    res->success = false;
    res->message = "Collection planner is disabled.";
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
  auto sum = quota_tracker_->get_summary();
  auto sess = quota_tracker_->get_session_state();

  std::ostringstream ss;
  ss << "Campaign: " << sum.total_completed << "/" << sum.total_target
     << " (MOVE: " << sum.completed_by_kind[domain::StepKind::MOVE]
     << ", CLEAR: " << sum.completed_by_kind[domain::StepKind::CLEAR]
     << " [L:" << sum.clear_left_completed << "/R:" << sum.clear_right_completed << "]"
     << ", PICK: " << sum.completed_by_kind[domain::StepKind::PICK]
     << ", PLACE: " << sum.completed_by_kind[domain::StepKind::PLACE] << ")"
     << " | Session #" << sess.session_id << ": " << sess.episodes_in_session << "/"
     << campaign_config_.episodes_per_session << " eps.";

  res->success = true;
  res->message = ss.str();
}

void EpisodeRecorderNode::init_diagnostics()
{
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID("chess_episode_recorder");
  diag_updater_->add(
    "Episode Recorder Status", this, &EpisodeRecorderNode::produce_recorder_diagnostics);
  diag_updater_->add(
    "Collection Campaign Status", this, &EpisodeRecorderNode::produce_campaign_diagnostics);

  RCLCPP_INFO(get_logger(), "Diagnostic updater initialized (Hardware ID: chess_episode_recorder)");
}

RecorderDiagnosticsSnapshot EpisodeRecorderNode::capture_diagnostics_snapshot() const
{
  RecorderDiagnosticsSnapshot snap;
  snap.episode_state = current_state_.load(std::memory_order_relaxed);
  snap.campaign_state = campaign_state_.load(std::memory_order_relaxed);
  snap.enable_collection_planner = enable_collection_planner_;
  snap.is_action_server_connected = is_action_server_connected();
  snap.has_active_goal = has_active_goal();
  {
    std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
    if (active_move_candidate_.has_value()) {
      snap.active_goal_uci = active_move_candidate_->uci;
    }
  }
  snap.current_episode_index = current_episode_index_;
  snap.base_output_dir = base_output_dir_.string();

  // Active instruction
  {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(instruction_mutex_));
    snap.active_instruction = current_instruction_;
  }

  // Writer message counts
  {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(writer_mutex_));
    uint64_t total = 0;
    for (const auto &[topic, cnt] : message_counts_) {
      total += cnt;
    }
    snap.total_recorded_messages = total;
  }

  // Disk space query (safe, non-throwing)
  std::error_code ec;
  const auto space_info = std::filesystem::space(base_output_dir_, ec);
  if (!ec) {
    snap.available_disk_space_bytes = space_info.available;
    snap.disk_query_success = true;
  } else {
    snap.available_disk_space_bytes = 0;
    snap.disk_query_success = false;
  }

  // Campaign state & quotas
  if (enable_collection_planner_ && quota_tracker_) {
    std::lock_guard<std::recursive_mutex> lock(domain_mutex_);
    auto sum = quota_tracker_->get_summary();
    auto sess = quota_tracker_->get_session_state();

    snap.total_completed_episodes = sum.total_completed;
    snap.target_total_episodes = sum.total_target;
    snap.session_id = sess.session_id;
    snap.episodes_in_session = sess.episodes_in_session;
    snap.session_target_episodes = 20;

    snap.completed_move = sum.completed_by_kind[domain::StepKind::MOVE];
    snap.target_move = sum.target_by_kind[domain::StepKind::MOVE];

    snap.completed_clear = sum.completed_by_kind[domain::StepKind::CLEAR];
    snap.target_clear = sum.target_by_kind[domain::StepKind::CLEAR];
    snap.clear_left_bin = sum.clear_left_completed;
    snap.clear_right_bin = sum.clear_right_completed;

    snap.completed_pick = sum.completed_by_kind[domain::StepKind::PICK];
    snap.target_pick = sum.target_by_kind[domain::StepKind::PICK];

    snap.completed_place = sum.completed_by_kind[domain::StepKind::PLACE];
    snap.target_place = sum.target_by_kind[domain::StepKind::PLACE];
  }

  return snap;
}

void EpisodeRecorderNode::produce_recorder_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  const auto snap = capture_diagnostics_snapshot();
  RecorderDiagnostics::evaluate_recorder_status(snap, stat);
}

void EpisodeRecorderNode::produce_campaign_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  const auto snap = capture_diagnostics_snapshot();
  RecorderDiagnostics::evaluate_campaign_status(snap, stat);
}

} // namespace chess_episode_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(chess_episode_recorder::EpisodeRecorderNode)

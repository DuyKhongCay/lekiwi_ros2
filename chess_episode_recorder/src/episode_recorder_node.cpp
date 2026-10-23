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
  details.piece_color = (board.sideToMove() == chess::Color::WHITE) ? "w" : "b";

  auto moving = cand.moving_piece(board);
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
  this->declare_parameter<std::string>("collection_config_path", "");
  this->declare_parameter<std::string>("game_status_topic", "/chess/game_status");
  this->declare_parameter<std::string>("import_fen_service", "/orchestrator/import_fen");
  this->declare_parameter<std::string>("export_fen_service", "/orchestrator/export_fen");
  this->declare_parameter<std::string>(
    "check_feasibility_service", "/workspace/check_move_feasibility");
  this->declare_parameter<std::string>(
    "execute_collection_task_service", "/orchestrator/execute_collection_task");

  root_dir_ = this->get_parameter("root_dir").as_string();
  experiment_name_ = this->get_parameter("experiment_name").as_string();
  task_ = this->get_parameter("task").as_string();
  storage_id_ = this->get_parameter("storage_id").as_string();
  feedback_topic_ = this->get_parameter("feedback_topic").as_string();
  requested_topics_ = this->get_parameter("topics").as_string_array();

  enable_collection_planner_ = this->get_parameter("enable_collection_planner").as_bool();
  collection_config_path_ = this->get_parameter("collection_config_path").as_string();
  game_status_topic_ = this->get_parameter("game_status_topic").as_string();
  import_fen_srv_name_ = this->get_parameter("import_fen_service").as_string();
  export_fen_srv_name_ = this->get_parameter("export_fen_service").as_string();
  check_feasibility_srv_name_ = this->get_parameter("check_feasibility_service").as_string();
  execute_collection_task_srv_name_ = this->get_parameter("execute_collection_task_service").as_string();

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

  quota_tracker_ = std::make_unique<domain::QuotaTracker>();
  if (!collection_config_path_.empty()) {
    quota_tracker_->load_config_file(collection_config_path_);
  }

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
  if (quota_tracker_->load_session_state(session_file)) {
    auto sess = quota_tracker_->get_session_state();
    RCLCPP_INFO(
      get_logger(),
      "💾 [CAMPAIGN CONDUCTOR] Resumed active session #%u (FEN: %s, %u/20 eps)",
      sess.session_id, sess.active_fen.c_str(), sess.episodes_in_session);
  }

  // Create ROS 2 service clients
  import_fen_client_ = this->create_client<lekiwi_interfaces::srv::SetFen>(
    import_fen_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);
  export_fen_client_ = this->create_client<std_srvs::srv::Trigger>(
    export_fen_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);
  check_feasibility_client_ = this->create_client<lekiwi_interfaces::srv::CheckMoveFeasibility>(
    check_feasibility_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);
  execute_collection_task_client_ = this->create_client<lekiwi_interfaces::srv::ExecuteCollectionTask>(
    execute_collection_task_srv_name_, rclcpp::ServicesQoS(), cb_group_clients_);

  // Subscribe to board game status with latched Reliable QoS (TransientLocal, Depth 1)
  rclcpp::QoS game_status_qos(1);
  game_status_qos.reliable();
  game_status_qos.transient_local();
  game_status_sub_ = this->create_subscription<lekiwi_interfaces::msg::ChessGameStatus>(
    game_status_topic_,
    game_status_qos,
    std::bind(&EpisodeRecorderNode::on_game_status_received, this, std::placeholders::_1));

  campaign_state_.store(CampaignState::IDLE_AWAITING_BOARD);
}

void EpisodeRecorderNode::on_game_status_received(
  const lekiwi_interfaces::msg::ChessGameStatus::ConstSharedPtr msg)
{
  if (!enable_collection_planner_ || !msg) {
    return;
  }

  if (quota_tracker_->is_campaign_complete()) {
    if (campaign_state_.load() != CampaignState::CAMPAIGN_FINISHED) {
      campaign_state_.store(CampaignState::CAMPAIGN_FINISHED);
      RCLCPP_INFO(get_logger(), "🎉 [CAMPAIGN COMPLETE] All 600 target episodes achieved successfully!");
    }
    return;
  }

  last_seen_fen_ = msg->full_fen;
  board_is_stable_ = msg->is_board_stable;

  // Trigger automated move selection or session synthesis only when board vision is fully settled
  if (msg->is_board_stable &&
      msg->game_phase == lekiwi_interfaces::msg::ChessGameStatus::PHASE_WAITING_PLAYER &&
      campaign_state_.load() == CampaignState::IDLE_AWAITING_BOARD)
  {
    if (!is_planning_in_progress_.exchange(true)) {
      std::thread([this, fen = msg->full_fen]() {
        trigger_campaign_step_async(fen);
        is_planning_in_progress_.store(false);
      }).detach();
    }
  }
}

bool EpisodeRecorderNode::query_move_feasibility_sync(
  const domain::MoveCandidate &cand,
  const chess::Board &board,
  uint8_t &out_plan_type)
{
  if (!check_feasibility_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Feasibility service '%s' is not ready yet.", check_feasibility_srv_name_.c_str());
    return false;
  }

  auto req = std::make_shared<lekiwi_interfaces::srv::CheckMoveFeasibility::Request>();
  req->move = to_ros_move_details(cand, board);

  auto future = check_feasibility_client_->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready) {
    return false;
  }

  auto resp = future.get();
  out_plan_type = resp->plan_type;
  return resp->feasible;
}

void EpisodeRecorderNode::trigger_campaign_step_async(const std::string &fen)
{
  std::lock_guard<std::mutex> lock(domain_mutex_);
  auto sess = quota_tracker_->get_session_state();

  // If no active session or 20 episodes completed, initialize new session FEN
  if (!sess.is_active || sess.episodes_in_session >= 20) {
    advance_to_new_session();
    return;
  }

  chess::Board board;
  if (!board.setFen(fen)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse board FEN: '%s'", fen.c_str());
    return;
  }

  campaign_state_.store(CampaignState::PLANNING_OR_SYNTHESIS);

  domain::MoveCandidateSelector::FeasibilityChecker checker =
    [this, &board](const domain::MoveCandidate &cand) -> std::pair<bool, uint8_t> {
      uint8_t plan_type = 0;
      bool feasible = query_move_feasibility_sync(cand, board, plan_type);
      return {feasible, plan_type};
    };

  auto best_move = move_selector_->select_best_move(board, *quota_tracker_, checker);
  if (best_move.has_value()) {
    RCLCPP_INFO(
      get_logger(),
      "🎯 [CANDIDATE SELECTED] Move: %s (Score: %.1f, Feasible: %s, Plan: %u, Bypass: %zu)",
      best_move->uci.c_str(), best_move->score, best_move->is_feasible ? "YES" : "NO",
      best_move->plan_type, best_move->bypass_steps.size());

    dispatch_task_to_orchestrator(*best_move, board);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Board exhausted: No contributing feasible moves found on active turn (%s). Advancing session...",
      (board.sideToMove() == chess::Color::WHITE ? "White" : "Black"));
    advance_to_new_session();
  }
}

void EpisodeRecorderNode::advance_to_new_session()
{
  std::string new_fen = fen_synthesizer_->synthesize_fen(*quota_tracker_);

  if (import_fen_client_->wait_for_service(std::chrono::seconds(5))) {
    auto req = std::make_shared<lekiwi_interfaces::srv::SetFen::Request>();
    req->fen = new_fen;
    req->robot_color = ""; // Deduce color automatically
    auto future = import_fen_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      auto resp = future.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "✔ [IMPORT FEN] Successfully synchronized FEN with Orchestrator");
      } else {
        RCLCPP_WARN(get_logger(), "⚠ [IMPORT FEN] Orchestrator SetFen returned false: %s", resp->message.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "⚠ [IMPORT FEN] Timed out waiting for SetFen service response");
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "⚠ [IMPORT FEN] SetFen service '%s' not available within 5s",
      import_fen_srv_name_.c_str());
  }

  quota_tracker_->advance_session();
  quota_tracker_->reset_session(new_fen);
  quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");

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

  campaign_state_.store(CampaignState::IDLE_AWAITING_BOARD);
}

void EpisodeRecorderNode::dispatch_task_to_orchestrator(
  const domain::MoveCandidate &cand,
  const chess::Board &board)
{
  if (!execute_collection_task_client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Execute collection task service '%s' is not available!",
                 execute_collection_task_srv_name_.c_str());
    campaign_state_.store(CampaignState::IDLE_AWAITING_BOARD);
    return;
  }

  auto req = std::make_shared<lekiwi_interfaces::srv::ExecuteCollectionTask::Request>();
  req->move = to_ros_move_details(cand, board);
  req->force_pick_place = cand.force_pick_place;
  req->clear_bin_side = cand.clear_bin_side;
  req->bypass_steps = cand.bypass_steps;

  // Record candidate move into session state so it is never re-selected in this session
  if (quota_tracker_) {
    quota_tracker_->record_executed_move(cand.uci);
    quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");
  }

  campaign_state_.store(CampaignState::DISPATCHED_TO_ORCH);

  auto future = execute_collection_task_client_->async_send_request(
    req,
    [this](rclcpp::Client<lekiwi_interfaces::srv::ExecuteCollectionTask>::SharedFuture fut) {
      auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "🚀 [DISPATCH ACCEPTED] Orchestrator started collection move pipeline.");
      } else {
        RCLCPP_ERROR(get_logger(), "❌ [DISPATCH REJECTED] Orchestrator rejected task: %s", resp->message.c_str());
        campaign_state_.store(CampaignState::IDLE_AWAITING_BOARD);
      }
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
      std::lock_guard<std::mutex> lock(domain_mutex_);
      quota_tracker_->record_episode(task_saved);
      auto sess = quota_tracker_->get_session_state();
      sess.episodes_in_session++;
      quota_tracker_->set_session_state(sess);
      quota_tracker_->save_session_state(base_output_dir_ / "session_state.yaml");

      auto sum = quota_tracker_->get_summary();
      RCLCPP_INFO(
        get_logger(),
        "✔ [EPISODE SAVED] Episode %06u! Campaign: %u/%u (MOVE: %u, CLEAR: %u, PICK: %u, PLACE: %u). Session: %u/20",
        current_episode_index_, sum.total_completed, sum.total_target,
        sum.completed_by_kind[domain::StepKind::MOVE],
        sum.completed_by_kind[domain::StepKind::CLEAR],
        sum.completed_by_kind[domain::StepKind::PICK],
        sum.completed_by_kind[domain::StepKind::PLACE],
        sess.episodes_in_session);

      if (sess.episodes_in_session >= 20) {
        advance_to_new_session();
      } else {
        campaign_state_.store(CampaignState::IDLE_AWAITING_BOARD);
      }
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
  if (enable_collection_planner_) {
    campaign_state_.store(CampaignState::IDLE_AWAITING_BOARD);
  }
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

  std::lock_guard<std::mutex> lock(domain_mutex_);
  auto sum = quota_tracker_->get_summary();
  auto sess = quota_tracker_->get_session_state();

  std::ostringstream ss;
  ss << "Campaign: " << sum.total_completed << "/" << sum.total_target
     << " (MOVE: " << sum.completed_by_kind[domain::StepKind::MOVE]
     << ", CLEAR: " << sum.completed_by_kind[domain::StepKind::CLEAR]
     << " [L:" << sum.clear_left_completed << "/R:" << sum.clear_right_completed << "]"
     << ", PICK: " << sum.completed_by_kind[domain::StepKind::PICK]
     << ", PLACE: " << sum.completed_by_kind[domain::StepKind::PLACE] << ")"
     << " | Session #" << sess.session_id << ": " << sess.episodes_in_session << "/20 eps.";

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
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(domain_mutex_));
    snap.board_is_stable = board_is_stable_;
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

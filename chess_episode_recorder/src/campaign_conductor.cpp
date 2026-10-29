#include "chess_episode_recorder/campaign_conductor.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace chess_episode_recorder
{

void RecorderDiagnostics::evaluate_recorder_status(
  const RecorderDiagnosticsSnapshot &snapshot,
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Episode State", std::string(storage::to_string(snapshot.episode_state)));
  stat.addf("Current Episode Index", "%06u", snapshot.current_episode_index);
  stat.add("Base Output Dir", snapshot.base_output_dir);
  stat.addf(
    "Total Recorded Msgs", "%llu",
    static_cast<unsigned long long>(snapshot.total_recorded_messages));
  stat.add(
    "Active Instruction",
    snapshot.active_instruction.empty() ? "(none)" : snapshot.active_instruction);

  if (!snapshot.disk_query_success) {
    stat.add("Disk Free Space", "Query Failed");
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Failed to access or query storage directory disk space");
    return;
  }

  const double free_gb = static_cast<double>(snapshot.available_disk_space_bytes) /
                         (1024.0 * 1024.0 * 1024.0);
  stat.addf("Disk Free Space (GB)", "%.2f", free_gb);

  if (snapshot.available_disk_space_bytes < ERROR_DISK_SPACE_BYTES) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Critical disk space: storage directory nearly full (< 200 MB)");
  } else if (snapshot.available_disk_space_bytes < WARN_DISK_SPACE_BYTES) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Low disk space: available storage below 1 GB");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Episode recorder nominal");
  }
}

void RecorderDiagnostics::evaluate_campaign_status(
  const RecorderDiagnosticsSnapshot &snapshot,
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!snapshot.enable_collection_planner) {
    stat.add("Campaign Mode", "MANUAL_TELEOP");
    stat.add("Campaign State", "DISABLED");
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Campaign planner disabled (Manual Teleop Mode)");
    return;
  }

  stat.add("Campaign Mode", "ENABLED");
  stat.add("Campaign State", std::string(to_string(snapshot.campaign_state)));
  stat.add("Action Server Connected", snapshot.is_action_server_connected ? "true" : "false");
  stat.add("Active Goal Dispatched", snapshot.has_active_goal ? "true" : "false");
  if (snapshot.has_active_goal && !snapshot.active_goal_uci.empty()) {
    stat.add("Active Goal UCI", snapshot.active_goal_uci);
  }

  const double progress_pct = (snapshot.target_total_episodes > 0)
    ? (100.0 * static_cast<double>(snapshot.total_completed_episodes) /
       static_cast<double>(snapshot.target_total_episodes))
    : 0.0;

  stat.addf(
    "Total Progress", "%u / %u (%.1f%%)",
    snapshot.total_completed_episodes, snapshot.target_total_episodes, progress_pct);

  stat.addf(
    "Session Progress", "Session %u: %u / %u",
    snapshot.session_id, snapshot.episodes_in_session, snapshot.session_target_episodes);

  stat.addf("MOVE Quotas", "%u / %u", snapshot.completed_move, snapshot.target_move);
  stat.addf(
    "CLEAR Quotas", "%u / %u (L: %u, R: %u)",
    snapshot.completed_clear, snapshot.target_clear,
    snapshot.clear_left_bin, snapshot.clear_right_bin);
  stat.addf("PICK Quotas", "%u / %u", snapshot.completed_pick, snapshot.target_pick);
  stat.addf("PLACE Quotas", "%u / %u", snapshot.completed_place, snapshot.target_place);

  if (snapshot.campaign_state == CampaignState::CAMPAIGN_FINISHED) {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Campaign complete: target %u episodes reached",
      snapshot.target_total_episodes);
  } else if (!snapshot.is_action_server_connected) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Waiting for Orchestrator ExecuteCollectionTask action server");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Collection campaign nominal");
  }
}

// ----------------------------------------------------------------------------
// CampaignConductor Implementation
// ----------------------------------------------------------------------------

CampaignConductor::CampaignConductor(
  domain::CampaignConfig config,
  std::filesystem::path storage_dir,
  std::string experiment_name,
  bool auto_advance_session)
: config_(config),
  storage_dir_(std::move(storage_dir)),
  experiment_name_(std::move(experiment_name)),
  auto_advance_session_(auto_advance_session),
  quota_tracker_(config_)
{
}

uint32_t CampaignConductor::initialize_from_storage(bool quarantine_corrupted)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  const auto report = storage::DatasetDirectoryScanner::scan_and_recover(
    storage_dir_, quarantine_corrupted);

  // Restore instructions to quota tracker
  for (const auto &task : report.recovered_instructions) {
    quota_tracker_.record_episode(task);
  }

  // Load session state if available
  const auto session_yaml = storage_dir_ / "session_state.yaml";
  const auto loaded_state = storage::SessionStateYamlRepository::load(session_yaml);

  if (loaded_state && loaded_state->is_active) {
    quota_tracker_.restore_session_state(*loaded_state);
    target_session_fen_ = loaded_state->active_fen;
  } else {
    // If no active session, advance to initial session
    const std::string new_fen = fen_synthesizer_.synthesize_fen(quota_tracker_);
    quota_tracker_.reset_session(new_fen);
    target_session_fen_ = new_fen;
    storage::SessionStateYamlRepository::save(session_yaml, quota_tracker_.get_session_state());
  }

  if (quota_tracker_.is_campaign_complete()) {
    campaign_state_ = CampaignState::CAMPAIGN_FINISHED;
  } else {
    campaign_state_ = CampaignState::IDLE_READY;
  }

  return report.recovered_episodes;
}

bool CampaignConductor::save_session_state()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return storage::SessionStateYamlRepository::save(
    storage_dir_ / "session_state.yaml", quota_tracker_.get_session_state());
}

void CampaignConductor::configure(const domain::CampaignConfig &config)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  config_ = config;
  quota_tracker_.configure(config);
}

bool CampaignConductor::advance_to_new_session()
{
  std::string new_fen = fen_synthesizer_.synthesize_fen(quota_tracker_);
  quota_tracker_.advance_session();
  quota_tracker_.reset_session(new_fen);
  storage::SessionStateYamlRepository::save(
    storage_dir_ / "session_state.yaml", quota_tracker_.get_session_state());

  active_goal_.reset();
  active_move_expected_episodes_ = 0;
  active_move_recorded_episodes_ = 0;

  target_session_fen_ = new_fen;
  board_synced_ = false;
  fen_sync_acknowledged_ = false;
  campaign_state_ = CampaignState::AWAITING_PHYSICAL_SYNC;
  return true;
}

CampaignState CampaignConductor::get_campaign_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return campaign_state_;
}

domain::QuotaSummary CampaignConductor::get_quota_summary() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return quota_tracker_.get_summary();
}

domain::SessionState CampaignConductor::get_session_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return quota_tracker_.get_session_state();
}

std::optional<DispatchedGoal> CampaignConductor::get_active_goal() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_goal_;
}

bool CampaignConductor::has_active_goal() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_goal_.has_value();
}

bool CampaignConductor::is_campaign_complete() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return quota_tracker_.is_campaign_complete();
}

uint32_t CampaignConductor::get_active_move_expected_episodes() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_move_expected_episodes_;
}

uint32_t CampaignConductor::get_active_move_recorded_episodes() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_move_recorded_episodes_;
}

void CampaignConductor::set_board_synced(bool synced)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  board_synced_ = synced;
  if (board_synced_ && fen_sync_acknowledged_ && campaign_state_ == CampaignState::AWAITING_PHYSICAL_SYNC) {
    campaign_state_ = CampaignState::IDLE_READY;
  }
}

void CampaignConductor::set_fen_sync_acknowledged(bool ack)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  fen_sync_acknowledged_ = ack;
  if (board_synced_ && fen_sync_acknowledged_ && campaign_state_ == CampaignState::AWAITING_PHYSICAL_SYNC) {
    campaign_state_ = CampaignState::IDLE_READY;
  }
}

bool CampaignConductor::is_board_synced() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return board_synced_;
}

bool CampaignConductor::is_fen_sync_acknowledged() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return fen_sync_acknowledged_;
}

std::string CampaignConductor::get_target_session_fen() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return target_session_fen_;
}

void CampaignConductor::sync_active_fen(const std::string &fen)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_session_fen_ = fen;
  board_synced_ = false;
  fen_sync_acknowledged_ = false;
  campaign_state_ = CampaignState::AWAITING_PHYSICAL_SYNC;
}

std::optional<PlanningSnapshot> CampaignConductor::prepare_planning_snapshot()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (quota_tracker_.is_campaign_complete()) {
    campaign_state_ = CampaignState::CAMPAIGN_FINISHED;
    return std::nullopt;
  }

  if (active_goal_.has_value()) {
    return std::nullopt;
  }

  const auto session = quota_tracker_.get_session_state();
  if (!session.is_active || quota_tracker_.is_session_budget_exhausted()) {
    if (auto_advance_session_) {
      advance_to_new_session();
    }
    return std::nullopt;
  }

  if (!board_synced_ || !fen_sync_acknowledged_) {
    campaign_state_ = CampaignState::AWAITING_PHYSICAL_SYNC;
    return std::nullopt;
  }

  if (campaign_state_ != CampaignState::IDLE_READY) {
    return std::nullopt;
  }

  const auto board = quota_tracker_.get_active_board();
  chess::Movelist legal_moves;
  chess::movegen::legalmoves(legal_moves, board);
  if (legal_moves.empty()) {
    // Checkmate or stalemate: advance session
    if (auto_advance_session_) {
      advance_to_new_session();
    }
    return std::nullopt;
  }

  campaign_state_ = CampaignState::PLANNING_OR_SYNTHESIS;

  PlanningSnapshot snap;
  snap.session_state = session;
  snap.current_fen = board.getFen();
  snap.current_board = board;
  snap.tracker = quota_tracker_;
  return snap;
}

void CampaignConductor::abort_planning()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (campaign_state_ == CampaignState::PLANNING_OR_SYNTHESIS) {
    campaign_state_ = CampaignState::IDLE_READY;
  }
}

std::optional<DispatchedGoal> CampaignConductor::commit_selected_candidate(
  const domain::MoveCandidate &cand)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (campaign_state_ != CampaignState::PLANNING_OR_SYNTHESIS &&
      campaign_state_ != CampaignState::IDLE_READY) {
    return std::nullopt;
  }

  DispatchedGoal goal;
  goal.candidate = cand;
  goal.expected_target_fen = cand.calculate_target_fen(quota_tracker_.get_active_board());
  goal.expected_episodes = cand.expected_episode_count(quota_tracker_.get_active_board());

  active_goal_ = goal;
  active_move_expected_episodes_ = goal.expected_episodes;
  active_move_recorded_episodes_ = 0;
  campaign_state_ = CampaignState::DISPATCHED_TO_ORCH;

  return goal;
}

std::optional<DispatchedGoal> CampaignConductor::plan_and_commit_best_candidate(
  domain::MoveCandidateSelector::FeasibilityChecker checker)
{
  auto snapshot = prepare_planning_snapshot();
  if (!snapshot) {
    return std::nullopt;
  }

  // Pure evaluation outside the state mutex lock!
  auto best = move_selector_.select_best_move(
    snapshot->current_board,
    snapshot->tracker,
    checker);

  if (!best) {
    abort_planning();
    return std::nullopt;
  }

  return commit_selected_candidate(*best);
}

void CampaignConductor::on_goal_rejected()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_goal_.reset();
  active_move_expected_episodes_ = 0;
  active_move_recorded_episodes_ = 0;
  campaign_state_ = CampaignState::IDLE_READY;
}

void CampaignConductor::on_goal_aborted_or_canceled()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_goal_.reset();
  active_move_expected_episodes_ = 0;
  active_move_recorded_episodes_ = 0;
  campaign_state_ = CampaignState::IDLE_READY;
}

domain::BoardTransitionOutcome CampaignConductor::on_collection_task_finished(
  bool action_success, const std::string &actual_fen, bool /*teleop_intervened*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!action_success) {
    active_goal_.reset();
    active_move_expected_episodes_ = 0;
    active_move_recorded_episodes_ = 0;
    campaign_state_ = CampaignState::IDLE_READY;
    return domain::BoardTransitionOutcome::INVALID_ACTUAL_FEN;
  }

  const std::string expected_uci = active_goal_ ? active_goal_->candidate.uci : "";
  const auto outcome = quota_tracker_.verify_and_commit_move(expected_uci, actual_fen);

  if (outcome == domain::BoardTransitionOutcome::DIVERGED_RESYNCED) {
    // Operator or teleop intervened to change state: FEN needs resync ack
    target_session_fen_ = actual_fen;
    fen_sync_acknowledged_ = false;
  }

  storage::SessionStateYamlRepository::save(
    storage_dir_ / "session_state.yaml", quota_tracker_.get_session_state());

  active_goal_.reset();
  active_move_expected_episodes_ = 0;
  active_move_recorded_episodes_ = 0;

  if (quota_tracker_.is_campaign_complete()) {
    campaign_state_ = CampaignState::CAMPAIGN_FINISHED;
  } else if (quota_tracker_.is_session_budget_exhausted()) {
    if (auto_advance_session_) {
      advance_to_new_session();
    } else {
      campaign_state_ = CampaignState::IDLE_READY;
    }
  } else {
    campaign_state_ = CampaignState::IDLE_READY;
  }

  return outcome;
}

void CampaignConductor::on_episode_recorded(const std::string &instruction)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  quota_tracker_.record_episode(instruction);
  active_move_recorded_episodes_++;

  auto sess = quota_tracker_.get_session_state();
  sess.episodes_in_session++;
  quota_tracker_.set_session_state(sess);
  storage::SessionStateYamlRepository::save(storage_dir_ / "session_state.yaml", sess);
}

RecorderDiagnosticsSnapshot CampaignConductor::capture_diagnostics_snapshot(
  const storage::DiskSpaceInfo &disk,
  EpisodeState ep_state,
  uint32_t current_ep_idx,
  uint64_t total_msgs,
  const std::string &active_instruction,
  bool is_action_server_connected) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RecorderDiagnosticsSnapshot snap;
  snap.episode_state = ep_state;
  snap.campaign_state = campaign_state_;
  snap.enable_collection_planner = (campaign_state_ != CampaignState::DISABLED);
  snap.is_action_server_connected = is_action_server_connected;
  snap.has_active_goal = active_goal_.has_value();
  if (active_goal_) {
    snap.active_goal_uci = active_goal_->candidate.uci;
  }
  snap.current_episode_index = current_ep_idx;
  snap.base_output_dir = storage_dir_.string();
  snap.active_instruction = active_instruction;
  snap.total_recorded_messages = total_msgs;
  snap.available_disk_space_bytes = disk.available_bytes;
  snap.disk_query_success = disk.success;

  const auto summary = quota_tracker_.get_summary();
  const auto session = quota_tracker_.get_session_state();

  snap.total_completed_episodes = summary.total_completed;
  snap.target_total_episodes = config_.total_episodes;
  snap.session_id = session.session_id;
  snap.episodes_in_session = session.episodes_in_session;
  snap.session_target_episodes = config_.episodes_per_session;

  snap.completed_move = summary.completed_by_kind.at(domain::StepKind::MOVE);
  snap.target_move = config_.move_total;
  snap.completed_clear = summary.completed_by_kind.at(domain::StepKind::CLEAR);
  snap.target_clear = config_.clear_total;
  snap.clear_left_bin = summary.clear_left_completed;
  snap.clear_right_bin = summary.clear_right_completed;
  snap.completed_pick = summary.completed_by_kind.at(domain::StepKind::PICK);
  snap.target_pick = config_.pick_total;
  snap.completed_place = summary.completed_by_kind.at(domain::StepKind::PLACE);
  snap.target_place = config_.place_total;

  return snap;
}

}  // namespace chess_episode_recorder


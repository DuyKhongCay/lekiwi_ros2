#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "chess_episode_recorder/campaign_quota_domain.hpp"
#include "chess_episode_recorder/move_planning_domain.hpp"
#include "chess_episode_recorder/dataset_storage.hpp"

namespace chess_episode_recorder
{

// Forward declare using the storage definitions
using storage::EpisodeState;

/**
 * @brief Finite State Machine states for the SmolVLA automated collection campaign conductor.
 */
enum class CampaignState : uint8_t
{
  DISABLED = 0,               ///< Collection planner disabled (pure manual teleop mode).
  IDLE_READY = 1,             ///< Campaign ready for next move dispatch or execution.
  PLANNING_OR_SYNTHESIS = 2,  ///< Actively evaluating moves or synthesizing new FEN.
  AWAITING_PHYSICAL_SYNC = 3, ///< Barrier: Waiting for Orchestrator FEN sync & physical board alignment.
  DISPATCHED_TO_ORCH = 4,     ///< Collection task sent to Orchestrator, waiting execution.
  CAMPAIGN_FINISHED = 5       ///< All 600 episodes target reached.
};

[[nodiscard]] constexpr std::string_view to_string(CampaignState state) noexcept
{
  switch (state) {
    case CampaignState::DISABLED:
      return "DISABLED";
    case CampaignState::IDLE_READY:
      return "IDLE_READY";
    case CampaignState::PLANNING_OR_SYNTHESIS:
      return "PLANNING_OR_SYNTHESIS";
    case CampaignState::AWAITING_PHYSICAL_SYNC:
      return "AWAITING_PHYSICAL_SYNC";
    case CampaignState::DISPATCHED_TO_ORCH:
      return "DISPATCHED_TO_ORCH";
    case CampaignState::CAMPAIGN_FINISHED:
      return "CAMPAIGN_FINISHED";
  }
  return "UNKNOWN";
}

/**
 * @brief Thread-safe immutable snapshot of episode recorder and campaign metrics.
 */
struct RecorderDiagnosticsSnapshot
{
  EpisodeState episode_state{EpisodeState::IDLE};
  CampaignState campaign_state{CampaignState::DISABLED};
  bool enable_collection_planner{false};
  bool is_action_server_connected{false};
  bool has_active_goal{false};
  std::string active_goal_uci{""};
  uint32_t current_episode_index{0};
  std::string base_output_dir{""};
  std::string active_instruction{""};
  uint64_t total_recorded_messages{0};
  uint64_t available_disk_space_bytes{0};
  bool disk_query_success{true};

  // Campaign specific metrics
  uint32_t total_completed_episodes{0};
  uint32_t target_total_episodes{600};
  uint32_t session_id{0};
  uint32_t episodes_in_session{0};
  uint32_t session_target_episodes{20};
  uint32_t completed_move{0};
  uint32_t target_move{240};
  uint32_t completed_clear{0};
  uint32_t target_clear{120};
  uint32_t clear_left_bin{0};
  uint32_t clear_right_bin{0};
  uint32_t completed_pick{0};
  uint32_t target_pick{120};
  uint32_t completed_place{0};
  uint32_t target_place{120};
};

/**
 * @brief Pure evaluator for episode recorder and collection campaign health.
 */
class RecorderDiagnostics
{
public:
  static constexpr uint64_t WARN_DISK_SPACE_BYTES{1024ULL * 1024ULL * 1024ULL};  ///< 1 GB warning threshold
  static constexpr uint64_t ERROR_DISK_SPACE_BYTES{200ULL * 1024ULL * 1024ULL};  ///< 200 MB critical threshold

  static void evaluate_recorder_status(
    const RecorderDiagnosticsSnapshot &snapshot,
    diagnostic_updater::DiagnosticStatusWrapper &stat);

  static void evaluate_campaign_status(
    const RecorderDiagnosticsSnapshot &snapshot,
    diagnostic_updater::DiagnosticStatusWrapper &stat);
};

/**
 * @brief Dispatched goal specification containing move and expected episode metrics.
 */
struct DispatchedGoal
{
  domain::MoveCandidate candidate;
  std::string expected_target_fen;
  uint32_t expected_episodes{1};
};

/**
 * @brief Snapshot of active domain board and candidate moves for non-blocking feasibility checks.
 */
struct PlanningSnapshot
{
  domain::SessionState session_state;
  std::string current_fen;
  chess::Board current_board;
  domain::QuotaTracker tracker;
};

/**
 * @brief Conductor coordinating campaign progression, FSM transitions, and state synchronization.
 */
class CampaignConductor
{
public:
  explicit CampaignConductor(
    domain::CampaignConfig config,
    std::filesystem::path storage_dir,
    std::string experiment_name,
    bool auto_advance_session = true);

  ~CampaignConductor() = default;

  // Non-copyable, movable
  CampaignConductor(const CampaignConductor &) = delete;
  CampaignConductor & operator=(const CampaignConductor &) = delete;
  CampaignConductor(CampaignConductor &&) noexcept = default;
  CampaignConductor & operator=(CampaignConductor &&) noexcept = default;

  // Lifecycle & Storage Recovery
  uint32_t initialize_from_storage(bool quarantine_corrupted = false);
  bool save_session_state();
  void configure(const domain::CampaignConfig &config);
  bool advance_to_new_session();

  // State inspection
  [[nodiscard]] CampaignState get_campaign_state() const;
  [[nodiscard]] domain::QuotaSummary get_quota_summary() const;
  [[nodiscard]] domain::SessionState get_session_state() const;
  [[nodiscard]] std::optional<DispatchedGoal> get_active_goal() const;
  [[nodiscard]] bool has_active_goal() const;
  [[nodiscard]] bool is_campaign_complete() const;
  [[nodiscard]] uint32_t get_active_move_expected_episodes() const;
  [[nodiscard]] uint32_t get_active_move_recorded_episodes() const;

  // Sync barrier
  void set_board_synced(bool synced);
  void set_fen_sync_acknowledged(bool ack);
  void sync_active_fen(const std::string &fen);
  [[nodiscard]] bool is_board_synced() const;
  [[nodiscard]] bool is_fen_sync_acknowledged() const;
  [[nodiscard]] std::string get_target_session_fen() const;

  // Non-blocking 2-phase planning
  [[nodiscard]] std::optional<PlanningSnapshot> prepare_planning_snapshot();
  void abort_planning();
  std::optional<DispatchedGoal> commit_selected_candidate(const domain::MoveCandidate &cand);

  // Single-call planning with decoupled feasibility checker
  std::optional<DispatchedGoal> plan_and_commit_best_candidate(
    domain::MoveCandidateSelector::FeasibilityChecker checker = nullptr);

  // Goal response & Result handlers
  void on_goal_rejected();
  void on_goal_aborted_or_canceled();
  domain::BoardTransitionOutcome on_collection_task_finished(
    bool action_success, const std::string &actual_fen, bool teleop_intervened = false);

  // Episode recording updates
  void on_episode_recorded(const std::string &instruction);

  // Diagnostics snapshot helper
  [[nodiscard]] RecorderDiagnosticsSnapshot capture_diagnostics_snapshot(
    const storage::DiskSpaceInfo &disk,
    EpisodeState ep_state,
    uint32_t current_ep_idx,
    uint64_t total_msgs,
    const std::string &active_instruction,
    bool is_action_server_connected) const;

private:
  mutable std::mutex state_mutex_;
  domain::CampaignConfig config_;
  std::filesystem::path storage_dir_;
  std::string experiment_name_;
  bool auto_advance_session_{true};

  CampaignState campaign_state_{CampaignState::IDLE_READY};
  domain::QuotaTracker quota_tracker_;
  domain::FenSynthesizer fen_synthesizer_;
  domain::MoveCandidateSelector move_selector_;

  std::optional<DispatchedGoal> active_goal_;
  uint32_t active_move_expected_episodes_{0};
  uint32_t active_move_recorded_episodes_{0};

  bool board_synced_{true};
  bool fen_sync_acknowledged_{true};
  std::string target_session_fen_;
};

}  // namespace chess_episode_recorder


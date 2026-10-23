// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include <diagnostic_updater/diagnostic_updater.hpp>

namespace chess_episode_recorder
{

/**
 * @brief Operational lifecycle states for episode recording to MCAP.
 */
enum class EpisodeState
{
  IDLE,       ///< Waiting for start trigger.
  RECORDING,  ///< Actively capturing messages to MCAP bag.
  STOPPED     ///< Trajectory ended, waiting for save or discard.
};

/**
 * @brief Finite State Machine states for the SmolVLA automated collection campaign conductor.
 */
enum class CampaignState
{
  DISABLED,               ///< Collection planner disabled (pure manual teleop mode).
  IDLE_AWAITING_BOARD,    ///< Waiting for stable chessboard state (/chess/game_status).
  PLANNING_OR_SYNTHESIS,  ///< Actively evaluating moves or synthesizing new FEN.
  DISPATCHED_TO_ORCH,     ///< Collection task sent to Orchestrator, waiting execution.
  CAMPAIGN_FINISHED       ///< All 600 episodes target reached.
};

/**
 * @brief Converts EpisodeState enum to human-readable string view.
 */
[[nodiscard]] constexpr std::string_view to_string(EpisodeState state) noexcept
{
  switch (state) {
    case EpisodeState::IDLE:
      return "IDLE";
    case EpisodeState::RECORDING:
      return "RECORDING";
    case EpisodeState::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
}

/**
 * @brief Converts CampaignState enum to human-readable string view.
 */
[[nodiscard]] constexpr std::string_view to_string(CampaignState state) noexcept
{
  switch (state) {
    case CampaignState::DISABLED:
      return "DISABLED";
    case CampaignState::IDLE_AWAITING_BOARD:
      return "IDLE_AWAITING_BOARD";
    case CampaignState::PLANNING_OR_SYNTHESIS:
      return "PLANNING_OR_SYNTHESIS";
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
  uint32_t current_episode_index{0};
  std::string base_output_dir{""};
  std::string active_instruction{""};
  uint64_t total_recorded_messages{0};
  uint64_t available_disk_space_bytes{0};
  bool disk_query_success{true};

  // Campaign specific metrics
  bool board_is_stable{false};
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
 * @brief Pure domain evaluator for episode recorder and collection campaign health.
 *
 * Implements Single Responsibility Principle and is 100% deterministically unit testable
 * without spinning ROS 2 node executors or spawning MCAP writers.
 */
class RecorderDiagnostics
{
public:
  static constexpr uint64_t WARN_DISK_SPACE_BYTES{1024ULL * 1024ULL * 1024ULL};  ///< 1 GB warning threshold
  static constexpr uint64_t ERROR_DISK_SPACE_BYTES{200ULL * 1024ULL * 1024ULL};  ///< 200 MB critical threshold

  /**
   * @brief Evaluates recording pipeline health, disk space, and FSM status.
   *
   * @param[in] snapshot Captured snapshot of the recorder state.
   * @param[out] stat Diagnostics status wrapper to populate.
   */
  static void evaluate_recorder_status(
    const RecorderDiagnosticsSnapshot &snapshot,
    diagnostic_updater::DiagnosticStatusWrapper &stat);

  /**
   * @brief Evaluates collection campaign progress, quota breakdown, and board stability.
   *
   * @param[in] snapshot Captured snapshot of the campaign state.
   * @param[out] stat Diagnostics status wrapper to populate.
   */
  static void evaluate_campaign_status(
    const RecorderDiagnosticsSnapshot &snapshot,
    diagnostic_updater::DiagnosticStatusWrapper &stat);
};

} // namespace chess_episode_recorder


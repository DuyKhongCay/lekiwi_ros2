// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "chess_episode_recorder/recorder_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace chess_episode_recorder
{

void RecorderDiagnostics::evaluate_recorder_status(
  const RecorderDiagnosticsSnapshot &snapshot,
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Episode State", std::string(to_string(snapshot.episode_state)));
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

  stat.add("Board Stable", snapshot.board_is_stable ? "true" : "false");

  stat.addf("MOVE Quotas", "%u / %u", snapshot.completed_move, snapshot.target_move);
  stat.addf(
    "CLEAR Quotas", "%u / %u (L: %u, R: %u)",
    snapshot.completed_clear, snapshot.target_clear,
    snapshot.clear_left_bin, snapshot.clear_right_bin);
  stat.addf("PICK Quotas", "%u / %u", snapshot.completed_pick, snapshot.target_pick);
  stat.addf("PLACE Quotas", "%u / %u", snapshot.completed_place, snapshot.target_place);

  if (snapshot.campaign_state == CampaignState::CAMPAIGN_FINISHED) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Campaign complete: target 600 episodes reached");
  } else if (snapshot.campaign_state == CampaignState::IDLE_AWAITING_BOARD && !snapshot.board_is_stable) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Campaign active: awaiting stable chessboard state");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Collection campaign nominal");
  }
}

} // namespace chess_episode_recorder


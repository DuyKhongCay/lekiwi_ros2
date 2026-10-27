// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "chess_episode_recorder/recorder_diagnostics.hpp"

namespace chess_episode_recorder
{
namespace test
{

class RecorderDiagnosticsTest : public ::testing::Test
{
protected:
  diagnostic_updater::DiagnosticStatusWrapper stat_;
};

TEST_F(RecorderDiagnosticsTest, RecorderStatusNominal)
{
  RecorderDiagnosticsSnapshot snap;
  snap.episode_state = EpisodeState::IDLE;
  snap.current_episode_index = 42;
  snap.base_output_dir = "/tmp/test_episodes";
  snap.total_recorded_messages = 1500;
  snap.active_instruction = "Move white pawn to target";
  snap.available_disk_space_bytes = 10ULL * 1024ULL * 1024ULL * 1024ULL; // 10 GB
  snap.disk_query_success = true;

  RecorderDiagnostics::evaluate_recorder_status(snap, stat_);

  EXPECT_EQ(stat_.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(stat_.message, "Episode recorder nominal");

  // Verify key metrics exist
  std::map<std::string, std::string> values;
  for (const auto &kv : stat_.values) {
    values[kv.key] = kv.value;
  }

  EXPECT_EQ(values["Episode State"], "IDLE");
  EXPECT_EQ(values["Current Episode Index"], "000042");
  EXPECT_EQ(values["Base Output Dir"], "/tmp/test_episodes");
  EXPECT_EQ(values["Total Recorded Msgs"], "1500");
  EXPECT_EQ(values["Active Instruction"], "Move white pawn to target");
  EXPECT_EQ(values["Disk Free Space (GB)"], "10.00");

  // Ensure "Topic Subscriptions" is NOT present (removed per user request)
  EXPECT_EQ(values.find("Topic Subscriptions"), values.end());
}

TEST_F(RecorderDiagnosticsTest, RecorderStatusDiskSpaceWarningsAndErrors)
{
  // 1. Low disk space (< 1 GB) -> WARN
  {
    RecorderDiagnosticsSnapshot snap;
    snap.available_disk_space_bytes = 500ULL * 1024ULL * 1024ULL; // 500 MB
    snap.disk_query_success = true;

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_recorder_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_TRUE(stat.message.find("Low disk space") != std::string::npos);
  }

  // 2. Critical disk space (< 200 MB) -> ERROR
  {
    RecorderDiagnosticsSnapshot snap;
    snap.available_disk_space_bytes = 150ULL * 1024ULL * 1024ULL; // 150 MB
    snap.disk_query_success = true;

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_recorder_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_TRUE(stat.message.find("Critical disk space") != std::string::npos);
  }

  // 3. Disk query failure -> ERROR
  {
    RecorderDiagnosticsSnapshot snap;
    snap.disk_query_success = false;

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_recorder_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_TRUE(stat.message.find("Failed to access") != std::string::npos);
  }
}

TEST_F(RecorderDiagnosticsTest, CampaignStatusDisabledManualTeleop)
{
  RecorderDiagnosticsSnapshot snap;
  snap.enable_collection_planner = false;

  RecorderDiagnostics::evaluate_campaign_status(snap, stat_);

  EXPECT_EQ(stat_.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_TRUE(stat_.message.find("Manual Teleop Mode") != std::string::npos);

  std::map<std::string, std::string> values;
  for (const auto &kv : stat_.values) {
    values[kv.key] = kv.value;
  }
  EXPECT_EQ(values["Campaign Mode"], "MANUAL_TELEOP");
  EXPECT_EQ(values["Campaign State"], "DISABLED");

  // Ensure "Last Seen FEN" is NOT present (removed per user request)
  EXPECT_EQ(values.find("Last Seen FEN"), values.end());
}

TEST_F(RecorderDiagnosticsTest, CampaignStatusActiveAndFinished)
{
  // 1. Active Campaign Nominal
  {
    RecorderDiagnosticsSnapshot snap;
    snap.enable_collection_planner = true;
    snap.campaign_state = CampaignState::DISPATCHED_TO_ORCH;
    snap.is_action_server_connected = true;
    snap.has_active_goal = true;
    snap.active_goal_uci = "e2e4";
    snap.total_completed_episodes = 120;
    snap.target_total_episodes = 600;
    snap.session_id = 6;
    snap.episodes_in_session = 14;
    snap.session_target_episodes = 20;

    snap.completed_move = 60;
    snap.target_move = 240;
    snap.completed_clear = 30;
    snap.target_clear = 120;
    snap.clear_left_bin = 15;
    snap.clear_right_bin = 15;
    snap.completed_pick = 15;
    snap.target_pick = 120;
    snap.completed_place = 15;
    snap.target_place = 120;

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_campaign_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(stat.message, "Collection campaign nominal");

    std::map<std::string, std::string> values;
    for (const auto &kv : stat.values) {
      values[kv.key] = kv.value;
    }
    EXPECT_EQ(values["Campaign Mode"], "ENABLED");
    EXPECT_EQ(values["Campaign State"], "DISPATCHED_TO_ORCH");
    EXPECT_EQ(values["Action Server Connected"], "true");
    EXPECT_EQ(values["Active Goal Dispatched"], "true");
    EXPECT_EQ(values["Active Goal UCI"], "e2e4");
    EXPECT_EQ(values["Total Progress"], "120 / 600 (20.0%)");
    EXPECT_EQ(values["Session Progress"], "Session 6: 14 / 20");
    EXPECT_EQ(values["MOVE Quotas"], "60 / 240");
    EXPECT_EQ(values["CLEAR Quotas"], "30 / 120 (L: 15, R: 15)");
    EXPECT_EQ(values["PICK Quotas"], "15 / 120");
    EXPECT_EQ(values["PLACE Quotas"], "15 / 120");

    // Ensure "Last Seen FEN" is NOT present (removed per user request)
    EXPECT_EQ(values.find("Last Seen FEN"), values.end());
  }

  // 2. Campaign Finished
  {
    RecorderDiagnosticsSnapshot snap;
    snap.enable_collection_planner = true;
    snap.campaign_state = CampaignState::CAMPAIGN_FINISHED;
    snap.total_completed_episodes = 600;
    snap.target_total_episodes = 600;

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_campaign_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_TRUE(stat.message.find("target 600 episodes reached") != std::string::npos);
  }

  // 3. Waiting for Orchestrator Action Server
  {
    RecorderDiagnosticsSnapshot snap;
    snap.enable_collection_planner = true;
    snap.campaign_state = CampaignState::IDLE_READY;
    snap.is_action_server_connected = false;

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_campaign_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_TRUE(stat.message.find("Waiting for Orchestrator ExecuteCollectionTask action server") != std::string::npos);

    std::map<std::string, std::string> values;
    for (const auto &kv : stat.values) {
      values[kv.key] = kv.value;
    }
    EXPECT_EQ(values["Campaign State"], "IDLE_READY");
    EXPECT_EQ(values["Action Server Connected"], "false");
  }

  // 4. Action Server Connected with Active Goal
  {
    RecorderDiagnosticsSnapshot snap;
    snap.enable_collection_planner = true;
    snap.campaign_state = CampaignState::DISPATCHED_TO_ORCH;
    snap.is_action_server_connected = true;
    snap.has_active_goal = true;
    snap.active_goal_uci = "g1f3";

    diagnostic_updater::DiagnosticStatusWrapper stat;
    RecorderDiagnostics::evaluate_campaign_status(snap, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(stat.message, "Collection campaign nominal");

    std::map<std::string, std::string> values;
    for (const auto &kv : stat.values) {
      values[kv.key] = kv.value;
    }
    EXPECT_EQ(values["Campaign State"], "DISPATCHED_TO_ORCH");
    EXPECT_EQ(values["Action Server Connected"], "true");
    EXPECT_EQ(values["Active Goal Dispatched"], "true");
    EXPECT_EQ(values["Active Goal UCI"], "g1f3");
  }
}

TEST_F(RecorderDiagnosticsTest, CampaignStateToStringConversions)
{
  EXPECT_EQ(to_string(CampaignState::DISABLED), "DISABLED");
  EXPECT_EQ(to_string(CampaignState::IDLE_READY), "IDLE_READY");
  EXPECT_EQ(to_string(CampaignState::PLANNING_OR_SYNTHESIS), "PLANNING_OR_SYNTHESIS");
  EXPECT_EQ(to_string(CampaignState::AWAITING_PHYSICAL_SYNC), "AWAITING_PHYSICAL_SYNC");
  EXPECT_EQ(to_string(CampaignState::DISPATCHED_TO_ORCH), "DISPATCHED_TO_ORCH");
  EXPECT_EQ(to_string(CampaignState::CAMPAIGN_FINISHED), "CAMPAIGN_FINISHED");
}

} // namespace test
} // namespace chess_episode_recorder


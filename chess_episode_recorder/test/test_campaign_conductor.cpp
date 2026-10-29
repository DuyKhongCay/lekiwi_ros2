#include <gtest/gtest.h>

#include <filesystem>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "chess_episode_recorder/campaign_quota_domain.hpp"
#include "chess_episode_recorder/move_planning_domain.hpp"
#include "chess_episode_recorder/dataset_storage.hpp"
#include "chess_episode_recorder/campaign_conductor.hpp"

using namespace chess_episode_recorder;
using namespace chess_episode_recorder::domain;
using namespace chess_episode_recorder::storage;

class CampaignConductorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = std::filesystem::temp_directory_path() / "lekiwi_test_conductor";
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(test_dir_);
  }

  std::filesystem::path test_dir_;
};

TEST_F(CampaignConductorTest, InitializationAndStorageRecovery)
{
  // Prepare an existing episode on disk
  auto ep0 = test_dir_ / "episode_000000";
  std::filesystem::create_directories(ep0);
  EpisodeMetadata meta;
  meta.episode_id = 0;
  meta.task = "Move white knight from b1 to c3";
  DatasetDirectoryScanner::write_episode_info(ep0, meta);

  CampaignConfig config;
  config.total_episodes = 10;
  CampaignConductor conductor(config, test_dir_, "exp_test", true);

  uint32_t recovered = conductor.initialize_from_storage(false);
  EXPECT_EQ(recovered, 1u);

  auto sum = conductor.get_quota_summary();
  EXPECT_EQ(sum.total_completed, 1u);
  EXPECT_EQ(sum.completed_by_kind.at(StepKind::MOVE), 1u);
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::IDLE_READY);
}

TEST_F(CampaignConductorTest, SyncBarrier_LocksUntilSyncedAndAcked)
{
  CampaignConfig config;
  CampaignConductor conductor(config, test_dir_, "exp_test", true);
  conductor.initialize_from_storage(false);

  conductor.advance_to_new_session();
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::AWAITING_PHYSICAL_SYNC);
  EXPECT_FALSE(conductor.is_board_synced());
  EXPECT_FALSE(conductor.is_fen_sync_acknowledged());

  // Cannot plan while awaiting sync
  auto snap1 = conductor.prepare_planning_snapshot();
  EXPECT_FALSE(snap1.has_value());

  // Acknowledge FEN from orchestrator
  conductor.set_fen_sync_acknowledged(true);
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::AWAITING_PHYSICAL_SYNC);

  // Still cannot plan until physical board is synced
  auto snap2 = conductor.prepare_planning_snapshot();
  EXPECT_FALSE(snap2.has_value());

  // Set board synced
  conductor.set_board_synced(true);
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::IDLE_READY);

  // Now planning snapshot is generated
  auto snap3 = conductor.prepare_planning_snapshot();
  EXPECT_TRUE(snap3.has_value());
}

TEST_F(CampaignConductorTest, NonBlockingTwoPhasePlanningAndDispatch)
{
  CampaignConfig config;
  CampaignConductor conductor(config, test_dir_, "exp_test", true);
  conductor.initialize_from_storage(false);

  conductor.set_board_synced(true);
  conductor.set_fen_sync_acknowledged(true);

  // Phase 1: Snapshot candidates
  auto snap = conductor.prepare_planning_snapshot();
  ASSERT_TRUE(snap.has_value());
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::PLANNING_OR_SYNTHESIS);
  EXPECT_FALSE(snap->current_fen.empty());

  // Outside the lock: select best candidate (using dummy feasibility checker)
  MoveCandidateSelector selector;
  auto best = selector.select_best_move(
    snap->current_board,
    snap->tracker,
    [](const MoveCandidate &) -> std::pair<bool, uint8_t> {
      return {true, 0};
    });
  ASSERT_TRUE(best.has_value());

  // Phase 2: Commit selected candidate
  auto dispatched = conductor.commit_selected_candidate(*best);
  ASSERT_TRUE(dispatched.has_value());
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::DISPATCHED_TO_ORCH);
  EXPECT_TRUE(conductor.has_active_goal());
  EXPECT_EQ(conductor.get_active_goal()->candidate.uci, best->uci);

  // On collection task finished: updates board and resets state
  auto outcome = conductor.on_collection_task_finished(
    true, dispatched->expected_target_fen);
  EXPECT_EQ(outcome, BoardTransitionOutcome::MATCHED_EXPECTED_MOVE);
  EXPECT_EQ(conductor.get_campaign_state(), CampaignState::IDLE_READY);
  EXPECT_FALSE(conductor.has_active_goal());
}

TEST_F(CampaignConductorTest, TeleopDivergenceResyncHandling)
{
  CampaignConfig config;
  CampaignConductor conductor(config, test_dir_, "exp_test", true);
  conductor.initialize_from_storage(false);

  conductor.set_board_synced(true);
  conductor.set_fen_sync_acknowledged(true);

  auto snap = conductor.prepare_planning_snapshot();
  ASSERT_TRUE(snap.has_value());

  MoveCandidateSelector selector;
  auto best = selector.select_best_move(snap->current_board, snap->tracker, nullptr);
  ASSERT_TRUE(best.has_value());

  auto dispatched = conductor.commit_selected_candidate(*best);
  ASSERT_TRUE(dispatched.has_value());

  // Teleop played a totally different move:
  std::string diverged_fen = "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2";
  auto outcome = conductor.on_collection_task_finished(true, diverged_fen);

  EXPECT_EQ(outcome, BoardTransitionOutcome::DIVERGED_RESYNCED);
  EXPECT_FALSE(conductor.is_fen_sync_acknowledged());
  EXPECT_EQ(conductor.get_session_state().active_fen, diverged_fen);
}

TEST_F(CampaignConductorTest, RecorderDiagnosticsEvaluator)
{
  RecorderDiagnosticsSnapshot snap;
  snap.episode_state = EpisodeState::RECORDING;
  snap.campaign_state = CampaignState::DISPATCHED_TO_ORCH;
  snap.enable_collection_planner = true;
  snap.is_action_server_connected = true;
  snap.has_active_goal = true;
  snap.active_goal_uci = "e2e4";
  snap.available_disk_space_bytes = 10ULL * 1024ULL * 1024ULL * 1024ULL; // 10 GB
  snap.disk_query_success = true;

  diagnostic_updater::DiagnosticStatusWrapper stat;
  RecorderDiagnostics::evaluate_recorder_status(snap, stat);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  diagnostic_updater::DiagnosticStatusWrapper camp_stat;
  RecorderDiagnostics::evaluate_campaign_status(snap, camp_stat);
  EXPECT_EQ(camp_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  // Test critical disk warning
  snap.available_disk_space_bytes = 100ULL * 1024ULL * 1024ULL; // 100 MB < 200 MB
  diagnostic_updater::DiagnosticStatusWrapper err_stat;
  RecorderDiagnostics::evaluate_recorder_status(snap, err_stat);
  EXPECT_EQ(err_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}


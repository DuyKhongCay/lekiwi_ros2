#include <gtest/gtest.h>

#include "chess_episode_recorder/campaign_quota_domain.hpp"

using namespace chess_episode_recorder::domain;

class CampaignQuotaDomainTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(CampaignQuotaDomainTest, InstructionParser_StandardTasks)
{
  auto res1 = InstructionParser::parse("Move white knight from b1 to c3");
  ASSERT_TRUE(res1.has_value());
  EXPECT_EQ(res1->kind, StepKind::MOVE);
  EXPECT_EQ(res1->piece.color, chess::Color::WHITE);
  EXPECT_EQ(res1->piece.type, chess::PieceType::KNIGHT);

  auto res2 = InstructionParser::parse("Clear black pawn from d4 to left bin");
  ASSERT_TRUE(res2.has_value());
  EXPECT_EQ(res2->kind, StepKind::CLEAR);
  EXPECT_EQ(res2->piece.color, chess::Color::BLACK);
  EXPECT_EQ(res2->piece.type, chess::PieceType::PAWN);
  EXPECT_EQ(res2->bin_side, "left");

  auto res3 = InstructionParser::parse("Pick white bishop at c1");
  ASSERT_TRUE(res3.has_value());
  EXPECT_EQ(res3->kind, StepKind::PICK);
  EXPECT_EQ(res3->piece.color, chess::Color::WHITE);
  EXPECT_EQ(res3->piece.type, chess::PieceType::BISHOP);

  auto res4 = InstructionParser::parse("Place white bishop on f4");
  ASSERT_TRUE(res4.has_value());
  EXPECT_EQ(res4->kind, StepKind::PLACE);
  EXPECT_EQ(res4->piece.color, chess::Color::WHITE);
  EXPECT_EQ(res4->piece.type, chess::PieceType::BISHOP);
}

TEST_F(CampaignQuotaDomainTest, InstructionParser_InvalidOrUnknown)
{
  EXPECT_FALSE(InstructionParser::parse("").has_value());
  EXPECT_FALSE(InstructionParser::parse("Dance around the board").has_value());
  EXPECT_FALSE(InstructionParser::parse("Move alien robot from z9 to w0").has_value());
}

TEST_F(CampaignQuotaDomainTest, QuotaTracker_DefaultQuotasAndRecording)
{
  CampaignConfig config;
  config.total_episodes = 600;
  config.episodes_per_session = 20;

  QuotaTracker tracker(config);
  auto summary = tracker.get_summary();
  EXPECT_EQ(summary.total_target, 600u);
  EXPECT_EQ(summary.total_completed, 0u);
  EXPECT_FALSE(tracker.is_campaign_complete());

  EXPECT_TRUE(tracker.record_episode("Move white knight from b1 to c3"));
  summary = tracker.get_summary();
  EXPECT_EQ(summary.total_completed, 1u);
  EXPECT_EQ(summary.completed_by_kind.at(StepKind::MOVE), 1u);

  EXPECT_TRUE(tracker.record_episode("Clear black pawn from d4 to left bin"));
  summary = tracker.get_summary();
  EXPECT_EQ(summary.total_completed, 2u);
  EXPECT_EQ(summary.completed_by_kind.at(StepKind::CLEAR), 1u);
  EXPECT_EQ(summary.clear_left_completed, 1u);

  EXPECT_TRUE(tracker.record_episode("Pick white bishop at c1"));
  EXPECT_TRUE(tracker.record_episode("Place white bishop on f4"));
  summary = tracker.get_summary();
  EXPECT_EQ(summary.total_completed, 4u);
  EXPECT_EQ(summary.completed_by_kind.at(StepKind::PICK), 1u);
  EXPECT_EQ(summary.completed_by_kind.at(StepKind::PLACE), 1u);
}

TEST_F(CampaignQuotaDomainTest, QuotaTracker_ConfigAutoDerivation)
{
  CampaignConfig custom;
  custom.total_episodes = 100;
  // Others 0 -> should auto-derive 40% MOVE, 20% CLEAR, 20% PICK, 20% PLACE
  custom.move_total = 0;
  custom.clear_total = 0;
  custom.pick_total = 0;
  custom.place_total = 0;

  QuotaTracker tracker(custom);
  const auto &derived = tracker.get_config();
  EXPECT_EQ(derived.total_episodes, 100u);
  EXPECT_EQ(derived.move_total, 40u);
  EXPECT_EQ(derived.clear_total, 20u);
  EXPECT_EQ(derived.pick_total, 20u);
  EXPECT_EQ(derived.place_total, 20u);
}

TEST_F(CampaignQuotaDomainTest, QuotaTracker_BoardProgressionAndMoveCommit)
{
  QuotaTracker tracker;
  std::string start_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
  tracker.reset_session(start_fen);

  EXPECT_EQ(tracker.get_active_fen(), start_fen);
  EXPECT_TRUE(tracker.get_session_state().is_active);

  // 1. Valid expected move: e2e4
  std::string expected_fen_after_e4 = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1";
  auto outcome = tracker.verify_and_commit_move("e2e4", expected_fen_after_e4);
  EXPECT_EQ(outcome, BoardTransitionOutcome::MATCHED_EXPECTED_MOVE);
  EXPECT_EQ(tracker.get_active_fen(), expected_fen_after_e4);
  EXPECT_EQ(tracker.get_session_state().executed_moves.size(), 1u);
  EXPECT_EQ(tracker.get_session_state().executed_moves[0], "e2e4");

  // 2. Divergence: expected e7e5, but human played d7d5 instead
  std::string actual_fen_d5 = "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2";
  auto div_outcome = tracker.verify_and_commit_move("e7e5", actual_fen_d5);
  EXPECT_EQ(div_outcome, BoardTransitionOutcome::DIVERGED_RESYNCED);
  EXPECT_EQ(tracker.get_active_fen(), actual_fen_d5);
  // Executed moves cleared upon divergence to restart sequential move history
  EXPECT_EQ(tracker.get_session_state().executed_moves.size(), 0u);

  // 3. Invalid FEN reported
  auto inv_outcome = tracker.verify_and_commit_move("e4d5", "invalid fen string");
  EXPECT_EQ(inv_outcome, BoardTransitionOutcome::INVALID_ACTUAL_FEN);
}

TEST_F(CampaignQuotaDomainTest, QuotaTracker_SessionStateRestore)
{
  QuotaTracker tracker;
  SessionState state;
  state.session_id = 42;
  state.episodes_in_session = 15;
  state.initial_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
  state.executed_moves = {"e2e4", "e7e5", "g1f3"};
  state.is_active = true;

  tracker.restore_session_state(state);
  EXPECT_TRUE(tracker.get_session_state().is_active);
  EXPECT_EQ(tracker.get_session_state().session_id, 42u);
  EXPECT_EQ(tracker.get_session_state().episodes_in_session, 15u);
  EXPECT_EQ(tracker.get_session_state().executed_moves.size(), 3u);

  // The active board should have e2e4, e7e5, g1f3 applied
  EXPECT_NE(tracker.get_active_fen(), state.initial_fen);
}


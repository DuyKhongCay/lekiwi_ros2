#include <gtest/gtest.h>

#include "chess_episode_recorder/campaign_quota_domain.hpp"
#include "chess_episode_recorder/move_planning_domain.hpp"

using namespace chess_episode_recorder::domain;

class MovePlanningDomainTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(MovePlanningDomainTest, FenSynthesizer_GeneratesValidPositions)
{
  FenSynthesizer synthesizer;
  QuotaTracker tracker;

  for (int i = 0; i < 10; ++i) {
    std::string fen = synthesizer.synthesize_fen(tracker);
    EXPECT_FALSE(fen.empty());

    chess::Board board;
    ASSERT_TRUE(board.setFen(fen)) << "Generated invalid FEN: " << fen;

    // Both kings must exist
    EXPECT_NE(board.kingSq(chess::Color::WHITE), chess::Square::NO_SQ);
    EXPECT_NE(board.kingSq(chess::Color::BLACK), chess::Square::NO_SQ);

    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board);
    EXPECT_GT(legal_moves.size(), 0u) << "Synthesized board should have legal moves: " << fen;
  }
}

TEST_F(MovePlanningDomainTest, MoveCandidate_SemanticProperties)
{
  chess::Board board("rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1");
  chess::Move move = chess::uci::uciToMove(board, "d7d5");
  MoveCandidate cand = MoveCandidateSelector::classify_chess_move(board, move);

  EXPECT_EQ(cand.uci, "d7d5");
  EXPECT_EQ(cand.from_sq(), "d7");
  EXPECT_EQ(cand.to_sq(), "d5");
  EXPECT_FALSE(cand.is_capture(board));
  EXPECT_FALSE(cand.is_castling());
  EXPECT_FALSE(cand.is_en_passant());
  EXPECT_EQ(cand.moving_piece(board).type, chess::PieceType::PAWN);
  EXPECT_EQ(cand.moving_piece(board).color, chess::Color::BLACK);

  std::string target_fen = cand.calculate_target_fen(board);
  EXPECT_FALSE(target_fen.empty());
  EXPECT_NE(target_fen, board.getFen());
}

TEST_F(MovePlanningDomainTest, MoveCandidateSelector_DowngradeAndBypass)
{
  QuotaTracker tracker;
  MoveCandidateSelector selector;

  // Board with simple position: White Pawn on e2, Black King on e8, White King on e1
  chess::Board board;
  ASSERT_TRUE(board.setFen("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1"));

  // Case A: White Pawn still lacks MOVE quota -> force_pick_place is false
  auto candidates_a = selector.evaluate_candidates(board, tracker, nullptr);
  ASSERT_FALSE(candidates_a.empty());
  EXPECT_FALSE(candidates_a[0].force_pick_place);
  EXPECT_TRUE(candidates_a[0].bypass_steps.empty());
  EXPECT_GT(candidates_a[0].score, 0.0);

  // Case B: Manually saturate White Pawn MOVE quota (20 done)
  PieceKey wp{chess::Color::WHITE, chess::PieceType::PAWN};
  for (int i = 0; i < 20; ++i) {
    tracker.record_episode("Move white pawn to target");
  }
  EXPECT_EQ(tracker.get_remaining_quota(StepKind::MOVE, wp), 0u);

  // Single-base move e2-e3 or e2-e4 should be downgraded (force_pick_place = true)
  auto candidates_b = selector.evaluate_candidates(board, tracker, nullptr);
  ASSERT_FALSE(candidates_b.empty());
  EXPECT_TRUE(candidates_b[0].force_pick_place);
  EXPECT_TRUE(candidates_b[0].bypass_steps.empty());
}

TEST_F(MovePlanningDomainTest, MoveCandidateSelector_PreventsRepeatingExecutedMoves)
{
  MoveCandidateSelector selector;
  QuotaTracker tracker;
  FenSynthesizer synthesizer;

  chess::Board board("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  tracker.reset_session(board.getFen());

  // Mark e2e4 as executed in this session
  auto sess = tracker.get_session_state();
  sess.executed_moves.push_back("e2e4");
  tracker.set_session_state(sess);

  auto best = selector.select_best_move(board, tracker, nullptr);
  ASSERT_TRUE(best.has_value());
  EXPECT_NE(best->uci, "e2e4");
}


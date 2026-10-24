// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "chess_episode_recorder/collection_planner_domain.hpp"

namespace chess_episode_recorder::domain::test
{

class CollectionPlannerDomainTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    temp_dir_ = std::filesystem::temp_directory_path() / "test_collection_planner_domain";
    std::filesystem::remove_all(temp_dir_);
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
  }

  std::filesystem::path temp_dir_;
};

// 1. QuotaTracker: Parsing SmolVLA task instructions
TEST_F(CollectionPlannerDomainTest, QuotaTracker_ParseSmolVLAInstructions)
{
  QuotaTracker tracker;
  PieceKey wp{chess::Color::WHITE, chess::PieceType::PAWN};
  PieceKey bn{chess::Color::BLACK, chess::PieceType::KNIGHT};
  PieceKey wq{chess::Color::WHITE, chess::PieceType::QUEEN};

  EXPECT_EQ(tracker.get_completed(StepKind::MOVE, wp), 0u);
  EXPECT_EQ(tracker.get_completed(StepKind::CLEAR, bn), 0u);
  EXPECT_EQ(tracker.get_completed(StepKind::PICK, wq), 0u);
  EXPECT_EQ(tracker.get_completed(StepKind::PLACE, wq), 0u);

  // Parse MOVE
  EXPECT_TRUE(tracker.record_episode("Move white pawn to target"));
  EXPECT_EQ(tracker.get_completed(StepKind::MOVE, wp), 1u);
  EXPECT_EQ(tracker.get_repeat_count("w_P"), 1u);

  // Parse CLEAR
  EXPECT_TRUE(tracker.record_episode("Clear black knight to left bin"));
  EXPECT_EQ(tracker.get_completed(StepKind::CLEAR, bn), 1u);
  EXPECT_EQ(tracker.get_summary().clear_left_completed, 1u);
  EXPECT_EQ(tracker.get_summary().clear_right_completed, 0u);

  // Next bin should be right (due to left having 1)
  EXPECT_EQ(tracker.get_next_clear_bin_side(), "right");

  // Parse CLEAR right
  EXPECT_TRUE(tracker.record_episode("Clear black knight to right bin"));
  EXPECT_EQ(tracker.get_completed(StepKind::CLEAR, bn), 2u);
  EXPECT_EQ(tracker.get_summary().clear_right_completed, 1u);

  // Parse PICK & PLACE
  EXPECT_TRUE(tracker.record_episode("Pick white queen"));
  EXPECT_EQ(tracker.get_completed(StepKind::PICK, wq), 1u);

  EXPECT_TRUE(tracker.record_episode("Place white queen"));
  EXPECT_EQ(tracker.get_completed(StepKind::PLACE, wq), 1u);

  // Invalid task
  EXPECT_FALSE(tracker.record_episode("Invalid random text"));
}

// 2. QuotaTracker: Crash Recovery from directory scanning
TEST_F(CollectionPlannerDomainTest, QuotaTracker_ScanAndRecover)
{
  // Create mock episode directories with episode_info.yaml
  auto create_mock_episode = [this](int id, const std::string &task_instruction) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "episode_%06d", id);
    auto ep_dir = temp_dir_ / buf;
    std::filesystem::create_directories(ep_dir);
    auto yaml_path = ep_dir / "episode_info.yaml";

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "episode_id" << YAML::Value << id;
    out << YAML::Key << "task" << YAML::Value << task_instruction;
    out << YAML::EndMap;

    std::ofstream fout(yaml_path);
    fout << out.c_str();
  };

  create_mock_episode(0, "Move white pawn to target");
  create_mock_episode(1, "Move white pawn to target"); // repeat
  create_mock_episode(2, "Clear black bishop to left bin");
  create_mock_episode(3, "Pick black rook");
  create_mock_episode(4, "Place black rook");

  QuotaTracker tracker;
  uint32_t recovered = tracker.scan_and_recover(temp_dir_);
  EXPECT_EQ(recovered, 5u);

  PieceKey wp{chess::Color::WHITE, chess::PieceType::PAWN};
  PieceKey bb{chess::Color::BLACK, chess::PieceType::BISHOP};
  PieceKey br{chess::Color::BLACK, chess::PieceType::ROOK};

  EXPECT_EQ(tracker.get_completed(StepKind::MOVE, wp), 2u);
  EXPECT_EQ(tracker.get_repeat_count("w_P"), 2u);
  EXPECT_EQ(tracker.get_completed(StepKind::CLEAR, bb), 1u);
  EXPECT_EQ(tracker.get_summary().clear_left_completed, 1u);
  EXPECT_EQ(tracker.get_completed(StepKind::PICK, br), 1u);
  EXPECT_EQ(tracker.get_completed(StepKind::PLACE, br), 1u);
}

// 3. QuotaTracker: Session State Save and Load
TEST_F(CollectionPlannerDomainTest, QuotaTracker_SessionStatePersistence)
{
  QuotaTracker tracker;
  SessionState state;
  state.session_id = 3;
  state.episodes_in_session = 14;
  state.initial_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1";
  state.active_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1";
  state.is_active = true;
  tracker.set_session_state(state);

  auto state_file = temp_dir_ / "session_state.yaml";
  EXPECT_TRUE(tracker.save_session_state(state_file));

  QuotaTracker tracker2;
  EXPECT_TRUE(tracker2.load_session_state(state_file));
  auto loaded = tracker2.get_session_state();

  EXPECT_EQ(loaded.session_id, 3u);
  EXPECT_EQ(loaded.episodes_in_session, 14u);
  chess::Board canonical_board;
  canonical_board.setFen("rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1");
  EXPECT_EQ(loaded.active_fen, canonical_board.getFen());
  EXPECT_TRUE(loaded.is_active);
}

// 4. FenSynthesizer: 100 consecutive valid FEN generations
TEST_F(CollectionPlannerDomainTest, FenSynthesizer_GeneratesValidPositions)
{
  QuotaTracker tracker;
  FenSynthesizer synth(1337);

  for (int i = 0; i < 100; ++i) {
    std::string fen = synth.synthesize_fen(tracker);
    ASSERT_FALSE(fen.empty());

    chess::Board board;
    ASSERT_TRUE(board.setFen(fen)) << "Failed to parse synthesized FEN: " << fen;

    // Must have exactly 1 White King and 1 Black King
    EXPECT_EQ(board.pieces(chess::PieceType::KING, chess::Color::WHITE).count(), 1);
    EXPECT_EQ(board.pieces(chess::PieceType::KING, chess::Color::BLACK).count(), 1);

    // Total pieces must be 8 to 12
    int total_pieces = board.occ().count();
    EXPECT_GE(total_pieces, 8);
    EXPECT_LE(total_pieces, 12);

    // Pawns cannot be on rank 1 or 8
    auto rank_1_8 = chess::Bitboard(chess::Rank::RANK_1) | chess::Bitboard(chess::Rank::RANK_8);
    EXPECT_EQ((board.pieces(chess::PieceType::PAWN) & rank_1_8).count(), 0);

    // Inactive king cannot be attacked
    chess::Color inactive = ~board.sideToMove();
    EXPECT_FALSE(board.isAttacked(board.kingSq(inactive), board.sideToMove()));

    // Active side must have >= 8 legal moves
    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board);
    EXPECT_GE(legal_moves.size(), 8u);
  }
}

// 5. MoveCandidateSelector: Legal move evaluation, Downgrade & Bypass logic
TEST_F(CollectionPlannerDomainTest, MoveCandidateSelector_DowngradeAndBypass)
{
  QuotaTracker tracker;
  MoveCandidateSelector selector;

  // Board with simple position: White Pawn on e2, Black King on e8, White King on e1
  chess::Board board;
  ASSERT_TRUE(board.setFen("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1"));

  // Check moving piece = White Pawn
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

  // Now White Pawn has 0 MOVE quota remaining, but PICK & PLACE quotas are available (10 each)
  // Single-base move e2-e3 or e2-e4 should be downgraded (force_pick_place = true)
  auto candidates_b = selector.evaluate_candidates(board, tracker, nullptr);
  ASSERT_FALSE(candidates_b.empty());
  EXPECT_TRUE(candidates_b[0].force_pick_place);
  EXPECT_TRUE(candidates_b[0].bypass_steps.empty()); // PICK and PLACE are active

  // Case C: Also saturate White Pawn PICK quota (10 done)
  for (int i = 0; i < 10; ++i) {
    tracker.record_episode("Pick white pawn");
  }
  EXPECT_EQ(tracker.get_remaining_quota(StepKind::PICK, wp), 0u);

  // Now PICK should be bypassed!
  auto candidates_c = selector.evaluate_candidates(board, tracker, nullptr);
  ASSERT_FALSE(candidates_c.empty());

  // Find the Pawn candidate specifically (since King moves with no bypass have higher score)
  auto it_pawn = std::find_if(candidates_c.begin(), candidates_c.end(), [&](const MoveCandidate &c) {
    return c.moving_piece(board).type == chess::PieceType::PAWN;
  });
  ASSERT_NE(it_pawn, candidates_c.end());
  EXPECT_TRUE(it_pawn->force_pick_place);
  bool has_pick_bypass = false;
  for (const auto &step : it_pawn->bypass_steps) {
    if (step == "PICK") has_pick_bypass = true;
  }
  EXPECT_TRUE(has_pick_bypass);

  // Case D: Also saturate White Pawn PLACE quota (10 done) -> All steps bypassed!
  for (int i = 0; i < 10; ++i) {
    tracker.record_episode("Place white pawn");
  }
  EXPECT_EQ(tracker.get_remaining_quota(StepKind::PLACE, wp), 0u);

  // Now White Pawn has MOVE, PICK, PLACE all saturated -> Score = 0, candidate rejected
  auto candidates_d = selector.evaluate_candidates(board, tracker, nullptr);
  for (const auto &cand : candidates_d) {
    // Only King moves should remain with positive score
    EXPECT_NE(cand.moving_piece(board).type, chess::PieceType::PAWN);
  }
}

// 6. MoveCandidateSelector: Verify non-repetition of executed moves & continuous play
TEST_F(CollectionPlannerDomainTest, MoveCandidateSelector_PreventsRepeatingExecutedMoves)
{
  QuotaTracker tracker;
  MoveCandidateSelector selector;

  chess::Board board;
  ASSERT_TRUE(board.setFen("r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/2N2N2/PPPP1PPP/R1BQK2R w KQkq - 4 4"));

  auto candidates = selector.evaluate_candidates(board, tracker, nullptr);
  ASSERT_FALSE(candidates.empty());
  std::string first_move_uci = candidates[0].uci;

  // Mark the move as executed in the session
  tracker.record_executed_move(first_move_uci);
  EXPECT_TRUE(tracker.has_executed_move(first_move_uci));

  // Evaluate candidates again on the same position
  auto candidates_after = selector.evaluate_candidates(board, tracker, nullptr);
  for (const auto &cand : candidates_after) {
    EXPECT_NE(cand.uci, first_move_uci);
  }

  // Other legal moves are still available and game continues
  EXPECT_FALSE(candidates_after.empty());

  // Resetting session clears executed moves history
  tracker.advance_session();
  EXPECT_FALSE(tracker.has_executed_move(first_move_uci));
  auto candidates_new_session = selector.evaluate_candidates(board, tracker, nullptr);
  bool found_first_move = false;
  for (const auto &cand : candidates_new_session) {
    if (cand.uci == first_move_uci) found_first_move = true;
  }
  EXPECT_TRUE(found_first_move);
}

// 7. QuotaTracker: Sequential Move Progression & Board Reconstruction
TEST_F(CollectionPlannerDomainTest, QuotaTracker_SequentialMoveProgressionAndBoardReconstruction)
{
  QuotaTracker tracker;
  const std::string init_fen = "r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/2N2N2/PPPP1PPP/R1BQK2R w KQkq - 4 4";
  tracker.reset_session(init_fen);

  EXPECT_EQ(tracker.get_active_board().sideToMove(), chess::Color::WHITE);

  // Move 1: White Bishop takes pawn on f7 with check (c4f7)
  EXPECT_TRUE(tracker.apply_move("c4f7"));
  EXPECT_EQ(tracker.get_active_board().sideToMove(), chess::Color::BLACK);
  EXPECT_TRUE(tracker.has_executed_move("c4f7"));
  EXPECT_NE(tracker.get_active_fen(), init_fen);

  // Move 2: Black King takes Bishop on f7 (e8f7)
  EXPECT_TRUE(tracker.apply_move("e8f7"));
  EXPECT_EQ(tracker.get_active_board().sideToMove(), chess::Color::WHITE);
  EXPECT_TRUE(tracker.has_executed_move("e8f7"));

  // Persist session state
  auto state_file = temp_dir_ / "session_state.yaml";
  EXPECT_TRUE(tracker.save_session_state(state_file));

  // Reconstruct in a new tracker
  QuotaTracker tracker2;
  EXPECT_TRUE(tracker2.load_session_state(state_file));
  EXPECT_EQ(tracker2.get_active_fen(), tracker.get_active_fen());
  EXPECT_EQ(tracker2.get_active_board().sideToMove(), chess::Color::WHITE);
  EXPECT_EQ(tracker2.get_executed_moves().size(), 2u);
  EXPECT_TRUE(tracker2.has_executed_move("c4f7"));
  EXPECT_TRUE(tracker2.has_executed_move("e8f7"));
}

// 8. MoveCandidateSelector: Sequential turns prevent moved piece repetition (Bug 2 Fix Verification)
TEST_F(CollectionPlannerDomainTest, MoveCandidateSelector_SequentialTurnsPreventsMovedPieceRepetition)
{
  QuotaTracker tracker;
  MoveCandidateSelector selector;

  // Exact FEN from Session 11 bug report
  const std::string session11_fen = "2k5/P2RR3/2r5/3b4/1pP5/8/2p5/6K1 w - - 0 1";
  tracker.reset_session(session11_fen);

  // Initial turn is White
  EXPECT_EQ(tracker.get_active_board().sideToMove(), chess::Color::WHITE);

  // White Rook moves from e7 to e3 ("e7e3")
  EXPECT_TRUE(tracker.apply_move("e7e3"));
  EXPECT_TRUE(tracker.has_executed_move("e7e3"));

  // After e7e3, turn must be Black!
  EXPECT_EQ(tracker.get_active_board().sideToMove(), chess::Color::BLACK);

  // Candidates generated for next turn must be for Black pieces
  auto black_candidates = selector.evaluate_candidates(tracker.get_active_board(), tracker, nullptr);
  ASSERT_FALSE(black_candidates.empty());
  for (const auto &cand : black_candidates) {
    EXPECT_EQ(cand.moving_piece(tracker.get_active_board()).color, chess::Color::BLACK);
    // Square e7 is now empty; no move can start from e7!
    EXPECT_NE(cand.from_sq(), "e7");
  }

  // Black moves King or piece, e.g. c6c4
  EXPECT_TRUE(tracker.apply_move("c6c4"));
  EXPECT_EQ(tracker.get_active_board().sideToMove(), chess::Color::WHITE);

  // Now White's turn again. Square e7 is still empty, rook is at e3
  auto white_candidates = selector.evaluate_candidates(tracker.get_active_board(), tracker, nullptr);
  ASSERT_FALSE(white_candidates.empty());
  for (const auto &cand : white_candidates) {
    // Engine can NEVER suggest picking from e7 because e7 is empty!
    EXPECT_NE(cand.from_sq(), "e7");
  }
}

} // namespace chess_episode_recorder::domain::test


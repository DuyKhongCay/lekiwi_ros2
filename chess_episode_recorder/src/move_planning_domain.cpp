// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "chess_episode_recorder/move_planning_domain.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <sstream>

namespace chess_episode_recorder::domain
{

// ============================================================================
// MoveCandidate Implementation
// ============================================================================

std::string MoveCandidate::from_sq() const
{
  if (!uci.empty() && uci.size() >= 2) {
    return uci.substr(0, 2);
  }
  return chess::uci::moveToUci(move).substr(0, 2);
}

std::string MoveCandidate::to_sq() const
{
  if (!uci.empty() && uci.size() >= 4) {
    return uci.substr(2, 2);
  }
  return chess::uci::moveToUci(move).substr(2, 2);
}

PieceKey MoveCandidate::moving_piece(const chess::Board &board) const
{
  chess::Square from = move.from();
  chess::Piece p = board.at(from);
  if (p != chess::Piece::NONE) {
    return PieceKey::from_chess_piece(p);
  }
  return PieceKey{board.sideToMove(), chess::PieceType::PAWN};
}

std::optional<PieceKey> MoveCandidate::captured_piece(const chess::Board &board) const
{
  chess::Square to = move.to();
  if (is_en_passant()) {
    chess::Color opp_color = ~board.sideToMove();
    return PieceKey{opp_color, chess::PieceType::PAWN};
  }
  chess::Piece p = board.at(to);
  if (p != chess::Piece::NONE) {
    return PieceKey::from_chess_piece(p);
  }
  return std::nullopt;
}

bool MoveCandidate::is_capture(const chess::Board &board) const
{
  return is_en_passant() || (board.at(move.to()) != chess::Piece::NONE);
}

bool MoveCandidate::is_castling() const
{
  return move.typeOf() == chess::Move::CASTLING;
}

bool MoveCandidate::is_en_passant() const
{
  return move.typeOf() == chess::Move::ENPASSANT;
}

std::string MoveCandidate::promotion_piece_name() const
{
  if (move.typeOf() == chess::Move::PROMOTION) {
    switch (move.promotionType().internal()) {
      case chess::PieceType::underlying::QUEEN: return "queen";
      case chess::PieceType::underlying::ROOK: return "rook";
      case chess::PieceType::underlying::BISHOP: return "bishop";
      case chess::PieceType::underlying::KNIGHT: return "knight";
      default: return "";
    }
  }
  return "";
}

std::string MoveCandidate::move_signature(const chess::Board &board) const
{
  auto mp = moving_piece(board);
  return mp.to_string() + "_" + from_sq() + "_" + to_sq();
}

uint32_t MoveCandidate::expected_episode_count(const chess::Board &board) const noexcept
{
  uint32_t expected = 0;
  auto is_bypassed = [this](std::string_view step) noexcept {
    return std::find(bypass_steps.begin(), bypass_steps.end(), step) != bypass_steps.end();
  };

  if (is_capture(board)) {
    if (!is_bypassed("CLEAR")) {
      expected++;
    }
  }

  if (force_pick_place) {
    if (!is_bypassed("PICK")) {
      expected++;
    }
    if (!is_bypassed("PLACE")) {
      expected++;
    }
  } else {
    if (!is_bypassed("MOVE")) {
      expected++;
    }
  }

  return expected;
}

std::string MoveCandidate::calculate_target_fen(const chess::Board &board) const
{
  chess::Board next_board = board;
  next_board.makeMove(move);
  return next_board.getFen();
}

// ============================================================================
// FenSynthesizer Implementation (Decomposed for low cognitive complexity)
// ============================================================================

FenSynthesizer::FenSynthesizer(uint64_t seed)
{
  if (seed == 0) {
    rng_.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  } else {
    rng_.seed(seed);
  }
}

std::vector<chess::PieceType> FenSynthesizer::sample_pieces_weighted(
  chess::Color color,
  int count,
  const QuotaTracker &tracker)
{
  std::vector<chess::PieceType> available_pool = {
    chess::PieceType::QUEEN,
    chess::PieceType::ROOK, chess::PieceType::ROOK,
    chess::PieceType::BISHOP, chess::PieceType::BISHOP,
    chess::PieceType::KNIGHT, chess::PieceType::KNIGHT,
    chess::PieceType::PAWN, chess::PieceType::PAWN, chess::PieceType::PAWN, chess::PieceType::PAWN,
    chess::PieceType::PAWN, chess::PieceType::PAWN, chess::PieceType::PAWN, chess::PieceType::PAWN
  };

  std::vector<chess::PieceType> selected;
  for (int i = 0; i < count && !available_pool.empty(); ++i) {
    std::vector<double> weights;
    weights.reserve(available_pool.size());
    for (auto pt : available_pool) {
      double def = tracker.get_deficit(color, pt);
      weights.push_back(std::max(1.0, def + 1.0));
    }

    std::discrete_distribution<size_t> dist(weights.begin(), weights.end());
    size_t chosen_idx = dist(rng_);
    selected.push_back(available_pool[chosen_idx]);
    available_pool.erase(available_pool.begin() + chosen_idx);
  }

  return selected;
}

bool FenSynthesizer::validate_position(const chess::Board &board) const
{
  if (board.pieces(chess::PieceType::KING, chess::Color::WHITE).count() != 1 ||
      board.pieces(chess::PieceType::KING, chess::Color::BLACK).count() != 1) {
    return false;
  }

  auto rank_1_8 = chess::Bitboard(chess::Rank::RANK_1) | chess::Bitboard(chess::Rank::RANK_8);
  if ((board.pieces(chess::PieceType::PAWN) & rank_1_8) != 0ull) {
    return false;
  }

  chess::Color inactive_color = ~board.sideToMove();
  chess::Square inactive_king = board.kingSq(inactive_color);
  if (board.isAttacked(inactive_king, board.sideToMove())) {
    return false;
  }

  chess::Movelist legal_moves;
  chess::movegen::legalmoves(legal_moves, board);
  return legal_moves.size() >= 8;
}

std::optional<chess::Square> FenSynthesizer::find_empty_square(
  const std::vector<bool> &occupied,
  const std::vector<chess::Square> &preferred,
  int min_rank,
  int max_rank,
  std::mt19937_64 &rng)
{
  std::vector<chess::Square> candidates;
  for (auto sq : preferred) {
    int idx = sq.index();
    if (!occupied[idx]) {
      int r = idx / 8;
      if (r >= min_rank && r <= max_rank) {
        candidates.push_back(sq);
      }
    }
  }

  if (!candidates.empty()) {
    std::uniform_int_distribution<size_t> d(0, candidates.size() - 1);
    return candidates[d(rng)];
  }

  std::vector<chess::Square> all_valid;
  for (int i = 0; i < 64; ++i) {
    if (!occupied[i]) {
      int r = i / 8;
      if (r >= min_rank && r <= max_rank) {
        all_valid.emplace_back(i);
      }
    }
  }

  if (all_valid.empty()) return std::nullopt;
  std::uniform_int_distribution<size_t> d(0, all_valid.size() - 1);
  return all_valid[d(rng)];
}

std::optional<std::pair<chess::Square, chess::Square>> FenSynthesizer::place_both_kings(
  std::vector<bool> &occupied)
{
  const std::vector<chess::Square> white_preferred = {
    chess::Square(chess::Square::underlying::SQ_E1),
    chess::Square(chess::Square::underlying::SQ_G1),
    chess::Square(chess::Square::underlying::SQ_C1)
  };
  const std::vector<chess::Square> black_preferred = {
    chess::Square(chess::Square::underlying::SQ_E8),
    chess::Square(chess::Square::underlying::SQ_G8),
    chess::Square(chess::Square::underlying::SQ_C8)
  };

  auto wk = find_empty_square(occupied, white_preferred, 0, 2, rng_);
  if (!wk) return std::nullopt;
  occupied[wk->index()] = true;

  auto bk = find_empty_square(occupied, black_preferred, 5, 7, rng_);
  if (!bk) return std::nullopt;
  occupied[bk->index()] = true;

  return std::make_pair(*wk, *bk);
}

bool FenSynthesizer::place_side_army(
  chess::Color color,
  const std::vector<chess::PieceType> &pieces,
  std::vector<bool> &occupied,
  std::vector<std::pair<PieceKey, chess::Square>> &placements)
{
  const std::vector<chess::Square> center_squares = {
    chess::Square(chess::Square::underlying::SQ_C3), chess::Square(chess::Square::underlying::SQ_C4),
    chess::Square(chess::Square::underlying::SQ_C5), chess::Square(chess::Square::underlying::SQ_C6),
    chess::Square(chess::Square::underlying::SQ_D3), chess::Square(chess::Square::underlying::SQ_D4),
    chess::Square(chess::Square::underlying::SQ_D5), chess::Square(chess::Square::underlying::SQ_D6),
    chess::Square(chess::Square::underlying::SQ_E3), chess::Square(chess::Square::underlying::SQ_E4),
    chess::Square(chess::Square::underlying::SQ_E5), chess::Square(chess::Square::underlying::SQ_E6),
    chess::Square(chess::Square::underlying::SQ_F3), chess::Square(chess::Square::underlying::SQ_F4),
    chess::Square(chess::Square::underlying::SQ_F5), chess::Square(chess::Square::underlying::SQ_F6)
  };

  for (auto pt : pieces) {
    const auto &preferred = (pt == chess::PieceType::KNIGHT) ? center_squares : std::vector<chess::Square>{};
    auto sq = find_empty_square(occupied, preferred, 1, 6, rng_);
    if (!sq) {
      return false;
    }
    occupied[sq->index()] = true;
    placements.emplace_back(PieceKey{color, pt}, *sq);
  }
  return true;
}

std::string FenSynthesizer::encode_grid_to_fen(
  const std::vector<std::pair<PieceKey, chess::Square>> &placements,
  const std::string &active_side)
{
  char board_matrix[8][8];
  for (int r = 0; r < 8; ++r) {
    for (int f = 0; f < 8; ++f) {
      board_matrix[r][f] = '.';
    }
  }

  for (const auto &[piece, sq] : placements) {
    int r = sq.index() / 8;
    int f = sq.index() % 8;
    char ch = 'P';
    switch (piece.type.internal()) {
      case chess::PieceType::underlying::PAWN: ch = 'p'; break;
      case chess::PieceType::underlying::KNIGHT: ch = 'n'; break;
      case chess::PieceType::underlying::BISHOP: ch = 'b'; break;
      case chess::PieceType::underlying::ROOK: ch = 'r'; break;
      case chess::PieceType::underlying::QUEEN: ch = 'q'; break;
      case chess::PieceType::underlying::KING: ch = 'k'; break;
      default: ch = 'p'; break;
    }
    if (piece.color == chess::Color::WHITE) {
      ch = static_cast<char>(std::toupper(ch));
    }
    board_matrix[r][f] = ch;
  }

  std::ostringstream fen_stream;
  for (int r = 7; r >= 0; --r) {
    int empty_count = 0;
    for (int f = 0; f < 8; ++f) {
      char ch = board_matrix[r][f];
      if (ch == '.') {
        empty_count++;
      } else {
        if (empty_count > 0) {
          fen_stream << empty_count;
          empty_count = 0;
        }
        fen_stream << ch;
      }
    }
    if (empty_count > 0) {
      fen_stream << empty_count;
    }
    if (r > 0) fen_stream << '/';
  }

  fen_stream << " " << active_side << " - - 0 1";
  return fen_stream.str();
}

std::string FenSynthesizer::synthesize_fen(const QuotaTracker &tracker, int max_attempts)
{
  std::uniform_int_distribution<int> piece_count_dist(8, 12);

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    int total_pieces = piece_count_dist(rng_);
    int white_count = total_pieces / 2;
    int black_count = total_pieces - white_count;

    auto white_pieces = sample_pieces_weighted(chess::Color::WHITE, white_count - 1, tracker);
    auto black_pieces = sample_pieces_weighted(chess::Color::BLACK, black_count - 1, tracker);

    std::vector<bool> occupied(64, false);
    auto kings = place_both_kings(occupied);
    if (!kings) continue;

    std::vector<std::pair<PieceKey, chess::Square>> placements;
    placements.emplace_back(PieceKey{chess::Color::WHITE, chess::PieceType::KING}, kings->first);
    placements.emplace_back(PieceKey{chess::Color::BLACK, chess::PieceType::KING}, kings->second);

    if (!place_side_army(chess::Color::WHITE, white_pieces, occupied, placements)) continue;
    if (!place_side_army(chess::Color::BLACK, black_pieces, occupied, placements)) continue;

    double white_def = 0.0;
    double black_def = 0.0;
    for (auto pt : {chess::PieceType::PAWN, chess::PieceType::KNIGHT, chess::PieceType::BISHOP,
                    chess::PieceType::ROOK, chess::PieceType::QUEEN, chess::PieceType::KING}) {
      white_def += tracker.get_deficit(chess::Color::WHITE, pt);
      black_def += tracker.get_deficit(chess::Color::BLACK, pt);
    }
    std::string active_side = (white_def >= black_def) ? "w" : "b";

    std::string candidate_fen = encode_grid_to_fen(placements, active_side);

    chess::Board test_board;
    if (test_board.setFen(candidate_fen) && validate_position(test_board)) {
      return candidate_fen;
    }
  }

  return "r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/2N2N2/PPPP1PPP/R1BQK2R w KQkq - 4 4";
}

// ============================================================================
// MoveCandidateSelector Implementation
// ============================================================================

MoveCandidate MoveCandidateSelector::classify_chess_move(
  const chess::Board &board,
  const chess::Move &move)
{
  (void)board;
  MoveCandidate candidate;
  candidate.move = move;
  candidate.uci = chess::uci::moveToUci(move);
  candidate.promotion_piece = candidate.promotion_piece_name();

  if (candidate.is_castling()) {
    std::string u = candidate.uci;
    if (u == "e1g1") {
      candidate.castling_rook_from_sq = "h1";
      candidate.castling_rook_to_sq = "f1";
    } else if (u == "e1c1") {
      candidate.castling_rook_from_sq = "a1";
      candidate.castling_rook_to_sq = "d1";
    } else if (u == "e8g8") {
      candidate.castling_rook_from_sq = "h8";
      candidate.castling_rook_to_sq = "f8";
    } else if (u == "e8c8") {
      candidate.castling_rook_from_sq = "a8";
      candidate.castling_rook_to_sq = "d8";
    }
  }

  return candidate;
}

void MoveCandidateSelector::apply_downgrade_and_bypass_policy(
  MoveCandidate &candidate,
  const chess::Board &board,
  const QuotaTracker &tracker)
{
  auto moving_p = candidate.moving_piece(board);

  bool is_single_base = (candidate.plan_type == 0 || candidate.plan_type == 1 ||
                         candidate.plan_type == 3 || candidate.plan_type == 4);
  if (is_single_base) {
    uint32_t rem_move = tracker.get_remaining_quota(StepKind::MOVE, moving_p);
    uint32_t rem_pick = tracker.get_remaining_quota(StepKind::PICK, moving_p);
    uint32_t rem_place = tracker.get_remaining_quota(StepKind::PLACE, moving_p);
    candidate.force_pick_place = (rem_move == 0 && (rem_pick > 0 || rem_place > 0));
  } else {
    candidate.force_pick_place = true;
  }

  candidate.bypass_steps.clear();

  if (candidate.is_capture(board)) {
    candidate.clear_bin_side = tracker.get_next_clear_bin_side();
    auto cap = candidate.captured_piece(board);
    if (cap && tracker.get_remaining_quota(StepKind::CLEAR, *cap) == 0) {
      candidate.bypass_steps.push_back("CLEAR");
    }
  }

  if (candidate.force_pick_place) {
    if (tracker.get_remaining_quota(StepKind::PICK, moving_p) == 0) {
      candidate.bypass_steps.push_back("PICK");
    }
    if (tracker.get_remaining_quota(StepKind::PLACE, moving_p) == 0) {
      candidate.bypass_steps.push_back("PLACE");
    }
  } else if (candidate.is_castling()) {
    PieceKey king_pk{board.sideToMove(), chess::PieceType::KING};
    PieceKey rook_pk{board.sideToMove(), chess::PieceType::ROOK};
    if (tracker.get_remaining_quota(StepKind::MOVE, king_pk) == 0) {
      candidate.bypass_steps.push_back("MOVE");
    }
    if (tracker.get_remaining_quota(StepKind::MOVE, rook_pk) == 0) {
      candidate.bypass_steps.push_back("MOVE");
    }
  } else {
    if (tracker.get_remaining_quota(StepKind::MOVE, moving_p) == 0) {
      candidate.bypass_steps.push_back("MOVE");
    }
  }
}

double MoveCandidateSelector::score_candidate(
  const MoveCandidate &candidate,
  const chess::Board &board,
  const QuotaTracker &tracker,
  size_t next_legal_moves_count) noexcept
{
  auto moving_piece = candidate.moving_piece(board);
  double total_score = 0.0;
  size_t active_steps_count = 0;

  auto evaluate_step = [&](StepKind kind, const PieceKey &piece) {
    std::string kind_str = to_string(kind);
    bool is_bypassed = std::find(candidate.bypass_steps.begin(), candidate.bypass_steps.end(), kind_str)
                       != candidate.bypass_steps.end();
    if (!is_bypassed) {
      double rem = static_cast<double>(tracker.get_remaining_quota(kind, piece));
      double tgt = static_cast<double>(tracker.get_target(kind, piece));
      if (tgt > 0.0) {
        total_score += 100.0 * (rem / tgt);
      }
      active_steps_count++;
    }
  };

  if (candidate.is_capture(board)) {
    auto cap = candidate.captured_piece(board);
    if (cap) {
      evaluate_step(StepKind::CLEAR, *cap);
    }
  }

  if (candidate.force_pick_place) {
    evaluate_step(StepKind::PICK, moving_piece);
    evaluate_step(StepKind::PLACE, moving_piece);
  } else if (candidate.is_castling()) {
    evaluate_step(StepKind::MOVE, PieceKey{board.sideToMove(), chess::PieceType::KING});
    evaluate_step(StepKind::MOVE, PieceKey{board.sideToMove(), chess::PieceType::ROOK});
  } else {
    evaluate_step(StepKind::MOVE, moving_piece);
  }

  if (active_steps_count == 0) {
    return 0.0;
  }

  total_score += 0.5 * static_cast<double>(next_legal_moves_count);
  total_score -= 30.0 * static_cast<double>(candidate.bypass_steps.size());

  return std::max(0.0, total_score);
}

std::vector<MoveCandidate> MoveCandidateSelector::evaluate_candidates(
  const chess::Board &board,
  const QuotaTracker &tracker,
  const FeasibilityChecker &feasibility_checker)
{
  std::vector<MoveCandidate> candidates;

  chess::Movelist legal_moves;
  chess::movegen::legalmoves(legal_moves, board);

  for (const auto &move : legal_moves) {
    MoveCandidate candidate = classify_chess_move(board, move);

    if (tracker.has_executed_move(candidate.uci)) {
      continue;
    }

    if (feasibility_checker) {
      auto [feasible, plan_type] = feasibility_checker(candidate);
      candidate.is_feasible = feasible;
      candidate.plan_type = plan_type;
      if (!feasible) {
        continue;
      }
    } else {
      candidate.is_feasible = true;
      candidate.plan_type = 0;
    }

    apply_downgrade_and_bypass_policy(candidate, board, tracker);

    chess::Board next_board = board;
    next_board.makeMove(move);
    chess::Movelist next_legal_moves;
    chess::movegen::legalmoves(next_legal_moves, next_board);

    candidate.score = score_candidate(candidate, board, tracker, next_legal_moves.size());

    if (candidate.score > 0.0) {
      candidates.push_back(candidate);
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const MoveCandidate &a, const MoveCandidate &b) {
    return a.score > b.score;
  });

  return candidates;
}

std::optional<MoveCandidate> MoveCandidateSelector::select_best_move(
  const chess::Board &board,
  const QuotaTracker &tracker,
  const FeasibilityChecker &feasibility_checker)
{
  auto candidates = evaluate_candidates(board, tracker, feasibility_checker);
  if (candidates.empty()) {
    return std::nullopt;
  }
  return candidates.front();
}

} // namespace chess_episode_recorder::domain


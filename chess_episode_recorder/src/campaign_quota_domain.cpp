// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "chess_episode_recorder/campaign_quota_domain.hpp"

#include <algorithm>
#include <array>
#include <regex>
#include <sstream>

namespace chess_episode_recorder::domain
{

// ============================================================================
// StepKind Helpers
// ============================================================================

std::string to_string(StepKind kind)
{
  switch (kind) {
    case StepKind::MOVE: return "MOVE";
    case StepKind::CLEAR: return "CLEAR";
    case StepKind::PICK: return "PICK";
    case StepKind::PLACE: return "PLACE";
  }
  return "UNKNOWN";
}

std::optional<StepKind> step_kind_from_string(std::string_view str)
{
  if (str == "MOVE") return StepKind::MOVE;
  if (str == "CLEAR") return StepKind::CLEAR;
  if (str == "PICK") return StepKind::PICK;
  if (str == "PLACE") return StepKind::PLACE;
  return std::nullopt;
}

// ============================================================================
// PieceKey Helpers
// ============================================================================

std::string PieceKey::to_string() const
{
  std::string s;
  s += (color == chess::Color::WHITE ? "w_" : "b_");
  switch (type.internal()) {
    case chess::PieceType::underlying::PAWN: s += "P"; break;
    case chess::PieceType::underlying::KNIGHT: s += "N"; break;
    case chess::PieceType::underlying::BISHOP: s += "B"; break;
    case chess::PieceType::underlying::ROOK: s += "R"; break;
    case chess::PieceType::underlying::QUEEN: s += "Q"; break;
    case chess::PieceType::underlying::KING: s += "K"; break;
    default: s += "?"; break;
  }
  return s;
}

std::string PieceKey::color_name() const
{
  return (color == chess::Color::WHITE) ? "white" : "black";
}

std::string PieceKey::piece_name() const
{
  switch (type.internal()) {
    case chess::PieceType::underlying::PAWN: return "pawn";
    case chess::PieceType::underlying::KNIGHT: return "knight";
    case chess::PieceType::underlying::BISHOP: return "bishop";
    case chess::PieceType::underlying::ROOK: return "rook";
    case chess::PieceType::underlying::QUEEN: return "queen";
    case chess::PieceType::underlying::KING: return "king";
    default: return "piece";
  }
}

std::optional<PieceKey> PieceKey::from_string(std::string_view str)
{
  if (str.size() < 3 || str[1] != '_') {
    return std::nullopt;
  }
  chess::Color c = (str[0] == 'w' || str[0] == 'W') ? chess::Color::WHITE : chess::Color::BLACK;
  chess::PieceType pt;
  switch (str[2]) {
    case 'p': case 'P': pt = chess::PieceType::PAWN; break;
    case 'n': case 'N': pt = chess::PieceType::KNIGHT; break;
    case 'b': case 'B': pt = chess::PieceType::BISHOP; break;
    case 'r': case 'R': pt = chess::PieceType::ROOK; break;
    case 'q': case 'Q': pt = chess::PieceType::QUEEN; break;
    case 'k': case 'K': pt = chess::PieceType::KING; break;
    default: return std::nullopt;
  }
  return PieceKey{c, pt};
}

std::optional<PieceKey> PieceKey::from_names(std::string_view color_name, std::string_view piece_name)
{
  chess::Color c;
  if (color_name == "white" || color_name == "w") {
    c = chess::Color::WHITE;
  } else if (color_name == "black" || color_name == "b") {
    c = chess::Color::BLACK;
  } else {
    return std::nullopt;
  }

  chess::PieceType pt;
  if (piece_name == "pawn" || piece_name == "P") pt = chess::PieceType::PAWN;
  else if (piece_name == "knight" || piece_name == "N") pt = chess::PieceType::KNIGHT;
  else if (piece_name == "bishop" || piece_name == "B") pt = chess::PieceType::BISHOP;
  else if (piece_name == "rook" || piece_name == "R") pt = chess::PieceType::ROOK;
  else if (piece_name == "queen" || piece_name == "Q") pt = chess::PieceType::QUEEN;
  else if (piece_name == "king" || piece_name == "K") pt = chess::PieceType::KING;
  else return std::nullopt;

  return PieceKey{c, pt};
}

PieceKey PieceKey::from_chess_piece(chess::Piece piece)
{
  return PieceKey{piece.color(), piece.type()};
}

// ============================================================================
// InstructionParser Implementation
// ============================================================================

std::optional<ParsedInstruction> InstructionParser::parse(std::string_view instruction)
{
  static const std::regex move_regex(
    R"(^Move (white|black) (pawn|knight|bishop|rook|queen|king)( to target| from [a-h][1-8] to [a-h][1-8])?$)",
    std::regex_constants::icase);
  static const std::regex clear_regex(
    R"(^Clear (white|black) (pawn|knight|bishop|rook|queen)( from [a-h][1-8])? to (left|right) bin$)",
    std::regex_constants::icase);
  static const std::regex pick_regex(
    R"(^Pick (white|black) (pawn|knight|bishop|rook|queen|king)( at [a-h][1-8])?$)",
    std::regex_constants::icase);
  static const std::regex place_regex(
    R"(^Place (white|black) (pawn|knight|bishop|rook|queen|king)( on [a-h][1-8])?$)",
    std::regex_constants::icase);

  std::string inst_str(instruction);
  std::smatch match;

  if (std::regex_match(inst_str, match, move_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      return ParsedInstruction{StepKind::MOVE, *pk, ""};
    }
  } else if (std::regex_match(inst_str, match, clear_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      // Clear bin side is in match[4] if match[3] was the optional from square, or match[3] if no from square
      std::string side = match[match.size() - 1].str();
      return ParsedInstruction{StepKind::CLEAR, *pk, side};
    }
  } else if (std::regex_match(inst_str, match, pick_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      return ParsedInstruction{StepKind::PICK, *pk, ""};
    }
  } else if (std::regex_match(inst_str, match, place_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      return ParsedInstruction{StepKind::PLACE, *pk, ""};
    }
  }

  return std::nullopt;
}

// ============================================================================
// QuotaTracker Implementation
// ============================================================================

QuotaTracker::QuotaTracker(const CampaignConfig &config)
{
  configure(config);
}

void QuotaTracker::configure(const CampaignConfig &config)
{
  config_ = config;
  if (config_.move_total == 0 && config_.clear_total == 0 &&
      config_.pick_total == 0 && config_.place_total == 0 && config_.total_episodes > 0)
  {
    config_.move_total = static_cast<uint32_t>(config_.total_episodes * 0.40);
    config_.clear_total = static_cast<uint32_t>(config_.total_episodes * 0.20);
    config_.pick_total = static_cast<uint32_t>(config_.total_episodes * 0.20);
    config_.place_total = config_.total_episodes - config_.move_total - config_.clear_total - config_.pick_total;
  }
  total_target_ = config_.total_episodes;
  episodes_per_session_ = config_.episodes_per_session;
  init_default_quotas();
}

void QuotaTracker::init_default_quotas()
{
  const auto &config = config_;
  targets_.clear();
  completed_.clear();
  repeat_counts_.clear();

  clear_left_target_ = config.clear_total / 2;
  clear_right_target_ = config.clear_total - clear_left_target_;
  clear_left_completed_ = 0;
  clear_right_completed_ = 0;

  const std::array<chess::Color, 2> colors = {chess::Color::WHITE, chess::Color::BLACK};
  const std::array<chess::PieceType, 6> all_pieces = {
    chess::PieceType::PAWN, chess::PieceType::KNIGHT, chess::PieceType::BISHOP,
    chess::PieceType::ROOK, chess::PieceType::QUEEN, chess::PieceType::KING
  };
  const std::array<chess::PieceType, 5> clearable_pieces = {
    chess::PieceType::PAWN, chess::PieceType::KNIGHT, chess::PieceType::BISHOP,
    chess::PieceType::ROOK, chess::PieceType::QUEEN
  };

  // 1. MOVE: 12 piece classes
  uint32_t move_per_piece = config.move_total / 12;
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      targets_[StepKind::MOVE][PieceKey{c, pt}] = move_per_piece;
      completed_[StepKind::MOVE][PieceKey{c, pt}] = 0;
    }
  }

  // 2. CLEAR: 10 piece classes (King cannot be cleared/captured)
  uint32_t clear_per_piece = config.clear_total / 10;
  for (auto c : colors) {
    for (auto pt : clearable_pieces) {
      targets_[StepKind::CLEAR][PieceKey{c, pt}] = clear_per_piece;
      completed_[StepKind::CLEAR][PieceKey{c, pt}] = 0;
    }
  }

  // 3. PICK: 12 piece classes
  uint32_t pick_per_piece = config.pick_total / 12;
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      targets_[StepKind::PICK][PieceKey{c, pt}] = pick_per_piece;
      completed_[StepKind::PICK][PieceKey{c, pt}] = 0;
    }
  }

  // 4. PLACE: 12 piece classes
  uint32_t place_per_piece = config.place_total / 12;
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      targets_[StepKind::PLACE][PieceKey{c, pt}] = place_per_piece;
      completed_[StepKind::PLACE][PieceKey{c, pt}] = 0;
    }
  }
}

bool QuotaTracker::record_completed_instruction(const std::string &instruction)
{
  auto parsed = InstructionParser::parse(instruction);
  if (!parsed) {
    return false;
  }

  completed_[parsed->kind][parsed->piece]++;

  if (parsed->kind == StepKind::CLEAR) {
    if (parsed->bin_side == "left") {
      clear_left_completed_++;
    } else {
      clear_right_completed_++;
    }
    repeat_counts_[parsed->piece.to_string() + "_clear_" + parsed->bin_side]++;
  } else if (parsed->kind == StepKind::PICK) {
    repeat_counts_[parsed->piece.to_string() + "_pick"]++;
  } else if (parsed->kind == StepKind::PLACE) {
    repeat_counts_[parsed->piece.to_string() + "_place"]++;
  } else {
    repeat_counts_[parsed->piece.to_string()]++;
  }

  return true;
}

bool QuotaTracker::record_episode(const std::string &task_instruction)
{
  return record_completed_instruction(task_instruction);
}

void QuotaTracker::restore_session_state(const SessionState &state)
{
  session_state_ = state;
  reconstruct_session_board();
}

bool QuotaTracker::reconstruct_session_board()
{
  if (session_state_.initial_fen.empty()) {
    session_state_.is_active = false;
    active_board_ = chess::Board();
    return false;
  }

  chess::Board temp_board;
  if (!temp_board.setFen(session_state_.initial_fen)) {
    session_state_.is_active = false;
    return false;
  }

  for (const auto &uci : session_state_.executed_moves) {
    chess::Move m = chess::uci::uciToMove(temp_board, uci);
    if (m == chess::Move::NO_MOVE) {
      session_state_.is_active = false;
      return false;
    }
    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, temp_board);
    if (std::find(legal_moves.begin(), legal_moves.end(), m) == legal_moves.end()) {
      session_state_.is_active = false;
      return false;
    }
    temp_board.makeMove(m);
  }

  active_board_ = std::move(temp_board);
  session_state_.active_fen = active_board_.getFen();
  session_state_.is_active = true;
  return true;
}

void QuotaTracker::set_active_fen(const std::string &fen)
{
  session_state_.active_fen = fen;
  session_state_.is_active = !fen.empty();
  if (session_state_.initial_fen.empty()) {
    session_state_.initial_fen = fen;
  }
  active_board_.setFen(fen);
}

bool QuotaTracker::apply_move(const std::string &uci)
{
  chess::Move m = chess::uci::uciToMove(active_board_, uci);
  if (m == chess::Move::NO_MOVE) {
    return false;
  }
  chess::Movelist legal_moves;
  chess::movegen::legalmoves(legal_moves, active_board_);
  if (std::find(legal_moves.begin(), legal_moves.end(), m) == legal_moves.end()) {
    return false;
  }
  active_board_.makeMove(m);
  session_state_.active_fen = active_board_.getFen();
  session_state_.is_active = true;
  record_executed_move(uci);
  return true;
}

BoardTransitionOutcome QuotaTracker::verify_and_commit_move(
  const std::string &expected_uci,
  const std::string &actual_fen)
{
  auto extract_placement = [](std::string_view fen) -> std::string_view {
    auto pos = fen.find(' ');
    return (pos != std::string_view::npos) ? fen.substr(0, pos) : fen;
  };

  if (!actual_fen.empty()) {
    chess::Board expected_board = active_board_;
    chess::Move m = chess::uci::uciToMove(expected_board, expected_uci);
    bool expected_applied = false;
    if (m != chess::Move::NO_MOVE) {
      chess::Movelist legal_moves;
      chess::movegen::legalmoves(legal_moves, expected_board);
      if (std::find(legal_moves.begin(), legal_moves.end(), m) != legal_moves.end()) {
        expected_board.makeMove(m);
        expected_applied = true;
      }
    }

    if (expected_applied && extract_placement(expected_board.getFen()) == extract_placement(actual_fen)) {
      active_board_ = expected_board;
      session_state_.active_fen = actual_fen;
      session_state_.is_active = true;
      record_executed_move(expected_uci);
      return BoardTransitionOutcome::MATCHED_EXPECTED_MOVE;
    }

    // Divergence: adopt actual_fen
    chess::Board actual_board;
    if (actual_board.setFen(actual_fen)) {
      active_board_ = actual_board;
      session_state_.active_fen = actual_fen;
      session_state_.is_active = true;
      session_state_.executed_moves.clear();
      return BoardTransitionOutcome::DIVERGED_RESYNCED;
    }

    return BoardTransitionOutcome::INVALID_ACTUAL_FEN;
  }

  // Fallback when actual_fen is empty
  if (apply_move(expected_uci)) {
    return BoardTransitionOutcome::MATCHED_EXPECTED_MOVE;
  }
  return BoardTransitionOutcome::INVALID_ACTUAL_FEN;
}

uint32_t QuotaTracker::get_completed(StepKind kind, const PieceKey &piece) const
{
  auto kind_it = completed_.find(kind);
  if (kind_it == completed_.end()) return 0;
  auto piece_it = kind_it->second.find(piece);
  if (piece_it == kind_it->second.end()) return 0;
  return piece_it->second;
}

uint32_t QuotaTracker::get_target(StepKind kind, const PieceKey &piece) const
{
  auto kind_it = targets_.find(kind);
  if (kind_it == targets_.end()) return 0;
  auto piece_it = kind_it->second.find(piece);
  if (piece_it == kind_it->second.end()) return 0;
  return piece_it->second;
}

uint32_t QuotaTracker::get_remaining_quota(StepKind kind, const PieceKey &piece) const
{
  uint32_t t = get_target(kind, piece);
  uint32_t c = get_completed(kind, piece);
  return (c >= t) ? 0 : (t - c);
}

double QuotaTracker::get_deficit(chess::Color color, chess::PieceType piece_type) const
{
  PieceKey pk{color, piece_type};
  double rem_move = static_cast<double>(get_remaining_quota(StepKind::MOVE, pk));
  double rem_pick = static_cast<double>(get_remaining_quota(StepKind::PICK, pk));
  double rem_place = static_cast<double>(get_remaining_quota(StepKind::PLACE, pk));
  double rem_clear = (piece_type == chess::PieceType::KING) ? 0.0 :
    static_cast<double>(get_remaining_quota(StepKind::CLEAR, pk));

  return rem_move + rem_pick + rem_place + 1.5 * rem_clear;
}

uint32_t QuotaTracker::get_repeat_count(const std::string &move_signature) const
{
  auto it = repeat_counts_.find(move_signature);
  return (it != repeat_counts_.end()) ? it->second : 0;
}

bool QuotaTracker::is_campaign_complete() const
{
  for (const auto &[kind, target_map] : targets_) {
    for (const auto &[piece, tgt] : target_map) {
      if (get_completed(kind, piece) < tgt) {
        return false;
      }
    }
  }
  return true;
}

bool QuotaTracker::is_step_kind_quota_reached(StepKind kind) const noexcept
{
  auto it_comp = completed_.find(kind);
  auto it_tgt = targets_.find(kind);
  if (it_comp == completed_.end() || it_tgt == targets_.end()) {
    return true;
  }
  uint32_t comp = 0;
  for (const auto &[piece, cnt] : it_comp->second) {
    comp += cnt;
  }
  uint32_t tgt = 0;
  for (const auto &[piece, cnt] : it_tgt->second) {
    tgt += cnt;
  }
  return comp >= tgt;
}

std::string QuotaTracker::get_next_clear_bin_side() const
{
  if (clear_left_completed_ < clear_left_target_ && clear_right_completed_ < clear_right_target_) {
    return (clear_left_completed_ <= clear_right_completed_) ? "left" : "right";
  }
  if (clear_left_completed_ < clear_left_target_) {
    return "left";
  }
  return "right";
}

QuotaSummary QuotaTracker::get_summary() const
{
  QuotaSummary summary;
  summary.total_target = total_target_;
  summary.clear_left_completed = clear_left_completed_;
  summary.clear_right_completed = clear_right_completed_;

  uint32_t total_done = 0;
  for (const auto &[kind, target_map] : targets_) {
    uint32_t kind_target = 0;
    uint32_t kind_done = 0;
    for (const auto &[piece, tgt] : target_map) {
      uint32_t done = get_completed(kind, piece);
      summary.completed_by_piece[kind][piece] = done;
      summary.target_by_piece[kind][piece] = tgt;
      kind_target += tgt;
      kind_done += done;
    }
    summary.target_by_kind[kind] = kind_target;
    summary.completed_by_kind[kind] = kind_done;
    total_done += kind_done;
  }
  summary.total_completed = total_done;
  return summary;
}

void QuotaTracker::record_executed_move(const std::string &uci)
{
  if (!uci.empty() && !has_executed_move(uci)) {
    session_state_.executed_moves.push_back(uci);
  }
}

bool QuotaTracker::has_executed_move(const std::string &uci) const
{
  return std::find(session_state_.executed_moves.begin(),
                   session_state_.executed_moves.end(),
                   uci) != session_state_.executed_moves.end();
}

void QuotaTracker::set_session_state(const SessionState &state)
{
  session_state_ = state;
  reconstruct_session_board();
}

void QuotaTracker::advance_session()
{
  session_state_.session_id++;
  session_state_.episodes_in_session = 0;
  session_state_.executed_moves.clear();
  session_state_.initial_fen.clear();
  session_state_.active_fen.clear();
  session_state_.is_active = false;
  active_board_ = chess::Board();
}

void QuotaTracker::reset_session(const std::string &new_fen)
{
  session_state_.episodes_in_session = 0;
  session_state_.executed_moves.clear();
  session_state_.initial_fen = new_fen;
  session_state_.active_fen = new_fen;
  session_state_.is_active = !new_fen.empty();
  active_board_.setFen(new_fen);
}

} // namespace chess_episode_recorder::domain


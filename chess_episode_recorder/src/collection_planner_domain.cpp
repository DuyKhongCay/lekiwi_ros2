// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "chess_episode_recorder/collection_planner_domain.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <set>
#include <sstream>

namespace chess_episode_recorder::domain
{

// ============================================================================
// StepKind & PieceKey Domain Helpers
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
// MoveCandidate Helper Methods
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
  return PieceKey::from_chess_piece(board.at(move.from()));
}

bool MoveCandidate::is_capture(const chess::Board &board) const
{
  return board.isCapture(move);
}

std::optional<PieceKey> MoveCandidate::captured_piece(const chess::Board &board) const
{
  if (!board.isCapture(move)) {
    return std::nullopt;
  }
  if (move.typeOf() == chess::Move::ENPASSANT) {
    return PieceKey{~board.sideToMove(), chess::PieceType::PAWN};
  }
  chess::Piece cap = board.at(move.to());
  return PieceKey::from_chess_piece(cap);
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
  if (move.typeOf() != chess::Move::PROMOTION) {
    return "";
  }
  switch (move.promotionType().internal()) {
    case chess::PieceType::underlying::QUEEN: return "queen";
    case chess::PieceType::underlying::KNIGHT: return "knight";
    case chess::PieceType::underlying::ROOK: return "rook";
    case chess::PieceType::underlying::BISHOP: return "bishop";
    default: return "";
  }
}

std::string MoveCandidate::move_signature(const chess::Board &board) const
{
  auto piece = moving_piece(board);
  return piece.to_string() + "_" + from_sq() + "_" + to_sq();
}

// ============================================================================
// QuotaTracker Implementation
// ============================================================================

QuotaTracker::QuotaTracker()
{
  init_default_quotas();
}

QuotaTracker::QuotaTracker(const std::filesystem::path &config_path)
{
  init_default_quotas();
  load_config_file(config_path);
}

QuotaTracker::QuotaTracker(const YAML::Node &config_node)
{
  init_default_quotas();
  load_config(config_node);
}

void QuotaTracker::init_default_quotas()
{
  total_target_ = 600;
  episodes_per_session_ = 20;

  const std::vector<chess::Color> colors = {chess::Color::WHITE, chess::Color::BLACK};
  const std::vector<chess::PieceType> all_pieces = {
    chess::PieceType::PAWN, chess::PieceType::KNIGHT, chess::PieceType::BISHOP,
    chess::PieceType::ROOK, chess::PieceType::QUEEN, chess::PieceType::KING
  };

  // 1. MOVE: 20 per piece x 12 pieces = 240
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      targets_[StepKind::MOVE][PieceKey{c, pt}] = 20;
      completed_[StepKind::MOVE][PieceKey{c, pt}] = 0;
    }
  }

  // 2. CLEAR: 12 per piece x 10 capturable pieces (no king) = 120
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      if (pt == chess::PieceType::KING) continue;
      targets_[StepKind::CLEAR][PieceKey{c, pt}] = 12;
      completed_[StepKind::CLEAR][PieceKey{c, pt}] = 0;
    }
  }
  clear_left_target_ = 60;
  clear_right_target_ = 60;
  clear_left_completed_ = 0;
  clear_right_completed_ = 0;

  // 3. PICK: 10 per piece x 12 pieces = 120
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      targets_[StepKind::PICK][PieceKey{c, pt}] = 10;
      completed_[StepKind::PICK][PieceKey{c, pt}] = 0;
    }
  }

  // 4. PLACE: 10 per piece x 12 pieces = 120
  for (auto c : colors) {
    for (auto pt : all_pieces) {
      targets_[StepKind::PLACE][PieceKey{c, pt}] = 10;
      completed_[StepKind::PLACE][PieceKey{c, pt}] = 0;
    }
  }
}

void QuotaTracker::load_config(const YAML::Node &config_node)
{
  if (!config_node || !config_node.IsMap()) {
    return;
  }

  if (config_node["campaign"]) {
    auto c = config_node["campaign"];
    if (c["total_episodes"]) total_target_ = c["total_episodes"].as<uint32_t>();
    if (c["episodes_per_session"]) episodes_per_session_ = c["episodes_per_session"].as<uint32_t>();
  }

  if (config_node["quotas"]) {
    auto q = config_node["quotas"];
    for (auto kind : {StepKind::MOVE, StepKind::CLEAR, StepKind::PICK, StepKind::PLACE}) {
      std::string key = (kind == StepKind::MOVE) ? "move" :
                        (kind == StepKind::CLEAR) ? "clear" :
                        (kind == StepKind::PICK) ? "pick" : "place";
      if (q[key]) {
        auto sub = q[key];
        uint32_t per_piece = sub["per_piece"] ? sub["per_piece"].as<uint32_t>() : 0;
        if (sub["pieces"] && sub["pieces"].IsSequence()) {
          for (const auto &item : sub["pieces"]) {
            auto pk = PieceKey::from_string(item.as<std::string>());
            if (pk) {
              targets_[kind][*pk] = per_piece;
            }
          }
        }
        if (kind == StepKind::CLEAR && sub["bins"]) {
          if (sub["bins"]["left"]) clear_left_target_ = sub["bins"]["left"].as<uint32_t>();
          if (sub["bins"]["right"]) clear_right_target_ = sub["bins"]["right"].as<uint32_t>();
        }
      }
    }
  }
}

void QuotaTracker::load_config_file(const std::filesystem::path &config_path)
{
  if (!std::filesystem::exists(config_path)) {
    return;
  }
  try {
    YAML::Node config = YAML::LoadFile(config_path.string());
    load_config(config);
  } catch (const std::exception &e) {
    std::cerr << "Warning: Failed to load collection config from " << config_path << ": " << e.what() << "\n";
  }
}

bool QuotaTracker::parse_instruction_and_increment(const std::string &instruction)
{
  static const std::regex move_regex(
    R"(^Move (white|black) (pawn|knight|bishop|rook|queen|king) to target$)");
  static const std::regex clear_regex(
    R"(^Clear (white|black) (pawn|knight|bishop|rook|queen) to (left|right) bin$)");
  static const std::regex pick_regex(
    R"(^Pick (white|black) (pawn|knight|bishop|rook|queen|king)$)");
  static const std::regex place_regex(
    R"(^Place (white|black) (pawn|knight|bishop|rook|queen|king)$)");

  std::smatch match;

  if (std::regex_match(instruction, match, move_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      completed_[StepKind::MOVE][*pk]++;
      repeat_counts_[pk->to_string()]++;
      return true;
    }
  } else if (std::regex_match(instruction, match, clear_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      completed_[StepKind::CLEAR][*pk]++;
      std::string bin_side = match[3].str();
      if (bin_side == "left") {
        clear_left_completed_++;
      } else {
        clear_right_completed_++;
      }
      repeat_counts_[pk->to_string() + "_clear_" + bin_side]++;
      return true;
    }
  } else if (std::regex_match(instruction, match, pick_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      completed_[StepKind::PICK][*pk]++;
      repeat_counts_[pk->to_string() + "_pick"]++;
      return true;
    }
  } else if (std::regex_match(instruction, match, place_regex)) {
    auto pk = PieceKey::from_names(match[1].str(), match[2].str());
    if (pk) {
      completed_[StepKind::PLACE][*pk]++;
      repeat_counts_[pk->to_string() + "_place"]++;
      return true;
    }
  }

  return false;
}

uint32_t QuotaTracker::scan_and_recover(const std::filesystem::path &base_output_dir)
{
  uint32_t recovered_count = 0;
  if (!std::filesystem::exists(base_output_dir)) {
    return 0;
  }

  // 1. Reset counters to zero prior to scanning
  for (auto &[kind, piece_map] : completed_) {
    for (auto &[piece, count] : piece_map) {
      count = 0;
    }
  }
  clear_left_completed_ = 0;
  clear_right_completed_ = 0;
  repeat_counts_.clear();

  // 2. Scan all episode_XXXXXX/episode_info.yaml files
  const std::regex ep_pattern(R"(episode_(\d{6}))");
  std::vector<std::filesystem::path> ep_dirs;
  for (const auto &entry : std::filesystem::directory_iterator(base_output_dir)) {
    if (!entry.is_directory()) continue;
    std::string name = entry.path().filename().string();
    if (std::regex_match(name, ep_pattern)) {
      ep_dirs.push_back(entry.path());
    }
  }
  std::sort(ep_dirs.begin(), ep_dirs.end());

  for (const auto &dir : ep_dirs) {
    auto info_path = dir / "episode_info.yaml";
    if (!std::filesystem::exists(info_path)) continue;

    try {
      YAML::Node info = YAML::LoadFile(info_path.string());
      if (info["task"]) {
        std::string task_str = info["task"].as<std::string>();
        if (parse_instruction_and_increment(task_str)) {
          recovered_count++;
        }
      }
    } catch (...) {
      // Ignore corrupted files during scanning
    }
  }

  // 3. Load session state if available
  auto state_path = base_output_dir / "session_state.yaml";
  if (std::filesystem::exists(state_path)) {
    load_session_state(state_path);
  }

  return recovered_count;
}

bool QuotaTracker::record_episode(const std::string &task_instruction)
{
  return parse_instruction_and_increment(task_instruction);
}

bool QuotaTracker::load_session_state(const std::filesystem::path &file_path)
{
  if (!std::filesystem::exists(file_path)) {
    session_state_ = SessionState{};
    return false;
  }
  try {
    YAML::Node node = YAML::LoadFile(file_path.string());
    if (node["session_id"]) session_state_.session_id = node["session_id"].as<uint32_t>();
    if (node["episodes_in_session"]) session_state_.episodes_in_session = node["episodes_in_session"].as<uint32_t>();
    if (node["active_fen"]) session_state_.active_fen = node["active_fen"].as<std::string>();
    session_state_.is_active = !session_state_.active_fen.empty();

    session_state_.executed_moves.clear();
    if (node["executed_moves"] && node["executed_moves"].IsSequence()) {
      for (const auto &item : node["executed_moves"]) {
        session_state_.executed_moves.push_back(item.as<std::string>());
      }
    }
    return true;
  } catch (...) {
    return false;
  }
}

bool QuotaTracker::save_session_state(const std::filesystem::path &file_path) const
{
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "session_id" << YAML::Value << session_state_.session_id;
    out << YAML::Key << "episodes_in_session" << YAML::Value << session_state_.episodes_in_session;
    out << YAML::Key << "active_fen" << YAML::Value << session_state_.active_fen;

    out << YAML::Key << "executed_moves" << YAML::Value << YAML::BeginSeq;
    for (const auto &mv : session_state_.executed_moves) {
      out << mv;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;

    std::ofstream fout(file_path);
    if (!fout.is_open()) return false;
    fout << out.c_str();
    return true;
  } catch (...) {
    return false;
  }
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

void QuotaTracker::advance_session()
{
  session_state_.session_id++;
  session_state_.episodes_in_session = 0;
  session_state_.active_fen.clear();
  session_state_.is_active = false;
  session_state_.executed_moves.clear();
}

void QuotaTracker::reset_session(const std::string &new_fen)
{
  session_state_.episodes_in_session = 0;
  session_state_.active_fen = new_fen;
  session_state_.is_active = !new_fen.empty();
  session_state_.executed_moves.clear();
}

// ============================================================================
// FenSynthesizer Implementation
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
  // 1. King presence (1 white, 1 black)
  if (board.pieces(chess::PieceType::KING, chess::Color::WHITE).count() != 1 ||
      board.pieces(chess::PieceType::KING, chess::Color::BLACK).count() != 1) {
    return false;
  }

  // 2. Pawns cannot reside on rank 1 or 8
  auto rank_1_8 = chess::Bitboard(chess::Rank::RANK_1) | chess::Bitboard(chess::Rank::RANK_8);
  if ((board.pieces(chess::PieceType::PAWN) & rank_1_8) != 0ull) {
    return false;
  }

  // 3. Opposing (inactive) king cannot be in check
  chess::Color inactive_color = ~board.sideToMove();
  chess::Square inactive_king = board.kingSq(inactive_color);
  if (board.isAttacked(inactive_king, board.sideToMove())) {
    return false;
  }

  // 4. Active side must have at least >= 8 legal moves and not be checkmated
  chess::Movelist legal_moves;
  chess::movegen::legalmoves(legal_moves, board);
  if (legal_moves.size() < 8) {
    return false;
  }

  return true;
}

std::string FenSynthesizer::synthesize_fen(const QuotaTracker &tracker, int max_attempts)
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

  std::uniform_int_distribution<int> piece_count_dist(8, 12);

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    int total_pieces = piece_count_dist(rng_);
    int white_count = total_pieces / 2;
    int black_count = total_pieces - white_count;

    auto white_pieces = sample_pieces_weighted(chess::Color::WHITE, white_count - 1, tracker);
    auto black_pieces = sample_pieces_weighted(chess::Color::BLACK, black_count - 1, tracker);

    std::set<int> occupied_squares;

    auto get_empty_square = [&](const std::vector<chess::Square> &preferred, int min_rank = 0, int max_rank = 7) -> std::optional<chess::Square> {
      std::vector<chess::Square> candidates;
      for (auto sq : preferred) {
        if (occupied_squares.find(sq.index()) == occupied_squares.end()) {
          int r = sq.index() / 8;
          if (r >= min_rank && r <= max_rank) {
            candidates.push_back(sq);
          }
        }
      }
      if (!candidates.empty()) {
        std::uniform_int_distribution<size_t> d(0, candidates.size() - 1);
        return candidates[d(rng_)];
      }
      std::vector<chess::Square> all_valid;
      for (int i = 0; i < 64; ++i) {
        if (occupied_squares.find(i) == occupied_squares.end()) {
          int r = i / 8;
          if (r >= min_rank && r <= max_rank) {
            all_valid.emplace_back(i);
          }
        }
      }
      if (all_valid.empty()) return std::nullopt;
      std::uniform_int_distribution<size_t> d(0, all_valid.size() - 1);
      return all_valid[d(rng_)];
    };

    auto wk_sq = get_empty_square({chess::Square(chess::Square::underlying::SQ_E1),
                                   chess::Square(chess::Square::underlying::SQ_G1),
                                   chess::Square(chess::Square::underlying::SQ_C1)}, 0, 2);
    if (!wk_sq) continue;
    occupied_squares.insert(wk_sq->index());

    auto bk_sq = get_empty_square({chess::Square(chess::Square::underlying::SQ_E8),
                                   chess::Square(chess::Square::underlying::SQ_G8),
                                   chess::Square(chess::Square::underlying::SQ_C8)}, 5, 7);
    if (!bk_sq) continue;
    occupied_squares.insert(bk_sq->index());

    std::vector<std::pair<PieceKey, chess::Square>> placements;
    placements.emplace_back(PieceKey{chess::Color::WHITE, chess::PieceType::KING}, *wk_sq);
    placements.emplace_back(PieceKey{chess::Color::BLACK, chess::PieceType::KING}, *bk_sq);

    bool placement_ok = true;

    auto place_piece = [&](chess::Color c, chess::PieceType pt) {
      std::optional<chess::Square> sq;
      if (pt == chess::PieceType::KNIGHT) {
        sq = get_empty_square(center_squares, 1, 6);
      } else if (pt == chess::PieceType::PAWN) {
        sq = get_empty_square({}, 1, 6);
      } else {
        sq = get_empty_square({}, 1, 6);
      }
      if (!sq) {
        placement_ok = false;
        return;
      }
      occupied_squares.insert(sq->index());
      placements.emplace_back(PieceKey{c, pt}, *sq);
    };

    for (auto pt : white_pieces) place_piece(chess::Color::WHITE, pt);
    for (auto pt : black_pieces) place_piece(chess::Color::BLACK, pt);

    if (!placement_ok) continue;

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

    double white_def = 0.0;
    double black_def = 0.0;
    for (auto pt : {chess::PieceType::PAWN, chess::PieceType::KNIGHT, chess::PieceType::BISHOP,
                    chess::PieceType::ROOK, chess::PieceType::QUEEN, chess::PieceType::KING}) {
      white_def += tracker.get_deficit(chess::Color::WHITE, pt);
      black_def += tracker.get_deficit(chess::Color::BLACK, pt);
    }
    std::string active_side = (white_def >= black_def) ? "w" : "b";

    fen_stream << " " << active_side << " - - 0 1";
    std::string candidate_fen = fen_stream.str();

    chess::Board test_board;
    if (test_board.setFen(candidate_fen)) {
      if (validate_position(test_board)) {
        return candidate_fen;
      }
    }
  }

  return "r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/2N2N2/PPPP1PPP/R1BQK2R w KQkq - 4 4";
}

// ============================================================================
// MoveCandidateSelector Implementation
// ============================================================================

double MoveCandidateSelector::compute_score(
  const MoveCandidate &candidate,
  const chess::Board &board,
  const QuotaTracker &tracker,
  size_t next_legal_moves_count)
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

  // 1. Capture step
  if (candidate.is_capture(board)) {
    auto cap = candidate.captured_piece(board);
    if (cap) {
      evaluate_step(StepKind::CLEAR, *cap);
    }
  }

  // 2. Moving steps
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
    MoveCandidate candidate;
    candidate.move = move;
    candidate.uci = chess::uci::moveToUci(move);

    // Skip moves that have already been executed in the active session
    if (tracker.has_executed_move(candidate.uci)) {
      continue;
    }

    if (candidate.is_capture(board)) {
      candidate.clear_bin_side = tracker.get_next_clear_bin_side();
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

    auto moving_p = candidate.moving_piece(board);

    bool is_single_base = (candidate.plan_type == 0 || candidate.plan_type == 1 ||
                           candidate.plan_type == 3 || candidate.plan_type == 4);
    if (is_single_base) {
      uint32_t rem_move = tracker.get_remaining_quota(StepKind::MOVE, moving_p);
      uint32_t rem_pick = tracker.get_remaining_quota(StepKind::PICK, moving_p);
      uint32_t rem_place = tracker.get_remaining_quota(StepKind::PLACE, moving_p);
      if (rem_move == 0 && (rem_pick > 0 || rem_place > 0)) {
        candidate.force_pick_place = true;
      } else {
        candidate.force_pick_place = false;
      }
    } else {
      candidate.force_pick_place = true;
    }

    candidate.bypass_steps.clear();

    if (candidate.is_capture(board)) {
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

    chess::Board next_board = board;
    next_board.makeMove(move);
    chess::Movelist next_legal_moves;
    chess::movegen::legalmoves(next_legal_moves, next_board);

    candidate.score = compute_score(candidate, board, tracker, next_legal_moves.size());

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

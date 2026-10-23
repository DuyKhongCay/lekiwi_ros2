// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <regex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <chess.hpp>
#include <yaml-cpp/yaml.h>

namespace chess_episode_recorder::domain
{

/**
 * @brief Atomic manipulation step kinds within the SmolVLA data collection framework.
 */
enum class StepKind
{
  MOVE,   // Robot navigates and moves piece from square to square
  CLEAR,  // Robot clears captured opponent piece to designated side bin
  PICK,   // Dedicated pick phase for decoupled or multi-base trajectories
  PLACE   // Dedicated place phase for decoupled or multi-base trajectories
};

std::string to_string(StepKind kind);
std::optional<StepKind> step_kind_from_string(std::string_view str);

/**
 * @brief Domain identifier for a chess piece entity (color + piece type).
 */
struct PieceKey
{
  chess::Color color{chess::Color::WHITE};
  chess::PieceType type{chess::PieceType::PAWN};

  bool operator<(const PieceKey &other) const
  {
    if (color != other.color) {
      return static_cast<int>(color.internal()) < static_cast<int>(other.color.internal());
    }
    return static_cast<int>(type.internal()) < static_cast<int>(other.type.internal());
  }

  bool operator==(const PieceKey &other) const
  {
    return color == other.color && type == other.type;
  }

  bool operator!=(const PieceKey &other) const
  {
    return !(*this == other);
  }

  [[nodiscard]] std::string to_string() const;
  [[nodiscard]] std::string color_name() const;
  [[nodiscard]] std::string piece_name() const;

  static std::optional<PieceKey> from_string(std::string_view str);
  static std::optional<PieceKey> from_names(std::string_view color_name, std::string_view piece_name);
  static PieceKey from_chess_piece(chess::Piece piece);
};

/**
 * @brief Candidate chess move evaluated for collection execution.
 * Minimalist design: square coordinates and captured piece are derived dynamically
 * from the underlying chess::Board and chess::Move.
 */
struct MoveCandidate
{
  chess::Move move;
  std::string uci;

  // Execution directives evaluated by MoveCandidateSelector
  bool force_pick_place{false};
  std::string clear_bin_side{"right"};    // "left" or "right"
  std::vector<std::string> bypass_steps;  // Step names to bypass physically: {"MOVE", "CLEAR", "PICK", "PLACE"}

  // Feasibility assessment attributes
  uint8_t plan_type{0};
  bool is_feasible{false};
  double score{0.0};

  // Convenient helper methods inferring details from board
  [[nodiscard]] std::string from_sq() const;
  [[nodiscard]] std::string to_sq() const;
  [[nodiscard]] PieceKey moving_piece(const chess::Board &board) const;
  [[nodiscard]] std::optional<PieceKey> captured_piece(const chess::Board &board) const;
  [[nodiscard]] bool is_capture(const chess::Board &board) const;
  [[nodiscard]] bool is_castling() const;
  [[nodiscard]] bool is_en_passant() const;
  [[nodiscard]] std::string promotion_piece_name() const;
  [[nodiscard]] std::string move_signature(const chess::Board &board) const;
};

/**
 * @brief High-level summary of collection campaign progress.
 */
struct QuotaSummary
{
  uint32_t total_target{600};
  uint32_t total_completed{0};
  std::map<StepKind, uint32_t> completed_by_kind;
  std::map<StepKind, uint32_t> target_by_kind;
  std::map<StepKind, std::map<PieceKey, uint32_t>> completed_by_piece;
  std::map<StepKind, std::map<PieceKey, uint32_t>> target_by_piece;
  uint32_t clear_left_completed{0};
  uint32_t clear_right_completed{0};
};

/**
 * @brief Persistent session state tracking active session progress and FEN.
 */
struct SessionState
{
  uint32_t session_id{1};
  uint32_t episodes_in_session{0};
  std::string active_fen;
  bool is_active{false};
  std::vector<std::string> executed_moves;
};

/**
 * @brief Aggregate managing campaign targets, quotas, crash recovery, and session persistence.
 */
class QuotaTracker
{
public:
  QuotaTracker();
  explicit QuotaTracker(const std::filesystem::path &config_path);
  explicit QuotaTracker(const YAML::Node &config_node);

  void load_config(const YAML::Node &config_node);
  void load_config_file(const std::filesystem::path &config_path);

  // Crash/Restart recovery: scans base directory for episode_*/episode_info.yaml
  uint32_t scan_and_recover(const std::filesystem::path &base_output_dir);

  // Record an episode given its parsed SmolVLA task instruction
  bool record_episode(const std::string &task_instruction);

  // Session state management
  bool load_session_state(const std::filesystem::path &file_path);
  bool save_session_state(const std::filesystem::path &file_path) const;

  // Quota queries
  [[nodiscard]] uint32_t get_completed(StepKind kind, const PieceKey &piece) const;
  [[nodiscard]] uint32_t get_target(StepKind kind, const PieceKey &piece) const;
  [[nodiscard]] uint32_t get_remaining_quota(StepKind kind, const PieceKey &piece) const;
  [[nodiscard]] double get_deficit(chess::Color color, chess::PieceType piece_type) const;
  [[nodiscard]] uint32_t get_repeat_count(const std::string &move_signature) const;

  // Session move history (prevent repeating moves in active session)
  void record_executed_move(const std::string &uci);
  [[nodiscard]] bool has_executed_move(const std::string &uci) const;
  [[nodiscard]] const std::vector<std::string> &get_executed_moves() const { return session_state_.executed_moves; }

  [[nodiscard]] bool is_campaign_complete() const;
  [[nodiscard]] std::string get_next_clear_bin_side() const;
  [[nodiscard]] QuotaSummary get_summary() const;

  [[nodiscard]] SessionState get_session_state() const { return session_state_; }
  void set_session_state(const SessionState &state) { session_state_ = state; }
  void advance_session();
  void reset_session(const std::string &new_fen);

private:
  void init_default_quotas();
  bool parse_instruction_and_increment(const std::string &instruction);

  uint32_t total_target_{600};
  uint32_t episodes_per_session_{20};

  std::map<StepKind, std::map<PieceKey, uint32_t>> targets_;
  std::map<StepKind, std::map<PieceKey, uint32_t>> completed_;

  uint32_t clear_left_target_{60};
  uint32_t clear_right_target_{60};
  uint32_t clear_left_completed_{0};
  uint32_t clear_right_completed_{0};

  std::map<std::string, uint32_t> repeat_counts_;
  SessionState session_state_;
};

/**
 * @brief Synthesizes valid, dense, and branching chess positions (8-12 pieces)
 * optimized for deficit quotas and physical chess set limitations.
 */
class FenSynthesizer
{
public:
  explicit FenSynthesizer(uint64_t seed = 0);

  /**
   * @brief Synthesizes an optimal FEN position given current deficit weights.
   * Ensures physical set piece count, 8-12 total pieces, inactive side not in check,
   * active side having >= 8 legal moves and not in checkmate/stalemate.
   */
  std::string synthesize_fen(const QuotaTracker &tracker, int max_attempts = 200);

private:
  std::mt19937_64 rng_;

  std::vector<chess::PieceType> sample_pieces_weighted(
    chess::Color color,
    int count,
    const QuotaTracker &tracker);

  bool validate_position(const chess::Board &board) const;
};

/**
 * @brief Selects and scores candidate moves based on feasibility, quota progress,
 * and downstream branching factor.
 */
class MoveCandidateSelector
{
public:
  using FeasibilityChecker = std::function<std::pair<bool, uint8_t>(const MoveCandidate &)>;

  MoveCandidateSelector() = default;

  /**
   * @brief Evaluates all legal moves on board, queries feasibility, applies downgrade logic,
   * identifies bypass steps, scores candidates, and returns sorted candidates descending by score.
   * Excludes any moves already executed in the active session.
   */
  std::vector<MoveCandidate> evaluate_candidates(
    const chess::Board &board,
    const QuotaTracker &tracker,
    const FeasibilityChecker &feasibility_checker);

  /**
   * @brief Selects best candidate move with score > 0.
   */
  std::optional<MoveCandidate> select_best_move(
    const chess::Board &board,
    const QuotaTracker &tracker,
    const FeasibilityChecker &feasibility_checker);

  /**
   * @brief Scores a candidate move based on active steps, downstream branching,
   * and bypass penalties.
   */
  static double compute_score(
    const MoveCandidate &candidate,
    const chess::Board &board,
    const QuotaTracker &tracker,
    size_t next_legal_moves_count);
};

} // namespace chess_episode_recorder::domain


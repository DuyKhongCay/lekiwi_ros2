// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <chess.hpp>

namespace chess_episode_recorder::domain
{

/**
 * @brief Atomic manipulation step kinds within the SmolVLA data collection framework.
 */
enum class StepKind : uint8_t
{
  MOVE,   // Robot navigates and moves piece from square to square
  CLEAR,  // Robot clears captured opponent piece to designated side bin
  PICK,   // Dedicated pick phase for decoupled or multi-base trajectories
  PLACE   // Dedicated place phase for decoupled or multi-base trajectories
};

[[nodiscard]] std::string to_string(StepKind kind);
[[nodiscard]] std::optional<StepKind> step_kind_from_string(std::string_view str);

/**
 * @brief Domain identifier for a chess piece entity (color + piece type).
 */
struct PieceKey
{
  chess::Color color{chess::Color::WHITE};
  chess::PieceType type{chess::PieceType::PAWN};

  bool operator<(const PieceKey &other) const noexcept
  {
    if (color != other.color) {
      return static_cast<int>(color.internal()) < static_cast<int>(other.color.internal());
    }
    return static_cast<int>(type.internal()) < static_cast<int>(other.type.internal());
  }

  bool operator==(const PieceKey &other) const noexcept
  {
    return color == other.color && type == other.type;
  }

  bool operator!=(const PieceKey &other) const noexcept
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
 * @brief Outcome of verifying actual physical board FEN against expected move.
 */
enum class BoardTransitionOutcome : uint8_t
{
  MATCHED_EXPECTED_MOVE,  ///< Actual board matched expected candidate move.
  DIVERGED_RESYNCED,      ///< Actual board diverged (teleop intervention) and active FEN resynced.
  INVALID_ACTUAL_FEN      ///< Actual board FEN is corrupted or illegal.
};

/**
 * @brief Structured result from parsing a natural language task instruction.
 */
struct ParsedInstruction
{
  StepKind kind{StepKind::MOVE};
  PieceKey piece;
  std::string bin_side; // "left" or "right" for CLEAR, empty otherwise
};

/**
 * @brief Pure domain parser for SmolVLA natural language instructions.
 */
class InstructionParser
{
public:
  [[nodiscard]] static std::optional<ParsedInstruction> parse(std::string_view instruction);
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
  std::string initial_fen;
  std::string active_fen;
  bool is_active{false};
  std::vector<std::string> executed_moves;
};

/**
 * @brief Campaign target quotas configuration (pure C++ domain POCO).
 */
struct CampaignConfig
{
  uint32_t total_episodes{600};
  uint32_t episodes_per_session{20};
  uint32_t move_total{240};
  uint32_t clear_total{120};
  uint32_t pick_total{120};
  uint32_t place_total{120};
};

/**
 * @brief Pure in-memory aggregate managing campaign quotas and session board progression.
 * Zero filesystem, zero YAML, zero ROS 2 dependencies.
 */
class QuotaTracker
{
public:
  explicit QuotaTracker(const CampaignConfig &config = CampaignConfig{});

  void configure(const CampaignConfig &config);
  [[nodiscard]] const CampaignConfig &get_config() const noexcept { return config_; }

  // Record an episode given its parsed SmolVLA task instruction
  bool record_completed_instruction(const std::string &instruction);
  bool record_episode(const std::string &task_instruction);

  // In-memory state restoration (used by Storage adapter)
  void restore_session_state(const SessionState &state);
  bool reconstruct_session_board();

  // Sequential Board & Move Progression
  [[nodiscard]] const chess::Board &get_active_board() const noexcept { return active_board_; }
  [[nodiscard]] const std::string &get_active_fen() const noexcept { return session_state_.active_fen; }
  void set_active_fen(const std::string &fen);
  bool apply_move(const std::string &uci);

  // Verifies physical board against candidate move and commits state transition
  BoardTransitionOutcome verify_and_commit_move(
    const std::string &expected_uci,
    const std::string &actual_fen);

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
  [[nodiscard]] bool is_step_kind_quota_reached(StepKind kind) const noexcept;
  [[nodiscard]] bool is_session_budget_exhausted() const noexcept
  {
    return session_state_.episodes_in_session >= episodes_per_session_;
  }
  [[nodiscard]] std::string get_next_clear_bin_side() const;
  [[nodiscard]] QuotaSummary get_summary() const;

  [[nodiscard]] SessionState get_session_state() const { return session_state_; }
  void set_session_state(const SessionState &state);
  void advance_session();
  void reset_session(const std::string &new_fen);

private:
  void init_default_quotas();

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
  chess::Board active_board_;
  CampaignConfig config_;
};

} // namespace chess_episode_recorder::domain


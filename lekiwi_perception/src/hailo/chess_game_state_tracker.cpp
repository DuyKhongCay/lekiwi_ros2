/**
 * @file chess_game_state_tracker.cpp
 * @brief Implementation of in-process chess game state tracking and legal move validation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "hailo/chess_game_state_tracker.hpp"

#include <utility>

namespace lekiwi_perception::hailo
{

  ChessGameStateTracker::ChessGameStateTracker(int debounce_frames)
      : board_(),
        debounce_frames_(debounce_frames),
        last_accepted_placement_("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"),
        pending_placement_(""),
        consecutive_count_(0)
  {
  }

  void ChessGameStateTracker::reset()
  {
    board_ = chess::Board();
    last_accepted_placement_ = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
    pending_placement_.clear();
    consecutive_count_ = 0;
  }

  std::string ChessGameStateTracker::get_full_fen() const
  {
    return board_.getFen();
  }

  bool ChessGameStateTracker::match_legal_move(
      const std::string &detected_placement, chess::Move &matched_move)
  {
    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board_);

    for (const auto &move : legal_moves)
    {
      board_.makeMove(move);
      std::string sim_fen = board_.getFen();
      board_.unmakeMove(move);

      auto space_idx = sim_fen.find(' ');
      std::string sim_placement = (space_idx != std::string::npos) ? sim_fen.substr(0, space_idx) : sim_fen;

      if (sim_placement == detected_placement)
      {
        matched_move = move;
        return true;
      }
    }
    return false;
  }

  GameStateResult ChessGameStateTracker::update(const std::string &placement)
  {
    GameStateResult res;
    res.is_legal_move = false;
    res.is_board_stable = false;
    res.last_move.clear();

    if (placement.empty())
    {
      res.full_fen = board_.getFen();
      return res;
    }

    // Debounce filter
    if (placement == pending_placement_)
    {
      consecutive_count_++;
    }
    else
    {
      pending_placement_ = placement;
      consecutive_count_ = 1;
    }

    if (consecutive_count_ >= debounce_frames_)
    {
      res.is_board_stable = true;
    }

    // Starting position detected
    const std::string START_PLACEMENT = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
    if (placement == START_PLACEMENT)
    {
      if (last_accepted_placement_ != START_PLACEMENT && res.is_board_stable)
      {
        reset();
      }
      res.is_legal_move = true;
      res.full_fen = board_.getFen();
      return res;
    }

    // If placement is identical to current accepted board placement, it is legal and steady
    if (placement == last_accepted_placement_)
    {
      res.is_legal_move = true;
      res.full_fen = board_.getFen();
      return res;
    }

    // If not yet stable, do not transition game state
    if (!res.is_board_stable)
    {
      res.full_fen = board_.getFen();
      return res;
    }

    // Check if stable new placement matches any legal move
    chess::Move matched_move;
    if (match_legal_move(placement, matched_move))
    {
      res.is_legal_move = true;
      res.last_move = chess::uci::moveToUci(matched_move);
      board_.makeMove(matched_move);
      last_accepted_placement_ = placement;
      res.full_fen = board_.getFen();
    }
    else
    {
      res.is_legal_move = false;
      res.full_fen = board_.getFen();
    }

    return res;
  }

} // namespace lekiwi_perception::hailo

/**
 * @file chess_domain.hpp
 * @brief Pure C++ domain logic for FIDE chess move classification and validation.
 *
 * Implements deterministic chess move analysis against chess::Board representation
 * to populate lekiwi_interfaces::msg::ChessMoveDetails. Operates purely in memory
 * with no external ROS 2 node or middleware dependencies.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <string>
#include <string_view>
#include <chess.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>

namespace lekiwi_chess_master::domain
{

  /**
   * @brief Converts chess::PieceType to lowercase canonical name ("pawn", "knight", etc.).
   *
   * Provides consistent naming for downstream vision, manipulation, and UI components.
   *
   * @param[in] pt Enum piece type to convert.
   * @return Lowercase piece name string, or "none" if invalid/empty.
   */
  [[nodiscard]] std::string piece_type_to_string(chess::PieceType pt);

  /**
   * @brief Classifies a UCI move string against current board state into rich move details.
   *
   * @details Evaluates move geometry, legality, piece dynamics, capture targets, and special
   * rules according to FIDE standards:
   * - En-passant pawn capture coordinate resolution ((to.file, from.rank)).
   * - Kingside/queenside castling rook travel trajectory determination.
   * - Pawn promotion piece type mapping.
   * - Standard Algebraic Notation (SAN) generation with fallback on exception.
   *
   * @param[in] board Reference to current FIDE board state before executing the move.
   * @param[in] uci_move Cleaned UCI move token (e.g. "e2e4", "e7e8q").
   * @return Populated ChessMoveDetails message containing geometric coordinates, capture
   * flags, and piece types.
   *
   * @pre board must represent a valid FIDE game position.
   * @post If uci_move is malformed or shorter than 4 characters, returns empty details msg.
   * @note If the move is illegal in the current position, populates fallback promotion and SAN
   * fields without crashing or throwing exceptions.
   */
  [[nodiscard]] lekiwi_interfaces::msg::ChessMoveDetails classify_move(
      const chess::Board &board,
      std::string_view uci_move);

} // namespace lekiwi_chess_master::domain

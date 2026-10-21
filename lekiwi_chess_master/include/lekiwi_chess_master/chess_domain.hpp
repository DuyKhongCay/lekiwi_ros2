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

  /**
   * @brief Validates a standard FEN or 1-field placement string and initializes chess::Board.
   *
   * @details Transactional validation:
   * 1. Trims leading and trailing whitespace.
   * 2. Validates placement token syntax: exactly 8 ranks separated by 7 '/', each rank summing to 8
   *    squares with valid piece characters (PNBRQKpnbrqk) or digits 1..8.
   * 3. Loads candidate into a scratch chess::Board instance.
   * 4. Enforces exactly 1 White King and 1 Black King.
   * 5. Enforces that pawns cannot be on the 1st or 8th rank.
   * 6. Enforces that the inactive side (the side NOT to move) is not currently under check.
   *
   * @param[in] fen_str Input FEN string (full 6-field or 1-field piece placement).
   * @param[out] out_board Populated valid chess::Board if successful.
   * @param[out] out_error Detailed failure explanation if invalid.
   * @return True if valid and safely parseable, false otherwise.
   */
  [[nodiscard]] bool validate_and_parse_fen(
      std::string_view fen_str,
      chess::Board &out_board,
      std::string &out_error);

} // namespace lekiwi_chess_master::domain

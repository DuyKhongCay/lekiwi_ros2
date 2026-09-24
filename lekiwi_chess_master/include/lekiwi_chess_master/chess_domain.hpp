/**
 * @file chess_domain.hpp
 * @brief Pure domain logic for chess move classification and validation.
 *
 * Implements FIDE move analysis (capture, en-passant, castling, promotion, piece type)
 * using the chess::Board domain model.
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
     * @brief Convert a chess::PieceType to a lowercase canonical string name.
     * e.g., "pawn", "knight", "bishop", "rook", "queen", "king", or "none".
     */
    [[nodiscard]] std::string piece_type_to_string(chess::PieceType pt);

    /**
     * @brief Classify a UCI move string against the current board state.
     *
     * Populates all fields of ChessMoveDetails:
     * - uci
     * - from_square
     * - to_square
     * - piece_type
     * - promotion_piece
     * - is_capture
     * - is_en_passant
     * - is_castling
     *
     * @param board Current board state
     * @param uci_move Cleaned UCI move string (e.g. "e2e4", "e7e8q")
     * @return Populated ChessMoveDetails message
     */
    [[nodiscard]] lekiwi_interfaces::msg::ChessMoveDetails classify_move(
        const chess::Board &board,
        std::string_view uci_move);

} // namespace lekiwi_chess_master::domain

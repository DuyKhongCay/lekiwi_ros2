/**
 * @file chess_domain.cpp
 * @brief Pure domain logic implementation for chess move classification.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_chess_master/chess_domain.hpp"

#include <algorithm>
#include <cctype>

namespace lekiwi_chess_master::domain
{

  std::string piece_type_to_string(chess::PieceType pt)
  {
    switch (pt.internal())
    {
    case chess::PieceType::PAWN:
      return "pawn";
    case chess::PieceType::KNIGHT:
      return "knight";
    case chess::PieceType::BISHOP:
      return "bishop";
    case chess::PieceType::ROOK:
      return "rook";
    case chess::PieceType::QUEEN:
      return "queen";
    case chess::PieceType::KING:
      return "king";
    default:
      return "none";
    }
  }

  lekiwi_interfaces::msg::ChessMoveDetails classify_move(
      const chess::Board &board,
      std::string_view uci_move)
  {
    lekiwi_interfaces::msg::ChessMoveDetails details;

    // 1. Guard against empty, null move "(none)", or too short strings
    if (uci_move.empty() || uci_move == "(none)" || uci_move.length() < 4)
    {
      return details;
    }

    // 2. Validate square characters [a-h][1-8][a-h][1-8]
    char f_col = static_cast<char>(std::tolower(static_cast<unsigned char>(uci_move[0])));
    char f_row = uci_move[1];
    char t_col = static_cast<char>(std::tolower(static_cast<unsigned char>(uci_move[2])));
    char t_row = uci_move[3];

    if (f_col < 'a' || f_col > 'h' || f_row < '1' || f_row > '8' ||
        t_col < 'a' || t_col > 'h' || t_row < '1' || t_row > '8')
    {
      return details;
    }

    details.uci = std::string(uci_move);
    details.from_square = std::string{f_col, f_row};
    details.to_square = std::string{t_col, t_row};
    details.piece_color = (board.sideToMove() == chess::Color::WHITE) ? "w" : "b";

    chess::Move move = chess::uci::uciToMove(board, uci_move);
    if (move == chess::Move::NO_MOVE)
    {
      // Fallback promotion piece parsing if move is not directly legal on this board
      if (uci_move.length() >= 5)
      {
        char promo_ch = static_cast<char>(std::tolower(static_cast<unsigned char>(uci_move[4])));
        if (promo_ch == 'q')
          details.promotion_piece = "queen";
        else if (promo_ch == 'r')
          details.promotion_piece = "rook";
        else if (promo_ch == 'b')
          details.promotion_piece = "bishop";
        else if (promo_ch == 'n')
          details.promotion_piece = "knight";
      }
      details.san = details.uci;
      return details;
    }

    // Moving piece
    auto piece_at_from = board.at(move.from());
    details.piece_type = piece_type_to_string(piece_at_from.type());

    // Capture handling
    details.is_capture = board.isCapture(move);
    details.is_en_passant = (move.typeOf() == chess::Move::ENPASSANT);

    if (details.is_en_passant)
    {
      // En Passant: captured pawn is at (to.file, from.rank)
      chess::Square cap_sq(move.to().file(), move.from().rank());
      details.captured_square = static_cast<std::string>(cap_sq);
      details.captured_piece_type = "pawn";
    }
    else if (details.is_capture)
    {
      details.captured_square = details.to_square;
      details.captured_piece_type = piece_type_to_string(board.at(move.to()).type());
    }

    // Castling handling
    details.is_castling = (move.typeOf() == chess::Move::CASTLING);
    if (details.is_castling)
    {
      bool is_white = (board.sideToMove() == chess::Color::WHITE);
      bool is_kingside = (move.to().file() > move.from().file());
      if (is_white)
      {
        if (is_kingside)
        {
          details.castling_rook_from = "h1";
          details.castling_rook_to = "f1";
        }
        else
        {
          details.castling_rook_from = "a1";
          details.castling_rook_to = "d1";
        }
      }
      else
      {
        if (is_kingside)
        {
          details.castling_rook_from = "h8";
          details.castling_rook_to = "f8";
        }
        else
        {
          details.castling_rook_from = "a8";
          details.castling_rook_to = "d8";
        }
      }
    }

    // Promotion handling
    if (move.typeOf() == chess::Move::PROMOTION)
    {
      details.promotion_piece = piece_type_to_string(move.promotionType());
    }

    // Standard Algebraic Notation (SAN)
    try
    {
      details.san = chess::uci::moveToSan(board, move);
    }
    catch (...)
    {
      details.san = details.uci;
    }

    return details;
  }

} // namespace lekiwi_chess_master::domain

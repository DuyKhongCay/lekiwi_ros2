/**
 * @file chess_domain.cpp
 * @brief Implementation of pure C++ FIDE chess move classification domain logic.
 *
 * Translates geometric UCI moves into complete ChessMoveDetails ROS 2 message fields,
 * computing capture coordinates, castling rook trajectories, and en-passant squares.
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
    // Map internal chess library enum to lowercase strings for downstream ROS messages
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

    // Guard against empty strings, engine null moves ("(none)"), or malformed short tokens
    if (uci_move.empty() || uci_move == "(none)" || uci_move.length() < 4)
    {
      return details;
    }

    // Validate standard algebraic square characters: [a-h][1-8] for both from and to squares
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
    details.piece_color = (board.sideToMove() == chess::Color::WHITE) ? "w" : "b";

    chess::Move move = chess::uci::uciToMove(board, uci_move);
    if (move == chess::Move::NO_MOVE)
    {
      // Fallback promotion piece parsing if move is not legal or out-of-order on this board
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
      return details;
    }

    // Identify the piece executing the move from the source square
    auto piece_at_from = board.at(move.from());
    details.piece_type = piece_type_to_string(piece_at_from.type());

    // Evaluate capture properties and merge captured piece type into piece_type
    details.is_capture = board.isCapture(move);
    details.is_en_passant = (move.typeOf() == chess::Move::ENPASSANT);

    if (details.is_capture)
    {
      std::string cap_type = details.is_en_passant
        ? "pawn"
        : piece_type_to_string(board.at(move.to()).type());
      details.piece_type += ":" + cap_type;
    }

    // Evaluate castling and promotion flags
    details.is_castling = (move.typeOf() == chess::Move::CASTLING);

    if (move.typeOf() == chess::Move::PROMOTION)
    {
      details.promotion_piece = piece_type_to_string(move.promotionType());
    }

    return details;
  }

  bool validate_and_parse_fen(
      std::string_view fen_str,
      chess::Board &out_board,
      std::string &out_error)
  {
    // 1. Trim leading and trailing whitespace
    while (!fen_str.empty() && std::isspace(static_cast<unsigned char>(fen_str.front())))
    {
      fen_str.remove_prefix(1);
    }
    while (!fen_str.empty() && std::isspace(static_cast<unsigned char>(fen_str.back())))
    {
      fen_str.remove_suffix(1);
    }

    if (fen_str.empty())
    {
      out_error = "FEN string is empty";
      return false;
    }

    // 2. Validate placement token syntax (8 ranks, each summing to 8 squares)
    auto space_idx = fen_str.find(' ');
    std::string_view placement = (space_idx != std::string_view::npos)
                                     ? fen_str.substr(0, space_idx)
                                     : fen_str;

    int rank_count = 1;
    int current_rank_squares = 0;
    for (char c : placement)
    {
      if (c == '/')
      {
        if (current_rank_squares != 8)
        {
          out_error = "Invalid rank in piece placement: rank does not sum to 8 squares";
          return false;
        }
        rank_count++;
        current_rank_squares = 0;
      }
      else if (c >= '1' && c <= '8')
      {
        current_rank_squares += (c - '0');
      }
      else if (std::string_view("PNBRQKpnbrqk").find(c) != std::string_view::npos)
      {
        current_rank_squares += 1;
      }
      else
      {
        out_error = std::string("Invalid character in piece placement: '") + c + "'";
        return false;
      }
    }
    if (rank_count != 8 || current_rank_squares != 8)
    {
      out_error = "Piece placement must have exactly 8 ranks of 8 squares";
      return false;
    }

    // 3. Attempt to load into scratch chess::Board
    chess::Board scratch;
    if (!scratch.setFen(fen_str))
    {
      out_error = "Failed to parse FEN in chess engine library";
      return false;
    }

    // 4. Enforce exactly 1 White King and 1 Black King
    if (scratch.pieces(chess::PieceType::KING, chess::Color::WHITE).count() != 1)
    {
      out_error = "Position must have exactly one White king";
      return false;
    }
    if (scratch.pieces(chess::PieceType::KING, chess::Color::BLACK).count() != 1)
    {
      out_error = "Position must have exactly one Black king";
      return false;
    }

    // 5. Enforce pawns cannot be on 1st or 8th rank
    auto rank_1_8 = chess::Bitboard(chess::Rank::RANK_1) | chess::Bitboard(chess::Rank::RANK_8);
    auto all_pawns = scratch.pieces(chess::PieceType::PAWN);
    if ((all_pawns & rank_1_8) != 0ull)
    {
      out_error = "Pawns cannot be placed on the 1st or 8th rank";
      return false;
    }

    // 6. Enforce that inactive side is not under check
    chess::Color inactive_color = ~scratch.sideToMove();
    chess::Square inactive_king = scratch.kingSq(inactive_color);
    if (scratch.isAttacked(inactive_king, scratch.sideToMove()))
    {
      out_error = "Opposing king is under attack (illegal state for player to move)";
      return false;
    }

    out_board = scratch;
    return true;
  }

} // namespace lekiwi_chess_master::domain

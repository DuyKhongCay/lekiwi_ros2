// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/chessboard_mapper.hpp"

#include <algorithm>
#include <cctype>
#include <regex>
#include <stdexcept>

namespace lekiwi_motion
{

  ChessboardMapper::ChessboardMapper(
      double board_width,
      double board_height,
      double grasp_z,
      bool origin_at_center)
      : board_width_(board_width),
        board_height_(board_height),
        grasp_z_(grasp_z),
        origin_at_center_(origin_at_center),
        square_size_x_(board_width / 8.0),
        square_size_y_(board_height / 8.0)
  {
    if (board_width_ <= 0.0 || board_height_ <= 0.0)
    {
      throw std::invalid_argument(
          "Board dimensions must be positive (got " +
          std::to_string(board_width_) + "x" + std::to_string(board_height_) + ")");
    }
  }

  bool ChessboardMapper::is_valid_square(std::string_view square) noexcept
  {
    if (square.size() != 2)
    {
      return false;
    }
    const char f = static_cast<char>(std::tolower(static_cast<unsigned char>(square[0])));
    const char r = square[1];
    return (f >= 'a' && f <= 'h') && (r >= '1' && r <= '8');
  }

  bool ChessboardMapper::is_valid_uci_move(std::string_view uci_move) noexcept
  {
    if (uci_move.size() != 4 && uci_move.size() != 5)
    {
      return false;
    }
    if (!is_valid_square(uci_move.substr(0, 2)) || !is_valid_square(uci_move.substr(2, 2)))
    {
      return false;
    }
    if (uci_move.size() == 5)
    {
      const char promo = static_cast<char>(std::tolower(static_cast<unsigned char>(uci_move[4])));
      return promo == 'q' || promo == 'r' || promo == 'b' || promo == 'n';
    }
    return true;
  }

  SquareCoordinate ChessboardMapper::square_to_metric(
      std::string_view square,
      std::optional<double> z_offset) const
  {
    std::string cleaned;
    cleaned.reserve(square.size());
    for (char c : square)
    {
      if (!std::isspace(static_cast<unsigned char>(c)))
      {
        cleaned.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
      }
    }

    if (cleaned.size() != 2)
    {
      throw std::invalid_argument(
          "Invalid chess square notation: '" + std::string(square) + "'. Must be 2 characters.");
    }

    const char file_char = cleaned[0];
    const char rank_char = cleaned[1];

    if (file_char < 'a' || file_char > 'h' || rank_char < '1' || rank_char > '8')
    {
      throw std::invalid_argument(
          "Square '" + std::string(square) + "' is outside standard FIDE board limits [a-h][1-8].");
    }

    const int file_idx = file_char - 'a'; // 0 .. 7
    const int rank_idx = rank_char - '1'; // 0 .. 7

    double x{0.0};
    double y{0.0};
    if (origin_at_center_)
    {
      // Center of square relative to board center
      x = (static_cast<double>(file_idx) - 3.5) * square_size_x_;
      y = (static_cast<double>(rank_idx) - 3.5) * square_size_y_;
    }
    else
    {
      // Relative to A1 bottom-left corner
      x = (static_cast<double>(file_idx) + 0.5) * square_size_x_;
      y = (static_cast<double>(rank_idx) + 0.5) * square_size_y_;
    }

    const double z = z_offset.value_or(grasp_z_);

    return SquareCoordinate{
        std::move(cleaned),
        file_char,
        rank_char,
        file_idx,
        rank_idx,
        x,
        y,
        z};
  }

  UciMoveDetails ChessboardMapper::parse_uci_move(
      std::string_view uci_move,
      bool is_capture) const
  {
    std::string cleaned;
    cleaned.reserve(uci_move.size());
    for (char c : uci_move)
    {
      if (!std::isspace(static_cast<unsigned char>(c)))
      {
        cleaned.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
      }
    }

    if (!is_valid_uci_move(cleaned))
    {
      throw std::invalid_argument(
          "Invalid UCI move string: '" + std::string(uci_move) + "'. Expected format like 'e2e4' or 'e7e8q'.");
    }

    const std::string from_sq = cleaned.substr(0, 2);
    const std::string to_sq = cleaned.substr(2, 2);
    const std::string promo = (cleaned.size() == 5) ? cleaned.substr(4, 1) : "";

    const SquareCoordinate pick_coord = square_to_metric(from_sq);
    const SquareCoordinate place_coord = square_to_metric(to_sq);

    return UciMoveDetails{
        std::move(cleaned),
        from_sq,
        to_sq,
        promo,
        pick_coord,
        place_coord,
        is_capture};
  }

} // namespace lekiwi_motion

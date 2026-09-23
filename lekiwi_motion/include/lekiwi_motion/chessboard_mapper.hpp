// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_MOTION__CHESSBOARD_MAPPER_HPP_
#define LEKIWI_MOTION__CHESSBOARD_MAPPER_HPP_

#include <optional>
#include <string>
#include <string_view>

#include <geometry_msgs/msg/point.hpp>

namespace lekiwi_motion
{

  /// Represents a resolved 3D metric position for a chess square in chessboard_frame
  struct SquareCoordinate
  {
    std::string square_name;
    char file_char{'a'};
    char rank_char{'1'};
    int file_idx{0}; // 0 for 'a' .. 7 for 'h'
    int rank_idx{0}; // 0 for '1' .. 7 for '8'
    double x{0.0};
    double y{0.0};
    double z{0.0};

    geometry_msgs::msg::Point to_point_msg() const noexcept
    {
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      return pt;
    }
  };

  /// Structured decomposition of a UCI move command
  struct UciMoveDetails
  {
    std::string uci;
    std::string from_square;
    std::string to_square;
    std::string promotion; // e.g., "q", "r", "b", "n" or empty
    SquareCoordinate pick_coord;
    SquareCoordinate place_coord;
    bool is_capture{false};

    geometry_msgs::msg::Point pick_point() const noexcept
    {
      return pick_coord.to_point_msg();
    }

    geometry_msgs::msg::Point place_point() const noexcept
    {
      return place_coord.to_point_msg();
    }
  };

  /**
   * @brief Translates FIDE algebraic square names to metric coordinates.
   *
   * All dimensions must be supplied by the caller (typically loaded from ROS parameters).
   *
   * Assumes chessboard_frame has:
   * - X axis running along files a -> h
   * - Y axis running along ranks 1 -> 8
   * - Z axis pointing upwards perpendicular to board surface
   * - Origin at board center by default (or corner A1 if configured)
   */
  class ChessboardMapper
  {
  public:
    explicit ChessboardMapper(
        double board_width,
        double board_height,
        double grasp_z,
        bool origin_at_center = true);

    double board_width() const noexcept { return board_width_; }
    double board_height() const noexcept { return board_height_; }
    double square_size_x() const noexcept { return square_size_x_; }
    double square_size_y() const noexcept { return square_size_y_; }
    double grasp_z() const noexcept { return grasp_z_; }
    bool origin_at_center() const noexcept { return origin_at_center_; }

    /**
     * @brief Convert algebraic square (e.g., 'e4', 'a1', 'h8') to metric coordinates.
     * @throws std::invalid_argument if square notation is invalid.
     */
    SquareCoordinate square_to_metric(
        std::string_view square,
        std::optional<double> z_offset = std::nullopt) const;

    /**
     * @brief Parse standard UCI move string (e.g., 'e2e4', 'e7e8q') into structured details.
     * @throws std::invalid_argument if uci string is invalid.
     */
    UciMoveDetails parse_uci_move(
        std::string_view uci_move,
        bool is_capture = false) const;

    static bool is_valid_square(std::string_view square) noexcept;
    static bool is_valid_uci_move(std::string_view uci_move) noexcept;

  private:
    double board_width_{0.0};
    double board_height_{0.0};
    double grasp_z_{0.0};
    bool origin_at_center_{true};
    double square_size_x_{0.0};
    double square_size_y_{0.0};
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__CHESSBOARD_MAPPER_HPP_

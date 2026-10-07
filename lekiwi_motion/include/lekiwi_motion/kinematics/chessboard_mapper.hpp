/**
 * @file chessboard_mapper.hpp
 * @brief Coordinate mapper converting algebraic chess squares into metric 3D Cartesian coordinates.
 *
 * Implements board spatial calculations translating standard FIDE algebraic squares
 * (e.g. "e4", "h8") into metric poses relative to chessboard_frame for pick-and-place
 * manipulation planning.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__CHESSBOARD_MAPPER_HPP_
#define LEKIWI_MOTION__CHESSBOARD_MAPPER_HPP_

#include <optional>
#include <string>
#include <string_view>

#include <geometry_msgs/msg/point.hpp>

namespace lekiwi_motion
{

  /**
   * @struct SquareCoordinate
   * @brief Resolved 3D metric coordinate position of a single chess square.
   */
  struct SquareCoordinate
  {
    std::string square_name; ///< Two-character algebraic name (e.g., "e4").
    char file_char{'a'};     ///< FIDE column character ('a' .. 'h').
    char rank_char{'1'};     ///< FIDE row character ('1' .. '8').
    int file_idx{0};         ///< 0-indexed file index (0 for 'a' to 7 for 'h').
    int rank_idx{0};         ///< 0-indexed rank index (0 for '1' to 7 for '8').
    double x{0.0};           ///< Metric position along X axis in meters.
    double y{0.0};           ///< Metric position along Y axis in meters.
    double z{0.0};           ///< Grasp clearance height above board plane in meters.

    /**
     * @brief Converts coordinate to ROS 2 geometry_msgs::msg::Point.
     * @return Point message with x, y, z fields populated.
     */
    geometry_msgs::msg::Point to_point_msg() const noexcept
    {
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      return pt;
    }
  };

  /**
   * @struct UciMoveDetails
   * @brief Structured breakdown of a Universal Chess Interface (UCI) move notation.
   */
  struct UciMoveDetails
  {
    std::string uci;             ///< Full UCI move string (e.g., "e2e4", "e7e8q").
    std::string from_square;     ///< Source algebraic square (e.g., "e2").
    std::string to_square;       ///< Destination algebraic square (e.g., "e4").
    std::string promotion;       ///< Promotion piece designation ("q", "r", "b", "n") or empty.
    SquareCoordinate pick_coord; ///< 3D metric coordinate for pick operation.
    SquareCoordinate place_coord;///< 3D metric coordinate for place operation.
    bool is_capture{false};      ///< Flag indicating if move captures an opposing piece.

    /**
     * @brief Returns pick point message.
     * @return Point message representing pick square.
     */
    geometry_msgs::msg::Point pick_point() const noexcept
    {
      return pick_coord.to_point_msg();
    }

    /**
     * @brief Returns place point message.
     * @return Point message representing place square.
     */
    geometry_msgs::msg::Point place_point() const noexcept
    {
      return place_coord.to_point_msg();
    }
  };

  /**
   * @class ChessboardMapper
   * @brief Translates FIDE algebraic square names to metric coordinates.
   *
   * Converts between chessboard algebraic nomenclature and metric dimensions.
   * Assumes chessboard_frame coordinate conventions:
   * - X axis runs along files 'a' -> 'h'.
   * - Y axis runs along ranks '1' -> '8'.
   * - Z axis points upwards perpendicular to board surface.
   * - Origin is located at board center by default (or corner A1 if configured).
   */
  class ChessboardMapper
  {
  public:
    /**
     * @brief Constructs chessboard mapper with physical dimensions and grasp elevation.
     * @param[in] board_width Physical width of chessboard along files in meters.
     * @param[in] board_height Physical height of chessboard along ranks in meters.
     * @param[in] grasp_z Default grasping elevation above board surface in meters.
     * @param[in] origin_at_center True if (0,0) is at geometric center; false if at A1 outer corner.
     * @throws std::invalid_argument if board dimensions are non-positive.
     */
    explicit ChessboardMapper(
        double board_width,
        double board_height,
        double grasp_z,
        bool origin_at_center = true);

    [[nodiscard]] double board_width() const noexcept { return board_width_; }
    [[nodiscard]] double board_height() const noexcept { return board_height_; }
    [[nodiscard]] double square_size_x() const noexcept { return square_size_x_; }
    [[nodiscard]] double square_size_y() const noexcept { return square_size_y_; }
    [[nodiscard]] double grasp_z() const noexcept { return grasp_z_; }
    [[nodiscard]] bool origin_at_center() const noexcept { return origin_at_center_; }

    /**
     * @brief Converts algebraic square notation to metric 3D coordinates.
     *
     * @details Sanitizes input characters, verifies standard FIDE boundaries [a-h][1-8],
     * and maps integer square indices into metric space using square_size_x/y and origin offset.
     *
     * @param[in] square Algebraic square string (e.g., "e4", "a1", "h8").
     * @param[in] z_offset Optional explicit grasp elevation override (meters).
     * @return SquareCoordinate struct populated with metric coordinates.
     * @throws std::invalid_argument if square notation length is invalid or outside [a-h][1-8].
     */
    SquareCoordinate square_to_metric(
        std::string_view square,
        std::optional<double> z_offset = std::nullopt) const;

    /**
     * @brief Parses standard UCI move string into pick and place metric targets.
     *
     * @details Validates 4-character or 5-character (promotion) UCI notation, decomposes
     * into from/to squares, extracts optional promotion piece, and evaluates 3D coordinates.
     *
     * @param[in] uci_move UCI move string (e.g., "e2e4", "e7e8q").
     * @param[in] is_capture Whether this move involves an opponent piece capture.
     * @return UciMoveDetails with populated pick and place coordinates.
     * @throws std::invalid_argument if UCI notation is malformed or contains invalid squares.
     */
    UciMoveDetails parse_uci_move(
        std::string_view uci_move,
        bool is_capture = false) const;

    /**
     * @brief Validates if string denotes a legal 2-character FIDE square notation.
     * @param[in] square Notation to inspect.
     * @return True if format is [a-hA-H][1-8].
     */
    static bool is_valid_square(std::string_view square) noexcept;

    /**
     * @brief Validates if string conforms to UCI move grammar.
     * @param[in] uci_move Notation to inspect.
     * @return True if format is 4 or 5 characters with legal squares and promotion token.
     */
    static bool is_valid_uci_move(std::string_view uci_move) noexcept;

  private:
    double board_width_{0.0};       ///< Total chessboard width along X (meters).
    double board_height_{0.0};      ///< Total chessboard height along Y (meters).
    double grasp_z_{0.0};           ///< Default piece grasp elevation above board (meters).
    bool origin_at_center_{true};   ///< Whether chessboard frame origin is located at geometric center.
    double square_size_x_{0.0};     ///< Individual square width along X (meters).
    double square_size_y_{0.0};     ///< Individual square height along Y (meters).
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__CHESSBOARD_MAPPER_HPP_


/**
 * @file chess_constants.hpp
 * @brief Centralized domain constants for chess perception, piece classifications,
 *        board geometry, and standard FEN representations.
 *
 * Single Source of Truth (SSOT) adhering to Clean Code and DRY principles.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__HAILO__CHESS_CONSTANTS_HPP_
#define LEKIWI_PERCEPTION__HAILO__CHESS_CONSTANTS_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <opencv2/core.hpp>

namespace lekiwi_perception::hailo
{

  // ---------------------------------------------------------------------------
  // Neural Network Detection Thresholds
  // ---------------------------------------------------------------------------

  /// Confidence score threshold for valid piece detection
  inline constexpr float kPieceDetectionThreshold = 0.3F;

  /// Maximum number of bounding boxes allocated per frame
  inline constexpr std::size_t kPieceMaxBoxes = 100U;

  /// Bounding box vertical ratio for piece ground contact point (0.88 compensates for ~47 deg camera tilt)
  inline constexpr float kPieceBaseYRatio = 0.88F;

  // ---------------------------------------------------------------------------
  // Piece Classifications & Neural Network Labels
  // ---------------------------------------------------------------------------

  /// Total number of chess piece classes predicted by YOLO neural network (6 white, 6 black)
  inline constexpr std::size_t kPieceClassCount = 12U;

  /// Canonical piece labels indexed 0 to 11
  /// 0: B (White Bishop), 1: K (White King),   2: N (White Knight),
  /// 3: P (White Pawn),   4: Q (White Queen),  5: R (White Rook),
  /// 6: b (Black Bishop), 7: k (Black King),   8: n (Black Knight),
  /// 9: p (Black Pawn),  10: q (Black Queen), 11: r (Black Rook)
  inline constexpr std::array<const char *, kPieceClassCount> kPieceLabels = {
      "B", "K", "N", "P", "Q", "R", "b", "k", "n", "p", "q", "r"};

  /// All valid FEN piece characters
  inline constexpr std::string_view kPieceLabelChars = "BKNPQRbknpqr";

  /**
   * @brief Validates whether a given label is a valid single-character chess piece identifier.
   */
  inline bool is_valid_piece_label(const std::string &label)
  {
    return label.size() == 1U && kPieceLabelChars.find(label[0]) != std::string_view::npos;
  }

  /**
   * @brief Retrieves canonical piece label from 0-based neural network class ID.
   */
  inline std::string piece_label_from_class_id(uint32_t class_id)
  {
    if (class_id < kPieceClassCount)
    {
      return kPieceLabels[class_id];
    }
    return std::to_string(class_id);
  }

  // ---------------------------------------------------------------------------
  // Chessboard Geometry & Grid Constants
  // ---------------------------------------------------------------------------

  /// Total number of grid intersection points ($9 \times 9 = 81$)
  inline constexpr std::size_t kGridPointsCount = 81U;

  /// Number of float values for 81 2D grid coordinates ($81 \times 2 = 162$)
  inline constexpr std::size_t kGridDataFloats = 162U;

  /// Grid indices corresponding to the 4 corners of the chessboard:
  /// Index 0: BL (A1) = 72
  /// Index 1: BR (H1) = 80
  /// Index 2: TR (H8) = 8
  /// Index 3: TL (A8) = 0
  inline constexpr std::array<int, 4> kCornerGridIndices = {72, 80, 8, 0};

  /// Nominal corner algebraic names ordered [BL, BR, TR, TL]
  inline constexpr std::array<const char *, 4> kCornerNames = {"A1", "H1", "H8", "A8"};


  // ---------------------------------------------------------------------------
  // Piece Visualization Colors
  // ---------------------------------------------------------------------------

  /**
   * @brief Returns distinct BGR color for visualizer bounding box overlays.
   * Only includes the 12 canonical model prediction classes (BKNPQRbknpqr).
   */
  inline cv::Scalar get_piece_color_bgr(const std::string &label)
  {
    static const std::map<std::string, cv::Scalar> kPieceColors = {
        {"b", cv::Scalar(99, 30, 233)},   // Black Bishop: Deep Pink
        {"B", cv::Scalar(243, 150, 33)},  // White Bishop: Sky Blue
        {"k", cv::Scalar(182, 233, 29)},  // Black King:   Turquoise
        {"K", cv::Scalar(99, 110, 141)},  // White King:   Brown
        {"n", cv::Scalar(0, 152, 255)},   // Black Knight: Orange
        {"N", cv::Scalar(145, 171, 255)}, // White Knight: Light Pink
        {"p", cv::Scalar(212, 188, 0)},   // Black Pawn:   Bright Cyan
        {"P", cv::Scalar(59, 235, 255)},  // White Pawn:   Yellow
        {"q", cv::Scalar(176, 39, 156)},  // Black Queen:  Purple
        {"Q", cv::Scalar(251, 64, 224)},  // White Queen:  Magenta
        {"r", cv::Scalar(0, 255, 198)},   // Black Rook:   Lime
        {"R", cv::Scalar(161, 71, 13)}};  // White Rook:   Dark Blue

    auto it = kPieceColors.find(label);
    if (it != kPieceColors.end())
    {
      return it->second;
    }
    return cv::Scalar(0, 255, 0); // Default fallback: Green
  }

} // namespace lekiwi_perception::hailo

#endif // LEKIWI_PERCEPTION__HAILO__CHESS_CONSTANTS_HPP_

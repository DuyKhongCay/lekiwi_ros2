/**
 * @file chess_vision_mapper.hpp
 * @brief Spatial mapping between camera pixel detections, board homography, and FEN state.
 *
 * Converts 2D piece bounding box locations into board square coordinates (`a1`..`h8`),
 * formats occupied board state into FEN notation string, and renders debug overlays.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__HAILO__CHESS_VISION_MAPPER_HPP_
#define LEKIWI_PERCEPTION__HAILO__CHESS_VISION_MAPPER_HPP_

#include <opencv2/opencv.hpp>
#include "hailo_objects.hpp"

#include <map>
#include <string>
#include <vector>

namespace lekiwi_perception::hailo
{

    /**
     * @brief 2D representation of an AprilTag detected in the image.
     */
    struct Tag2D
    {
        int id{0};
        cv::Point2f center_norm{0.0F, 0.0F};
    };

    /**
     * @brief Structure representing a detected chess piece on 2D image plane.
     */
    struct PieceDetection
    {
        /// Piece label string (e.g. "w-king", "b-pawn").
        std::string label;
        /// Hailo NPU neural network class ID.
        int class_id{0};
        /// Neural network detection confidence score (0.0 .. 1.0).
        float confidence{0.0F};
        /// Bounding box rectangle in normalized/pixel coordinates.
        cv::Rect2f bbox;
        /// Base ground contact point of the piece.
        cv::Point2f base_pt;
        /// Algebraic square designation (e.g. "e4").
        std::string square;
    };

    /**
     * @brief Complete board state representation.
     */
    struct ChessboardState
    {
        /// FEN notation string representing active board setup.
        std::string fen;
        /// Total number of detected chess pieces.
        int num_pieces{0};
        /// Index of top-left square corner.
        int a1_corner_idx{0};
        /// Perspective transformation matrix.
        cv::Mat homography_matrix;
        /// Grid intersection points.
        std::vector<cv::Point2f> grid_points_norm;
        /// Outer boundary polygon points.
        std::vector<cv::Point2f> poly_points_norm;
        /// Detected piece array.
        std::vector<PieceDetection> pieces;
        /// Occupancy map from square string to piece code.
        std::map<std::string, std::string> occupancy_map;
    };

    /**
     * @brief Utilities for decoding Hailo ROI metadata and generating FEN.
     */
    class ChessVisionMapper
    {
    public:
        /**
         * @brief Determines the A1 corner index (0=TL, 1=TR, 2=BR, 3=BL) by matching AprilTags with grid corners.
         * @param[in] grid_points_norm 81 normalized grid intersection points.
         * @param[in] detected_tags List of detected AprilTags with IDs and normalized image centers.
         * @param[in] fallback_a1_idx Fallback A1 corner index if no valid tag matches.
         * @return Corner index corresponding to square A1: 0 (TL), 1 (TR), 2 (BR), or 3 (BL).
         */
        static int match_a1_corner_index(
            const std::vector<cv::Point2f> &grid_points_norm,
            const std::vector<Tag2D> &detected_tags,
            int fallback_a1_idx = 0);

        /**
         * @brief Decodes Hailo ROI tensors and populates board state.
         */
        [[nodiscard]] static bool decode_hailo_metadata(
            const HailoROIPtr &roi, ChessboardState &state);
        [[nodiscard]] static bool decode_hailo_metadata(
            const HailoROIPtr &roi, ChessboardState &state,
            const std::vector<Tag2D> &detected_tags);

        static void remap_board_orientation(ChessboardState &state, int new_a1_corner_idx);

        static std::string map_pixel_to_square(
            float cx, float cy, const cv::Mat &homography_matrix, int a1_corner_idx = 0);
        static std::string generate_fen(
            const std::map<std::string, std::string> &occupancy_map);
        static void draw_chessboard_overlay(
            cv::Mat &frame,
            const std::vector<cv::Point2f> &grid_points,
            const std::vector<cv::Point2f> &polygon_points = {});
        static void draw_piece_detections(
            cv::Mat &frame, const std::vector<PieceDetection> &pieces);
    };

} // namespace lekiwi_perception::hailo

#endif // LEKIWI_PERCEPTION__HAILO__CHESS_VISION_MAPPER_HPP_

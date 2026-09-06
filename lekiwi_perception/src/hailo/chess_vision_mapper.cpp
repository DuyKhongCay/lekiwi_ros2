/**
 * @file chess_vision_mapper.cpp
 * @brief Implementation of spatial mapping between detections, homography, and piece placement.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "hailo/chess_vision_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace lekiwi_perception::hailo
{

  static const std::vector<int> CORNER_GRID_INDICES = {0, 8, 80, 72};

  // ---------------------------------------------------------------------------
  // ChessVisionMapper Implementation
  // ---------------------------------------------------------------------------

  std::string ChessVisionMapper::map_pixel_to_square(
      float cx, float cy, const cv::Mat &homography_matrix, int a1_corner_idx)
  {
    if (homography_matrix.empty() || homography_matrix.cols != 3 || homography_matrix.rows != 3)
    {
      return "";
    }

    std::vector<cv::Point2f> src_pts = {cv::Point2f(cx, cy)};
    std::vector<cv::Point2f> dst_pts;
    cv::perspectiveTransform(src_pts, dst_pts, homography_matrix);

    float norm_x = dst_pts[0].x;
    float norm_y = dst_pts[0].y;

    if (norm_x < 0.0F || norm_x > 1.0F || norm_y < 0.0F || norm_y > 1.0F)
    {
      return "";
    }

    int c = std::clamp(static_cast<int>(std::floor(norm_x * 8.0F)), 0, 7);
    int r = std::clamp(static_cast<int>(std::floor(norm_y * 8.0F)), 0, 7);

    int rot_steps = (4 - (a1_corner_idx % 4)) % 4;
    if (rot_steps == 1)
    {
      int old_c = c;
      c = 7 - r;
      r = old_c;
    }
    else if (rot_steps == 2)
    {
      c = 7 - c;
      r = 7 - r;
    }
    else if (rot_steps == 3)
    {
      int old_c = c;
      c = r;
      r = 7 - old_c;
    }

    char file_char = static_cast<char>('a' + c);
    char rank_char = static_cast<char>('0' + (8 - r));
    return std::string(1, file_char) + std::string(1, rank_char);
  }

  std::string ChessVisionMapper::generate_fen(
      const std::map<std::string, std::string> &occupancy_map)
  {
    std::vector<char> files = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    std::vector<char> ranks = {'8', '7', '6', '5', '4', '3', '2', '1'};
    std::stringstream fen_stream;

    for (size_t r = 0; r < ranks.size(); ++r)
    {
      int empty_count = 0;
      for (size_t f = 0; f < files.size(); ++f)
      {
        std::string sq = std::string(1, files[f]) + std::string(1, ranks[r]);
        auto it = occupancy_map.find(sq);
        if (it != occupancy_map.end() && !it->second.empty())
        {
          if (empty_count > 0)
          {
            fen_stream << empty_count;
            empty_count = 0;
          }
          fen_stream << it->second;
        }
        else
        {
          empty_count++;
        }
      }
      if (empty_count > 0)
      {
        fen_stream << empty_count;
      }
      if (r < ranks.size() - 1)
      {
        fen_stream << "/";
      }
    }

    return fen_stream.str();
  }

  namespace
  {

    const std::map<int, std::string> kPieceClassMap = {
        {0, "B"}, {1, "K"}, {2, "N"}, {3, "P"}, {4, "Q"}, {5, "R"},
        {6, "b"}, {7, "k"}, {8, "n"}, {9, "p"}, {10, "q"}, {11, "r"}};

    bool is_piece_label(const std::string &label)
    {
      return label.size() == 1U && std::string("BKNPQRbknpqr").find(label[0]) != std::string::npos;
    }

  } // namespace

  bool ChessVisionMapper::decode_hailo_metadata(const HailoROIPtr &roi, ChessboardState &state)
  {
    if (!roi)
    {
      return false;
    }

    for (const auto &matrix_obj : roi->get_objects_typed(HAILO_MATRIX))
    {
      auto matrix = std::dynamic_pointer_cast<HailoMatrix>(matrix_obj);
      if (!matrix)
      {
        continue;
      }

      if (matrix->height() == 3U && matrix->width() == 3U)
      {
        const auto &data = matrix->get_data();
        if (data.size() == 9U)
        {
          state.homography_matrix = cv::Mat(3, 3, CV_32F);
          for (int r = 0; r < 3; ++r)
          {
            for (int c = 0; c < 3; ++c)
            {
              state.homography_matrix.at<float>(r, c) = data[r * 3 + c];
            }
          }
        }
      }
      else if (matrix->height() == 81U && matrix->width() == 2U)
      {
        const auto &data = matrix->get_data();
        if (data.size() == 162U)
        {
          state.grid_points_norm.clear();
          state.grid_points_norm.reserve(81U);
          for (size_t i = 0; i < 81U; ++i)
          {
            state.grid_points_norm.emplace_back(data[i * 2], data[i * 2 + 1]);
          }
        }
      }
    }

    state.pieces.clear();
    state.occupancy_map.clear();

    for (const auto &object : roi->get_objects_typed(HAILO_DETECTION))
    {
      auto det = std::dynamic_pointer_cast<HailoDetection>(object);
      if (!det)
      {
        continue;
      }

      std::string label = det->get_label();
      int class_id = det->get_class_id();
      if (label.empty())
      {
        auto it = kPieceClassMap.find(class_id);
        if (it != kPieceClassMap.end())
        {
          label = it->second;
        }
      }
      if (!is_piece_label(label))
      {
        continue;
      }

      auto bbox = det->get_bbox();
      PieceDetection piece;
      piece.label = label;
      piece.class_id = class_id;
      piece.confidence = det->get_confidence();
      piece.bbox = cv::Rect2f(bbox.xmin(), bbox.ymin(), bbox.width(), bbox.height());
      piece.base_pt = cv::Point2f(bbox.xmin() + bbox.width() / 2.0F, bbox.ymin() + bbox.height());

      if (!state.homography_matrix.empty())
      {
        piece.square = ChessVisionMapper::map_pixel_to_square(
            piece.base_pt.x, piece.base_pt.y, state.homography_matrix, state.a1_corner_idx);
        if (!piece.square.empty() && !piece.label.empty())
        {
          state.occupancy_map[piece.square] = piece.label;
        }
      }
      state.pieces.push_back(std::move(piece));
    }

    state.num_pieces = static_cast<int>(state.occupancy_map.size());
    if (!state.homography_matrix.empty() && !state.occupancy_map.empty())
    {
      state.piece_placement = ChessVisionMapper::generate_fen(state.occupancy_map);
    }
    return true;
  }

  // Canonical tag placement offset (in CW steps from corner A1):
  // Tag 1 (A1): offset 0 (0 steps from A1)
  // Tag 6 (A8): offset 1 (1 step CW from A1)
  // Tag 3 (H8): offset 2 (2 steps CW from A1)
  // Tag 4 (H1): offset 3 (3 steps CW from A1)
  static const std::map<int, int> TAG_CANONICAL_OFFSET = {
      {1, 0}, // A1
      {6, 1}, // A8
      {3, 2}, // H8
      {4, 3}, // H1
      {0, 0}, // Optional fallback for 0-based tag numbering
      {2, 2}
  };

  int ChessVisionMapper::match_a1_corner_index(
      const std::vector<cv::Point2f> &grid_points_norm,
      const std::vector<Tag2D> &detected_tags,
      int fallback_a1_idx)
  {
    if (detected_tags.empty() || grid_points_norm.size() != 81U)
    {
      return fallback_a1_idx;
    }

    // The 4 chessboard corners in normalized coordinates:
    // CORNER_GRID_INDICES: {0: TL, 8: TR, 80: BR, 72: BL}
    const std::vector<cv::Point2f> corner_pts = {
        grid_points_norm[CORNER_GRID_INDICES[0]], // 0: TL
        grid_points_norm[CORNER_GRID_INDICES[1]], // 1: TR
        grid_points_norm[CORNER_GRID_INDICES[2]], // 2: BR
        grid_points_norm[CORNER_GRID_INDICES[3]]  // 3: BL
    };

    // Consensus voting array for a1_corner_idx (0..3)
    int votes[4] = {0, 0, 0, 0};
    int valid_votes = 0;

    for (const auto &tag : detected_tags)
    {
      auto it = TAG_CANONICAL_OFFSET.find(tag.id);
      if (it == TAG_CANONICAL_OFFSET.end())
      {
        continue;
      }

      int tag_offset = it->second;

      // Find nearest image corner k in {0:TL, 1:TR, 2:BR, 3:BL}
      float min_dist_sq = 1e9F;
      int nearest_corner = 0;

      for (int k = 0; k < 4; ++k)
      {
        float dx = corner_pts[k].x - tag.center_norm.x;
        float dy = corner_pts[k].y - tag.center_norm.y;
        float d2 = dx * dx + dy * dy;
        if (d2 < min_dist_sq)
        {
          min_dist_sq = d2;
          nearest_corner = k;
        }
      }

      // Tag must be within reasonable proximity of a corner in normalized coords
      if (min_dist_sq < (0.35F * 0.35F))
      {
        int a1_idx = (nearest_corner - tag_offset + 4) % 4;
        votes[a1_idx]++;
        valid_votes++;
      }
    }

    if (valid_votes == 0)
    {
      return fallback_a1_idx;
    }

    int best_a1 = fallback_a1_idx;
    int max_votes = 0;
    for (int i = 0; i < 4; ++i)
    {
      if (votes[i] > max_votes)
      {
        max_votes = votes[i];
        best_a1 = i;
      }
    }

    return best_a1;
  }

  bool ChessVisionMapper::decode_hailo_metadata(
      const HailoROIPtr &roi, ChessboardState &state,
      const std::vector<Tag2D> &detected_tags)
  {
    if (!decode_hailo_metadata(roi, state))
    {
      return false;
    }

    if (!detected_tags.empty() && state.grid_points_norm.size() == 81U)
    {
      int matched_a1 = match_a1_corner_index(
          state.grid_points_norm, detected_tags, state.a1_corner_idx);
      if (matched_a1 != state.a1_corner_idx)
      {
        remap_board_orientation(state, matched_a1);
      }
    }
    return true;
  }

  void ChessVisionMapper::remap_board_orientation(
      ChessboardState &state, int new_a1_corner_idx)
  {
    state.a1_corner_idx = new_a1_corner_idx;
    if (state.homography_matrix.empty())
    {
      return;
    }
    state.occupancy_map.clear();
    for (auto &piece : state.pieces)
    {
      piece.square = ChessVisionMapper::map_pixel_to_square(
          piece.base_pt.x, piece.base_pt.y, state.homography_matrix, state.a1_corner_idx);
      if (!piece.square.empty() && !piece.label.empty())
      {
        state.occupancy_map[piece.square] = piece.label;
      }
    }
    state.num_pieces = static_cast<int>(state.occupancy_map.size());
    if (!state.occupancy_map.empty())
    {
      state.piece_placement = ChessVisionMapper::generate_fen(state.occupancy_map);
    }
  }

} // namespace lekiwi_perception::hailo

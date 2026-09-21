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

    float board_x = dst_pts[0].x;
    float board_y = dst_pts[0].y;

    if (board_x < -0.1F || board_x > 8.1F || board_y < -0.1F || board_y > 8.1F)
    {
      return "";
    }

    int c = std::clamp(static_cast<int>(std::floor(board_x)), 0, 7);
    int r = std::clamp(static_cast<int>(std::floor(board_y)), 0, 7);

    // a1_corner_idx: 0=BL, 1=BR, 2=TR, 3=TL
    int c_sq = c;
    int r_sq = r;
    switch ((a1_corner_idx % 4 + 4) % 4)
    {
    case 0: // A1 at BL (standard White)
      c_sq = c;
      r_sq = r;
      break;
    case 1: // A1 at BR (90 deg CCW)
      c_sq = 7 - r;
      r_sq = c;
      break;
    case 2: // A1 at TR (180 deg, Black)
      c_sq = 7 - c;
      r_sq = 7 - r;
      break;
    case 3: // A1 at TL (90 deg CW)
      c_sq = r;
      r_sq = 7 - c;
      break;
    }

    char file_char = static_cast<char>('a' + c_sq);
    char rank_char = static_cast<char>('0' + (8 - r_sq));
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
      else if (matrix->height() == kGridPointsCount && matrix->width() == 2U)
      {
        const auto &data = matrix->get_data();
        if (data.size() == kGridDataFloats)
        {
          state.grid_points_norm.clear();
          state.grid_points_norm.reserve(kGridPointsCount);
          for (size_t i = 0; i < kGridPointsCount; ++i)
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
        label = piece_label_from_class_id(static_cast<uint32_t>(class_id));
      }
      if (!is_valid_piece_label(label))
      {
        continue;
      }

      auto bbox = det->get_bbox();
      PieceDetection piece;
      piece.label = label;
      piece.class_id = class_id;
      piece.confidence = det->get_confidence();
      piece.bbox = cv::Rect2f(bbox.xmin(), bbox.ymin(), bbox.width(), bbox.height());
      piece.base_pt = cv::Point2f(
          bbox.xmin() + bbox.width() / 2.0F,
          bbox.ymin() + bbox.height() * kPieceBaseYRatio);

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

  // Canonical tag placement offset (in CCW steps from corner A1 [0: BL, 1: BR, 2: TR, 3: TL]):
  // Tag 0 (A1): offset 0 (0 steps from A1)
  // Tag 1 (H1): offset 1 (1 step CCW from A1)
  // Tag 2 (H8): offset 2 (2 steps CCW from A1)
  // Tag 3 (A8): offset 3 (3 steps CCW from A1)
  int ChessVisionMapper::match_a1_corner_index(
      const std::vector<cv::Point2f> &grid_points_norm,
      const std::vector<Tag2D> &detected_tags,
      const std::map<int, int> &tag_offsets)
  {
    if (detected_tags.empty() || grid_points_norm.size() != kGridPointsCount || tag_offsets.empty())
    {
      return -1;
    }

    // The 4 chessboard corners in normalized coordinates:
    // kCornerGridIndices: {72: BL, 80: BR, 8: TR, 0: TL}
    const std::vector<cv::Point2f> corner_pts = {
        grid_points_norm[kCornerGridIndices[0]], // 0: BL
        grid_points_norm[kCornerGridIndices[1]], // 1: BR
        grid_points_norm[kCornerGridIndices[2]], // 2: TR
        grid_points_norm[kCornerGridIndices[3]]  // 3: TL
    };

    // Consensus voting array for a1_corner_idx (0..3)
    int votes[4] = {0, 0, 0, 0};
    int valid_votes = 0;

    for (const auto &tag : detected_tags)
    {
      auto it = tag_offsets.find(tag.id);
      if (it == tag_offsets.end())
      {
        continue;
      }

      int tag_offset = it->second;

      // Find nearest image corner k in {0:BL, 1:BR, 2:TR, 3:TL}
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
      return -1;
    }

    int best_a1 = -1;
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
      const std::vector<Tag2D> &detected_tags,
      const std::map<int, int> &tag_offsets)
  {
    if (!decode_hailo_metadata(roi, state))
    {
      return false;
    }

    if (!detected_tags.empty() && state.grid_points_norm.size() == kGridPointsCount)
    {
      int matched_a1 = match_a1_corner_index(
          state.grid_points_norm, detected_tags, tag_offsets);
      if (matched_a1 != -1 && matched_a1 != state.a1_corner_idx)
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

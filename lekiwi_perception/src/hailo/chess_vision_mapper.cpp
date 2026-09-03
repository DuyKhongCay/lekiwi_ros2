/**
 * @file chess_vision_mapper.cpp
 * @brief Implementation of spatial mapping and FEN generation utilities.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "hailo/chess_vision_mapper.hpp"
#include <memory>

#include "hailo_common.hpp"
#include <cmath>
#include <sstream>

namespace lekiwi_perception::hailo
{

  static const cv::Scalar GRID_COLOR(255, 255, 0); // Cyan (BGR)
  static const cv::Scalar CORNER_COLOR(0, 0, 255); // Red (BGR)
  static const std::vector<int> CORNER_GRID_INDICES = {0, 8, 80, 72};
  static const std::vector<std::string> CORNER_LABELS = {"TL", "TR", "BR", "BL"};

  // Palette for bounding boxes (BGR)
  static const std::map<std::string, cv::Scalar> CLASS_COLORS_BGR = {
      {"B", cv::Scalar(255, 128, 0)},   // Light Blue
      {"K", cv::Scalar(42, 42, 165)},   // Brown
      {"N", cv::Scalar(193, 182, 255)}, // Light Pink
      {"P", cv::Scalar(0, 255, 255)},   // Yellow
      {"Q", cv::Scalar(255, 0, 255)},   // Magenta
      {"R", cv::Scalar(255, 0, 0)},     // Dark Blue
      {"b", cv::Scalar(60, 20, 220)},   // Crimson Red
      {"k", cv::Scalar(255, 255, 0)},   // Cyan
      {"n", cv::Scalar(0, 165, 255)},   // Orange
      {"p", cv::Scalar(255, 191, 0)},   // Sky Blue
      {"q", cv::Scalar(226, 43, 138)},  // Purple
      {"r", cv::Scalar(0, 255, 127)}    // Lime Green
  };

  std::string ChessVisionMapper::map_pixel_to_square(
      float cx, float cy, const cv::Mat &homography_matrix, int a1_corner_idx)
  {
    if (homography_matrix.empty() || homography_matrix.rows != 3 || homography_matrix.cols != 3)
    {
      return "";
    }

    cv::Mat pt = (cv::Mat_<double>(3, 1) << cx, cy, 1.0);
    cv::Mat warped = homography_matrix * pt;
    double w = warped.at<double>(2, 0);
    if (std::abs(w) < 1e-6)
      return "";

    int c = static_cast<int>(warped.at<double>(0, 0) / w);
    int r = static_cast<int>(warped.at<double>(1, 0) / w);

    if (c < 0 || c >= 8 || r < 0 || r >= 8)
      return "";

    int rot_steps = (3 - a1_corner_idx + 4) % 4;
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

    fen_stream << " w KQkq - 0 1";
    return fen_stream.str();
  }

  void ChessVisionMapper::draw_chessboard_overlay(
      cv::Mat &frame, const std::vector<cv::Point2f> &grid_pts_norm,
      const std::vector<cv::Point2f> &poly_pts_norm)
  {
    int w = frame.cols;
    int h = frame.rows;

    if (poly_pts_norm.size() >= 4)
    {
      std::vector<cv::Point> poly_px;
      poly_px.reserve(poly_pts_norm.size());
      for (const auto &pt : poly_pts_norm)
      {
        poly_px.push_back(cv::Point(static_cast<int>(pt.x * w), static_cast<int>(pt.y * h)));
      }
      cv::Mat overlay = frame.clone();
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{poly_px}, cv::Scalar(0, 200, 0));
      cv::addWeighted(overlay, 0.35, frame, 0.65, 0, frame);
      cv::polylines(frame, std::vector<std::vector<cv::Point>>{poly_px}, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    if (grid_pts_norm.size() != 81)
      return;

    std::vector<cv::Point> pts_px(81);
    for (size_t i = 0; i < 81; ++i)
    {
      pts_px[i] = cv::Point(static_cast<int>(grid_pts_norm[i].x * w), static_cast<int>(grid_pts_norm[i].y * h));
    }

    for (int row = 0; row < 9; ++row)
    {
      for (int col = 0; col < 8; ++col)
      {
        int idx_a = row * 9 + col;
        int idx_b = row * 9 + col + 1;
        cv::line(frame, pts_px[idx_a], pts_px[idx_b], GRID_COLOR, 1, cv::LINE_AA);
      }
    }

    for (int col = 0; col < 9; ++col)
    {
      for (int row = 0; row < 8; ++row)
      {
        int idx_a = row * 9 + col;
        int idx_b = (row + 1) * 9 + col;
        cv::line(frame, pts_px[idx_a], pts_px[idx_b], GRID_COLOR, 1, cv::LINE_AA);
      }
    }

    for (size_t i = 0; i < CORNER_GRID_INDICES.size(); ++i)
    {
      cv::Point pt = pts_px[CORNER_GRID_INDICES[i]];
      cv::circle(frame, pt, 4, CORNER_COLOR, -1, cv::LINE_AA);
      cv::putText(
          frame, CORNER_LABELS[i], cv::Point(pt.x + 5, pt.y - 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
  }

  void ChessVisionMapper::draw_piece_detections(
      cv::Mat &frame, const std::vector<PieceDetection> &pieces)
  {
    int w = frame.cols;
    int h = frame.rows;

    for (const auto &piece : pieces)
    {
      int x1 = static_cast<int>(piece.bbox.x * w);
      int y1 = static_cast<int>(piece.bbox.y * h);
      int bw = static_cast<int>(piece.bbox.width * w);
      int bh = static_cast<int>(piece.bbox.height * h);

      cv::Scalar color(0, 255, 0);
      auto it = CLASS_COLORS_BGR.find(piece.label);
      if (it != CLASS_COLORS_BGR.end())
        color = it->second;

      cv::rectangle(frame, cv::Rect(x1, y1, bw, bh), color, 2);

      int bx = static_cast<int>(piece.base_pt.x * w);
      int by = static_cast<int>(piece.base_pt.y * h);
      cv::circle(frame, cv::Point(bx, by), 3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

      std::stringstream label_ss;
      label_ss << piece.label;
      if (!piece.square.empty())
      {
        label_ss << " @" << piece.square;
      }
      std::string label_str = label_ss.str();

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(label_str, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
      int label_y1 = std::max(y1 - text_size.height - 4, 0);
      cv::rectangle(
          frame, cv::Rect(x1, label_y1, text_size.width + 4, text_size.height + 4),
          color, -1);
      cv::putText(
          frame, label_str, cv::Point(x1 + 2, label_y1 + text_size.height + 1),
          cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
  }

  namespace
  {

    const std::map<int, std::string> kPieceClassMap = {
        {0, "B"}, {1, "K"}, {2, "N"}, {3, "P"}, {4, "Q"}, {5, "R"}, {6, "b"}, {7, "k"}, {8, "n"}, {9, "p"}, {10, "q"}, {11, "r"}};

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

    for (const auto &object : roi->get_objects_typed(HAILO_MATRIX))
    {
      const auto matrix = std::dynamic_pointer_cast<HailoMatrix>(object);
      if (!matrix)
      {
        continue;
      }
      const auto &data = matrix->get_data();
      if (data.size() == 9U)
      {
        state.homography_matrix = cv::Mat(3, 3, CV_64F);
        for (std::size_t index = 0; index < data.size(); ++index)
        {
          state.homography_matrix.at<double>(
              static_cast<int>(index / 3U), static_cast<int>(index % 3U)) =
              static_cast<double>(data[index]);
        }
      }
      else if (data.size() == 162U)
      {
        state.grid_points_norm.clear();
        state.grid_points_norm.reserve(81U);
        for (std::size_t index = 0; index < 81U; ++index)
        {
          state.grid_points_norm.emplace_back(data[index * 2U], data[index * 2U + 1U]);
        }
      }
      else if (data.size() > 162U && (data.size() % 2U) == 0U)
      {
        state.poly_points_norm.clear();
        state.poly_points_norm.reserve(data.size() / 2U);
        for (std::size_t index = 0; index < data.size() / 2U; ++index)
        {
          state.poly_points_norm.emplace_back(data[index * 2U], data[index * 2U + 1U]);
        }
      }
    }

    constexpr float kBaseOffsetRatio = 0.15F;
    for (const auto &detection : hailo_common::get_hailo_detections(roi))
    {
      const auto bbox = detection->get_bbox();
      PieceDetection piece;
      piece.label = detection->get_label();
      piece.class_id = detection->get_class_id();
      piece.confidence = detection->get_confidence();
      piece.bbox = cv::Rect2f(bbox.xmin(), bbox.ymin(), bbox.width(), bbox.height());
      piece.base_pt = cv::Point2f(
          bbox.xmin() + bbox.width() / 2.0F,
          bbox.ymin() + bbox.height() * (1.0F - kBaseOffsetRatio));

      if (!is_piece_label(piece.label))
      {
        const auto label = kPieceClassMap.find(piece.class_id - 1);
        if (label != kPieceClassMap.end())
        {
          piece.label = label->second;
        }
      }

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
      state.fen = ChessVisionMapper::generate_fen(state.occupancy_map);
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
      state.fen = ChessVisionMapper::generate_fen(state.occupancy_map);
    }
  }

} // namespace lekiwi_perception::hailo

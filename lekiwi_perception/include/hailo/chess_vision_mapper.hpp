// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__HAILO__CHESS_VISION_MAPPER_HPP_
#define LEKIWI_PERCEPTION__HAILO__CHESS_VISION_MAPPER_HPP_

#include <opencv2/opencv.hpp>
#include "hailo_objects.hpp"

#include <map>
#include <string>
#include <vector>

namespace lekiwi_perception::hailo
{

struct PieceDetection
{
  std::string label;
  int class_id{0};
  float confidence{0.0F};
  cv::Rect2f bbox;
  cv::Point2f base_pt;
  std::string square;
};

struct ChessboardState
{
  std::string fen;
  int num_pieces{0};
  int a1_corner_idx{0};
  cv::Mat homography_matrix;
  std::vector<cv::Point2f> grid_points_norm;
  std::vector<cv::Point2f> poly_points_norm;
  std::vector<PieceDetection> pieces;
  std::map<std::string, std::string> occupancy_map;
};

class ChessVisionMapper
{
public:
  [[nodiscard]] static bool decode_hailo_metadata(
    const HailoROIPtr & roi, ChessboardState & state);

  static std::string map_pixel_to_square(
    float cx, float cy, const cv::Mat & homography_matrix, int a1_corner_idx = 0);
  static std::string generate_fen(
    const std::map<std::string, std::string> & occupancy_map);
  static void draw_chessboard_overlay(
    cv::Mat & frame,
    const std::vector<cv::Point2f> & grid_points,
    const std::vector<cv::Point2f> & polygon_points = {});
  static void draw_piece_detections(
    cv::Mat & frame, const std::vector<PieceDetection> & pieces);
};

}  // namespace lekiwi_perception::hailo

#endif  // LEKIWI_PERCEPTION__HAILO__CHESS_VISION_MAPPER_HPP_

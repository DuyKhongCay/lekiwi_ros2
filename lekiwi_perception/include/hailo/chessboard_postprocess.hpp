/**
 * @file chessboard_postprocess.hpp
 * @brief Homography and RANSAC perspective transform post-processor for chessboard detection.
 *
 * Extracts board boundary polygon from Hailo NPU segmentation masks, fits 4 edge lines using RANSAC,
 * computes homography perspective matrices ($M$ and $M^{-1}$), and projects an $8 \times 8$ grid of square regions.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#if defined(__has_include) && __has_include(<sys/cdefs.h>)
#include <sys/cdefs.h>
#endif

#ifndef __BEGIN_DECLS
#ifdef __cplusplus
#define __BEGIN_DECLS \
  extern "C"          \
  {
#define __END_DECLS }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif
#endif

#include "hailo_common.hpp"
#include "hailo_objects.hpp"
#include "hailo/hailo_gst_tensor_metadata.hpp"
#include "hailo_tensors.hpp"

/**
 * @brief Post-processor class interfacing with HailoRT postprocess plugin C-API.
 */
class ChessboardPostprocess
{
public:
  /**
   * @brief Filters ROI tensor data to compute chessboard perspective matrix and grid squares.
   * @param[in,out] roi Region of interest containing Hailo segmentation mask tensors.
   */
  void filter(HailoROIPtr roi);

private:
  static constexpr float kRansacThreshold = 0.015F;
  static constexpr int kMaxIterations = 25;
  static constexpr int kFullEvaluationInterval = 30;

  bool has_cache_{false};
  int frame_idx_{0};
  std::vector<float> cached_mat_data_;
  std::vector<float> cached_grid_data_;

  void add_cached_result(HailoROIPtr roi) const;
};

__BEGIN_DECLS

ChessboardPostprocess *init(const std::string &, const std::string &);
void free_resources(void *params_void_ptr);
void filter_chessboard(HailoROIPtr roi, void *params_void_ptr);
void filter(HailoROIPtr roi, void *params_void_ptr);

__END_DECLS

namespace lekiwi_perception::hailo
{

  float pointToSegmentDistance(const cv::Point2f &p, const cv::Point2f &a, const cv::Point2f &b);
  std::vector<cv::Point2f> sortCornersByPolarAngle(const std::vector<cv::Point2f> &corners);
  std::vector<cv::Point2f> extractInitial4Corners(const std::vector<cv::Point2f> &polygon_points);
  bool bestFitLineRansac(
      const std::vector<cv::Point2f> &points, cv::Vec4f &out_line,
      std::vector<cv::Point2f> &out_inliers, float threshold = 0.015f, int max_iters = 80);
  std::pair<std::vector<cv::Vec4f>, std::vector<cv::Point2f>> extract4EdgeLinesRansac(
      const std::vector<cv::Point2f> &polygon_points, float threshold = 0.015f, int max_iters = 80);
  bool lineIntersection(const cv::Vec4f &line1, const cv::Vec4f &line2, cv::Point2f &intersection);
  bool extractChessboardPerspective(
      const std::vector<cv::Point2f> &polygon_points,
      cv::Mat &out_M, cv::Mat &out_M_inv,
      std::vector<std::vector<cv::Point2f>> &out_squares,
      std::vector<cv::Point2f> &out_corners,
      float threshold = 0.015f, int max_iters = 80);

} // namespace lekiwi_perception::hailo

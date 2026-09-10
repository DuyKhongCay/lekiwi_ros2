/**
 * @file pose_solver.cpp
 * @brief Implementation of pure geometry and PnP solver for AprilTag chessboard localization.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "apriltag_localizer/pose_solver.hpp"

#include <cmath>
#include <utility>

namespace apriltag_localizer
{

  std::vector<cv::Point3d> PoseSolver::compute_tag_corners(
      const cv::Point3d &center, double size, double yaw_rad)
  {
    const double h = size / 2.0;
    // Calibrated corner ordering: matches calibrate_chessboard_tags.py
    // [h, -h], [-h, -h], [-h, h], [h, h]
    const std::vector<std::pair<double, double>> local_pts = {
        {h, -h},  // 0: Bottom-Right (Red)
        {-h, -h}, // 1: Bottom-Left  (Green)
        {-h, h},  // 2: Top-Left     (Blue)
        {h, h}};  // 3: Top-Right    (Yellow)

    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);

    std::vector<cv::Point3d> corners;
    corners.reserve(4);
    for (const auto &p : local_pts)
    {
      const double rx = p.first * cos_yaw - p.second * sin_yaw;
      const double ry = p.first * sin_yaw + p.second * cos_yaw;
      corners.emplace_back(center.x + rx, center.y + ry, center.z);
    }
    return corners;
  }

  bool PoseSolver::estimate_board_pose(
      const std::vector<std::vector<cv::Point2f>> &marker_corners,
      const std::vector<int> &marker_ids,
      const std::map<int, TagConfig> &tag_configs,
      const cv::Mat &camera_mat,
      const cv::Mat &dist_coeffs,
      cv::Mat &rvec,
      cv::Mat &tvec,
      int &used_tags_cnt)
  {
    used_tags_cnt = 0;
    if (marker_corners.empty() || marker_ids.empty() || camera_mat.empty())
    {
      return false;
    }

    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;

    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
      const int tag_id = marker_ids[i];
      auto it = tag_configs.find(tag_id);
      if (it != tag_configs.end() && it->second.corners_board.size() == 4U && marker_corners[i].size() == 4U)
      {
        const auto &cfg = it->second;
        for (size_t k = 0; k < 4U; ++k)
        {
          object_points.push_back(cfg.corners_board[k]);
          image_points.emplace_back(marker_corners[i][k].x, marker_corners[i][k].y);
        }
        used_tags_cnt++;
      }
    }

    // Hard constraint: Global Multi-Tag PnP requires >= 2 valid board tags (>= 8 points)
    if (used_tags_cnt < 2 || object_points.size() < 8U)
    {
      return false;
    }

    bool solve_ok = false;
    try
    {
      solve_ok = cv::solvePnP(
          object_points, image_points, camera_mat, dist_coeffs,
          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    catch (const cv::Exception &)
    {
      solve_ok = false;
    }

    if (!solve_ok || rvec.empty() || tvec.empty())
    {
      return false;
    }

    return (tvec.at<double>(2) > 0.0);
  }

} // namespace apriltag_localizer

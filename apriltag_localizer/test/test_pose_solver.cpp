/**
 * @file test_pose_solver.cpp
 * @brief Unit tests (L1 verification) for AprilTag pose estimation and orientation calculation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <cmath>
#include <vector>
#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "apriltag_localizer/chessboard_pose_estimator.hpp"

using namespace apriltag_localizer;

TEST(PoseSolverTest, TagCornersComputation)
{
  const cv::Point3d center(0.38, 0.0, 0.0);
  const double tag_size = 0.02; // 20mm
  const auto corners = PoseSolver::compute_tag_corners(center, tag_size, 0.0);

  ASSERT_EQ(corners.size(), 4U);

  // Check distances between adjacent corners
  const double d01 = cv::norm(corners[0] - corners[1]);
  const double d12 = cv::norm(corners[1] - corners[2]);
  const double d23 = cv::norm(corners[2] - corners[3]);
  const double d30 = cv::norm(corners[3] - corners[0]);

  EXPECT_NEAR(d01, tag_size, 1e-6);
  EXPECT_NEAR(d12, tag_size, 1e-6);
  EXPECT_NEAR(d23, tag_size, 1e-6);
  EXPECT_NEAR(d30, tag_size, 1e-6);

  // Check center
  const cv::Point3d computed_center = 0.25 * (corners[0] + corners[1] + corners[2] + corners[3]);
  EXPECT_NEAR(computed_center.x, center.x, 1e-6);
  EXPECT_NEAR(computed_center.y, center.y, 1e-6);
  EXPECT_NEAR(computed_center.z, center.z, 1e-6);
}

TEST(PoseSolverTest, SingleTagMathematicalIdentity)
{
  // Test T_cam^board = T_cam^tag * (T_board^tag)^-1 for all 4 tags
  const std::vector<std::pair<int, cv::Point3d>> tags = {
      {0, cv::Point3d(0.00, 0.00, 0.0)},
      {1, cv::Point3d(0.38, 0.00, 0.0)},
      {2, cv::Point3d(0.38, 0.38, 0.0)},
      {3, cv::Point3d(0.00, 0.38, 0.0)}};

  // Known ground truth camera-to-board pose
  cv::Mat rvec_gt = (cv::Mat_<double>(3, 1) << 0.1, -0.2, 0.05);
  cv::Mat tvec_gt = (cv::Mat_<double>(3, 1) << -0.15, 0.10, 0.65);

  cv::Mat R_gt;
  cv::Rodrigues(rvec_gt, R_gt);

  for (const auto &tag_info : tags)
  {
    const cv::Point3d &t_board_tag = tag_info.second;
    const double yaw_tag = 0.0;

    // T_cam^tag = T_cam^board * T_board^tag
    cv::Mat t_board_tag_mat = (cv::Mat_<double>(3, 1) << t_board_tag.x, t_board_tag.y, t_board_tag.z);
    cv::Mat R_cam_tag = R_gt.clone(); // yaw=0 -> R_board_tag = I
    cv::Mat t_cam_tag = tvec_gt + R_gt * t_board_tag_mat;

    cv::Mat rvec_tag;
    cv::Rodrigues(R_cam_tag, rvec_tag);

    // Compute board pose from single tag
    cv::Mat rvec_est, tvec_est;
    PoseSolver::compute_board_from_single_tag(
        rvec_tag, t_cam_tag, t_board_tag, yaw_tag, rvec_est, tvec_est);

    EXPECT_NEAR(tvec_est.at<double>(0), tvec_gt.at<double>(0), 1e-6);
    EXPECT_NEAR(tvec_est.at<double>(1), tvec_gt.at<double>(1), 1e-6);
    EXPECT_NEAR(tvec_est.at<double>(2), tvec_gt.at<double>(2), 1e-6);

    cv::Mat R_est;
    cv::Rodrigues(rvec_est, R_est);
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        EXPECT_NEAR(R_est.at<double>(r, c), R_gt.at<double>(r, c), 1e-5);
      }
    }
  }
}

TEST(PoseSolverTest, SingleTagAndMultiTagPnPAccuracy)
{
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 600.0, 0.0, 320.0,
                                 0.0, 600.0, 240.0,
                                 0.0, 0.0, 1.0);
  const cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

  // True board pose: camera at (0.19, 0.19, 0.6) looking straight down at board
  cv::Mat R_gt = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
                  0.0, -1.0, 0.0,
                  0.0, 0.0, -1.0);
  cv::Mat rvec_gt;
  cv::Rodrigues(R_gt, rvec_gt);
  cv::Mat tvec_gt = (cv::Mat_<double>(3, 1) << -0.19, 0.19, 0.60);

  // Setup tag configs for 4 tags
  std::map<int, TagConfig> tag_configs;
  const std::vector<std::pair<int, cv::Point3d>> tags = {
      {0, cv::Point3d(0.00, 0.00, 0.0)},
      {1, cv::Point3d(0.38, 0.00, 0.0)},
      {2, cv::Point3d(0.38, 0.38, 0.0)},
      {3, cv::Point3d(0.00, 0.38, 0.0)}};
  for (const auto &t : tags)
  {
    TagConfig cfg;
    cfg.id = t.first;
    cfg.center = t.second;
    cfg.yaw = 0.0;
    cfg.corners_board = PoseSolver::compute_tag_corners(cfg.center, 0.02, 0.0);
    tag_configs[cfg.id] = cfg;
  }

  // Generate synthetic 2D marker detections
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  for (const auto &t : tags)
  {
    const auto &corners_3d = tag_configs[t.first].corners_board;
    std::vector<cv::Point2d> projected;
    cv::projectPoints(corners_3d, rvec_gt, tvec_gt, camera_matrix, dist_coeffs, projected);

    std::vector<cv::Point2f> tag_corners_2f;
    for (const auto &pt : projected)
    {
      tag_corners_2f.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
    }
    marker_corners.push_back(tag_corners_2f);
    marker_ids.push_back(t.first);
  }

  // 1. Test Single-Tag detection for each individual tag
  for (size_t i = 0; i < marker_ids.size(); ++i)
  {
    std::vector<std::vector<cv::Point2f>> single_corners = {marker_corners[i]};
    std::vector<int> single_ids = {marker_ids[i]};
    cv::Mat rvec_est, tvec_est;
    int used_tags = 0;
    const bool ok = PoseSolver::estimate_board_pose(
        single_corners, single_ids, tag_configs, camera_matrix, dist_coeffs, rvec_est, tvec_est, used_tags);

    EXPECT_TRUE(ok);
    EXPECT_EQ(used_tags, 1);
    EXPECT_NEAR(tvec_est.at<double>(0), tvec_gt.at<double>(0), 1e-3);
    EXPECT_NEAR(tvec_est.at<double>(1), tvec_gt.at<double>(1), 1e-3);
    EXPECT_NEAR(tvec_est.at<double>(2), tvec_gt.at<double>(2), 1e-3);
  }

  // 2. Test Multi-Tag detection (all 4 tags)
  {
    cv::Mat rvec_est, tvec_est;
    int used_tags = 0;
    const bool ok = PoseSolver::estimate_board_pose(
        marker_corners, marker_ids, tag_configs, camera_matrix, dist_coeffs, rvec_est, tvec_est, used_tags);

    EXPECT_TRUE(ok);
    EXPECT_EQ(used_tags, 4);
    EXPECT_NEAR(tvec_est.at<double>(0), tvec_gt.at<double>(0), 1e-4);
    EXPECT_NEAR(tvec_est.at<double>(1), tvec_gt.at<double>(1), 1e-4);
    EXPECT_NEAR(tvec_est.at<double>(2), tvec_gt.at<double>(2), 1e-4);
  }
}

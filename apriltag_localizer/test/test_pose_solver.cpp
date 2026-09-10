/**
 * @file test_pose_solver.cpp
 * @brief Unit tests (L1 verification) for SE(3) Lie Algebra and per-tag disambiguated pose estimation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <cmath>
#include <vector>
#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "apriltag_localizer/pose_solver.hpp"

using namespace apriltag_localizer;

TEST(PoseSolverTest, TagCornersComputation)
{
  const cv::Point3d center(0.38, 0.0, 0.004);
  const double tag_size = 0.02; // 20mm
  const double h = tag_size / 2.0;
  const auto corners = PoseSolver::compute_tag_corners(center, tag_size, 0.0);

  ASSERT_EQ(corners.size(), 4U);

  // Check exact corner offsets for yaw=0: [h, -h], [-h, -h], [-h, h], [h, h]
  EXPECT_NEAR(corners[0].x, center.x + h, 1e-6);
  EXPECT_NEAR(corners[0].y, center.y - h, 1e-6);
  EXPECT_NEAR(corners[0].z, center.z, 1e-6);

  EXPECT_NEAR(corners[1].x, center.x - h, 1e-6);
  EXPECT_NEAR(corners[1].y, center.y - h, 1e-6);
  EXPECT_NEAR(corners[1].z, center.z, 1e-6);

  EXPECT_NEAR(corners[2].x, center.x - h, 1e-6);
  EXPECT_NEAR(corners[2].y, center.y + h, 1e-6);
  EXPECT_NEAR(corners[2].z, center.z, 1e-6);

  EXPECT_NEAR(corners[3].x, center.x + h, 1e-6);
  EXPECT_NEAR(corners[3].y, center.y + h, 1e-6);
  EXPECT_NEAR(corners[3].z, center.z, 1e-6);

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

// Helper fixture for multi-tag synthetic tests
class MultiTagPnPFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 600.0, 0.0, 320.0,
                      0.0, 600.0, 240.0,
                      0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    // Board layout matching real chessboard (positions in meters, Z=0.004m)
    const std::vector<std::pair<int, cv::Point3d>> tags = {
        {0, cv::Point3d(0.0000, 0.0000, 0.0040)},
        {1, cv::Point3d(0.3832, 0.0037, 0.0040)},
        {2, cv::Point3d(0.3772, 0.3885, 0.0040)},
        {3, cv::Point3d(-0.0039, 0.3850, 0.0040)}};

    for (const auto &t : tags)
    {
      TagConfig cfg;
      cfg.id = t.first;
      cfg.center = t.second;
      cfg.yaw = 0.0;
      cfg.corners_board = PoseSolver::compute_tag_corners(cfg.center, 0.02, cfg.yaw);
      tag_configs_[cfg.id] = cfg;
    }

    // Ground truth camera-to-board pose:
    // Camera is looking down towards the board at ~0.65m distance with slight tilt
    R_gt_ = (cv::Mat_<double>(3, 3) << 0.9950042, 0.0000000, 0.0998334,
             0.0000000, -1.0000000, 0.0000000,
             0.0998334, 0.0000000, -0.9950042);
    cv::Rodrigues(R_gt_, rvec_gt_);
    tvec_gt_ = (cv::Mat_<double>(3, 1) << -0.19, 0.19, 0.65);

    // Project corners into image for each tag
    for (const auto &t : tags)
    {
      const auto &corners_3d = tag_configs_[t.first].corners_board;
      std::vector<cv::Point2d> projected;
      cv::projectPoints(corners_3d, rvec_gt_, tvec_gt_, camera_matrix_, dist_coeffs_, projected);

      std::vector<cv::Point2f> tag_corners_2f;
      for (const auto &pt : projected)
      {
        tag_corners_2f.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
      }
      all_marker_corners_.push_back(tag_corners_2f);
      all_marker_ids_.push_back(t.first);
    }
  }

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::map<int, TagConfig> tag_configs_;
  cv::Mat R_gt_, rvec_gt_, tvec_gt_;
  std::vector<std::vector<cv::Point2f>> all_marker_corners_;
  std::vector<int> all_marker_ids_;
};

TEST_F(MultiTagPnPFixture, SingleTagStrictlyRejected)
{
  // Any single tag MUST be rejected (returns false) because Global Multi-Tag PnP requires >= 2 tags
  for (size_t i = 0; i < all_marker_ids_.size(); ++i)
  {
    std::vector<std::vector<cv::Point2f>> single_corner = {all_marker_corners_[i]};
    std::vector<int> single_id = {all_marker_ids_[i]};

    cv::Mat rvec_est, tvec_est;
    int used_tags = 0;
    const bool ok = PoseSolver::estimate_board_pose(
        single_corner, single_id, tag_configs_, camera_matrix_, dist_coeffs_,
        rvec_est, tvec_est, used_tags);

    EXPECT_FALSE(ok);
    EXPECT_LT(used_tags, 2);
  }
}

TEST_F(MultiTagPnPFixture, TwoTagsSucceedWithHighAccuracy)
{
  // Test all pairs of tags (0-1, 1-2, 2-3, 0-2, 0-3, 1-3)
  const std::vector<std::pair<int, int>> pairs = {
      {0, 1}, {1, 2}, {2, 3}, {0, 2}, {0, 3}, {1, 3}};

  for (const auto &p : pairs)
  {
    std::vector<std::vector<cv::Point2f>> pair_corners = {
        all_marker_corners_[p.first], all_marker_corners_[p.second]};
    std::vector<int> pair_ids = {
        all_marker_ids_[p.first], all_marker_ids_[p.second]};

    cv::Mat rvec_est, tvec_est;
    int used_tags = 0;
    const bool ok = PoseSolver::estimate_board_pose(
        pair_corners, pair_ids, tag_configs_, camera_matrix_, dist_coeffs_,
        rvec_est, tvec_est, used_tags);

    ASSERT_TRUE(ok) << "Failed for tag pair (" << p.first << ", " << p.second << ")";
    EXPECT_EQ(used_tags, 2);

    // Sub-millimeter translation accuracy
    EXPECT_NEAR(tvec_est.at<double>(0), tvec_gt_.at<double>(0), 1e-3);
    EXPECT_NEAR(tvec_est.at<double>(1), tvec_gt_.at<double>(1), 1e-3);
    EXPECT_NEAR(tvec_est.at<double>(2), tvec_gt_.at<double>(2), 1e-3);

    // Rotation accuracy
    cv::Mat R_est;
    cv::Rodrigues(rvec_est, R_est);
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        EXPECT_NEAR(R_est.at<double>(r, c), R_gt_.at<double>(r, c), 1e-3);
      }
    }
  }
}

TEST_F(MultiTagPnPFixture, FourTagsSucceedWithHighAccuracy)
{
  cv::Mat rvec_est, tvec_est;
  int used_tags = 0;
  const bool ok = PoseSolver::estimate_board_pose(
      all_marker_corners_, all_marker_ids_, tag_configs_, camera_matrix_, dist_coeffs_,
      rvec_est, tvec_est, used_tags);

  ASSERT_TRUE(ok);
  EXPECT_EQ(used_tags, 4);

  // Sub-millimeter translation accuracy
  EXPECT_NEAR(tvec_est.at<double>(0), tvec_gt_.at<double>(0), 5e-4);
  EXPECT_NEAR(tvec_est.at<double>(1), tvec_gt_.at<double>(1), 5e-4);
  EXPECT_NEAR(tvec_est.at<double>(2), tvec_gt_.at<double>(2), 5e-4);

  cv::Mat R_est;
  cv::Rodrigues(rvec_est, R_est);
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      EXPECT_NEAR(R_est.at<double>(r, c), R_gt_.at<double>(r, c), 5e-4);
    }
  }
}

TEST_F(MultiTagPnPFixture, RejectionAndDeterministicFailure)
{
  cv::Mat rvec, tvec;
  int used_tags = 0;

  // 1. Empty corners should fail-safe return false
  EXPECT_FALSE(PoseSolver::estimate_board_pose(
      {}, {}, tag_configs_, camera_matrix_, dist_coeffs_, rvec, tvec, used_tags));

  // 2. Unknown tag IDs should fail-safe return false
  std::vector<std::vector<cv::Point2f>> dummy_corners = {
      {cv::Point2f(10, 10), cv::Point2f(20, 10), cv::Point2f(20, 20), cv::Point2f(10, 20)},
      {cv::Point2f(50, 50), cv::Point2f(60, 50), cv::Point2f(60, 60), cv::Point2f(50, 60)}};
  EXPECT_FALSE(PoseSolver::estimate_board_pose(
      dummy_corners, {998, 999}, tag_configs_, camera_matrix_, dist_coeffs_, rvec, tvec, used_tags));

  // 3. Degenerate / collinear 2D image points should fail-safe return false
  std::vector<std::vector<cv::Point2f>> collinear_corners = {
      {cv::Point2f(100, 100), cv::Point2f(100, 100), cv::Point2f(100, 100), cv::Point2f(100, 100)},
      {cv::Point2f(100, 100), cv::Point2f(100, 100), cv::Point2f(100, 100), cv::Point2f(100, 100)}};
  EXPECT_FALSE(PoseSolver::estimate_board_pose(
      collinear_corners, {0, 1}, tag_configs_, camera_matrix_, dist_coeffs_, rvec, tvec, used_tags));
}

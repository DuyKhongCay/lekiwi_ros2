/**
 * @file test_chessboard_covariance.cpp
 * @brief Unit tests for ChessboardPoseEstimator velocity-dependent covariance logic.
 *
 * Verification Level: L1 (Unit tests, isolated logic, no ROS graph).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "chessboard_pose_estimator.hpp"

class ChessboardCovarianceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    estimator_ = std::make_shared<lekiwi_perception::ChessboardPoseEstimator>();
  }

  void TearDown() override
  {
    estimator_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<lekiwi_perception::ChessboardPoseEstimator> estimator_;
};

TEST_F(ChessboardCovarianceTest, BaselineAtRestWithFullTags)
{
  // When robot is at rest and 4 tags are detected, base variance should be returned.
  const auto res = estimator_->compute_covariance(0.0, 0.0, 4);

  EXPECT_NEAR(res.pos_var, 0.0001, 1e-6); // 1 cm std
  EXPECT_NEAR(res.rot_var, 0.0004, 1e-6); // 1.15 deg std
}

TEST_F(ChessboardCovarianceTest, LinearMotionScaling)
{
  // At speed = 0.5 m/s, pos_var = 0.0001 + 0.01 * (0.5)^2 = 0.0001 + 0.0025 = 0.0026
  // rot_var remains at baseline (0.0004)
  const auto res = estimator_->compute_covariance(0.5, 0.0, 4);

  EXPECT_NEAR(res.pos_var, 0.0026, 1e-6);
  EXPECT_NEAR(res.rot_var, 0.0004, 1e-6);
}

TEST_F(ChessboardCovarianceTest, AngularMotionClamping)
{
  // At wz = 1.0 rad/s (~57.3 deg/s), rot_var = 0.0004 + 0.05 * 1.0 = 0.0504
  // Clamped by max_rot_var = 0.04 (std ~11.46 deg)
  const auto res = estimator_->compute_covariance(0.0, 1.0, 4);

  EXPECT_NEAR(res.pos_var, 0.0001, 1e-6);
  EXPECT_NEAR(res.rot_var, 0.04, 1e-6);
}

TEST_F(ChessboardCovarianceTest, ModerateAngularMotion)
{
  // At wz = 0.5 rad/s (~28.6 deg/s), rot_var = 0.0004 + 0.05 * 0.25 = 0.0129
  // Below max_rot_var (0.04), so not clamped
  const auto res = estimator_->compute_covariance(0.0, 0.5, 4);

  EXPECT_NEAR(res.pos_var, 0.0001, 1e-6);
  EXPECT_NEAR(res.rot_var, 0.0129, 1e-6);
}

TEST_F(ChessboardCovarianceTest, TagCountScaling)
{
  // At rest:
  // 4 tags: tag_scale = 1.0 -> pos=0.0001, rot=0.0004
  // 3 tags: tag_scale = 2.0 -> pos=0.0002, rot=0.0008
  // 2 tags: tag_scale = 4.0 -> pos=0.0004, rot=0.0016
  const auto res4 = estimator_->compute_covariance(0.0, 0.0, 4);
  const auto res3 = estimator_->compute_covariance(0.0, 0.0, 3);
  const auto res2 = estimator_->compute_covariance(0.0, 0.0, 2);

  EXPECT_NEAR(res4.pos_var, 0.0001, 1e-6);
  EXPECT_NEAR(res4.rot_var, 0.0004, 1e-6);

  EXPECT_NEAR(res3.pos_var, 0.0002, 1e-6);
  EXPECT_NEAR(res3.rot_var, 0.0008, 1e-6);

  EXPECT_NEAR(res2.pos_var, 0.0004, 1e-6);
  EXPECT_NEAR(res2.rot_var, 0.0016, 1e-6);
}

TEST_F(ChessboardCovarianceTest, MaxBoundsClamping)
{
  // Extreme velocities should be clamped to max bounds
  const auto res = estimator_->compute_covariance(10.0, 10.0, 2);

  EXPECT_NEAR(res.pos_var, 0.01, 1e-6); // max_pos_var
  EXPECT_NEAR(res.rot_var, 0.04, 1e-6); // max_rot_var
}

TEST_F(ChessboardCovarianceTest, DefensiveNanInfHandling)
{
  // NaN or Inf inputs should not crash or produce invalid variance
  const double nan_val = std::numeric_limits<double>::quiet_NaN();
  const double inf_val = std::numeric_limits<double>::infinity();

  const auto res_nan = estimator_->compute_covariance(nan_val, nan_val, 4);
  EXPECT_NEAR(res_nan.pos_var, 0.0001, 1e-6);
  EXPECT_NEAR(res_nan.rot_var, 0.0004, 1e-6);

  const auto res_inf = estimator_->compute_covariance(inf_val, inf_val, 4);
  EXPECT_NEAR(res_inf.pos_var, 0.0001, 1e-6);
  EXPECT_NEAR(res_inf.rot_var, 0.0004, 1e-6);

  const auto res_neg = estimator_->compute_covariance(-1.0, -2.0, 4);
  // Negative speed treated as 0, negative wz uses abs(wz)
  EXPECT_NEAR(res_neg.pos_var, 0.0001, 1e-6);
  EXPECT_NEAR(res_neg.rot_var, 0.04, 1e-6); // wz=2.0 -> clamped to 0.04
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

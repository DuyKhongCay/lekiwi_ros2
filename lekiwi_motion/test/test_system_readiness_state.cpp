// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include "lekiwi_motion/system_readiness_state.hpp"

using namespace lekiwi_motion;

class SystemReadinessEvaluatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    evaluator_ = std::make_unique<SystemReadinessEvaluator>();

    // Chuẩn bị snapshot hợp lệ tiêu chuẩn ở t = 10.0s
    now_sec_ = 10.0;

    local_odom_.valid = true;
    local_odom_.stamp_sec = 9.95; // trễ 0.05s
    local_odom_.vx = 0.0;
    local_odom_.vy = 0.0;
    local_odom_.wz = 0.0;

    global_odom_.valid = true;
    global_odom_.stamp_sec = 9.95;
    global_odom_.pos_variance = 0.0005; // < 0.0012
    global_odom_.yaw_variance = 0.0010; // < 0.0030

    joints_.valid = true;
    joints_.stamp_sec = 9.95;
    joints_.joint_names = {
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"};

    tf_chains_.odom_to_base_fresh = true;
    tf_chains_.base_to_gripper_fresh = true;
    tf_chains_.map_to_board_fresh = true;
  }

  std::unique_ptr<SystemReadinessEvaluator> evaluator_;
  double now_sec_{10.0};
  LocalOdomSnapshot local_odom_;
  GlobalOdomSnapshot global_odom_;
  JointsSnapshot joints_;
  TfChainSnapshot tf_chains_;
};

TEST_F(SystemReadinessEvaluatorTest, FullReadinessWhenStationaryAndAllSignalsFresh)
{
  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.nav_ready);
  EXPECT_TRUE(report.grasp_ready);
  EXPECT_TRUE(report.is_stationary);
  EXPECT_EQ(report.grasp_blocker_reason, "Ready");
}

TEST_F(SystemReadinessEvaluatorTest, NavigationReadyWhileMovingGraspNotReady)
{
  // Xe đang chạy với vận tốc 0.2 m/s
  local_odom_.vx = 0.20;

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  // Navigation vẫn phải READY để xe tiếp tục di chuyển
  EXPECT_TRUE(report.nav_ready);
  EXPECT_FALSE(report.is_stationary);
  // Grasp KHÔNG được READY khi xe đang chuyển động
  EXPECT_FALSE(report.grasp_ready);
  EXPECT_NE(report.grasp_blocker_reason.find("Moving"), std::string::npos);
}

TEST_F(SystemReadinessEvaluatorTest, NavigationReadyWhenAprilTagLostTemporarily)
{
  // 1. Bước đầu: Global EKF hội tụ lúc xe đứng yên
  auto initial_report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(initial_report.grasp_ready);

  // 2. Sau đó: Xe di chuyển và mất dấu Tag (global odom stale > 0.3s)
  now_sec_ = 12.0;
  local_odom_.stamp_sec = 11.95;
  local_odom_.vx = 0.15;                 // đang chạy
  global_odom_.stamp_sec = 10.0;         // Tag đã mất 2 giây
  tf_chains_.map_to_board_fresh = false; // TF bàn cờ mất

  auto moving_report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  // Navigation VẪN READY nhờ Local Odom và seed trước đó
  EXPECT_TRUE(moving_report.nav_ready);
  // Grasp bị khóa vì mất Tag và xe đang chạy
  EXPECT_FALSE(moving_report.grasp_ready);
}

TEST_F(SystemReadinessEvaluatorTest, MissingJointBlocksGraspNotNavigation)
{
  // Thiếu khớp arm_gripper
  joints_.joint_names.erase("arm_gripper");

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  EXPECT_TRUE(report.nav_ready);
  EXPECT_FALSE(report.grasp_ready);
  EXPECT_NE(report.grasp_blocker_reason.find("Arm joints incomplete"), std::string::npos);
}

TEST_F(SystemReadinessEvaluatorTest, StaleLocalOdomBlocksNavigation)
{
  // Local odom quá cũ (> 0.5s)
  local_odom_.stamp_sec = now_sec_ - 1.0;

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  EXPECT_FALSE(report.nav_ready);
}

TEST_F(SystemReadinessEvaluatorTest, HighVarianceBlocksGrasp)
{
  // Phương sai EKF quá cao (0.005 > 0.0012)
  global_odom_.pos_variance = 0.0050;

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  EXPECT_FALSE(report.grasp_ready);
  EXPECT_NE(report.grasp_blocker_reason.find("Global EKF unconverged"), std::string::npos);
}

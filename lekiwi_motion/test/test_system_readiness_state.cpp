// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test_system_readiness_state.cpp
 * @brief Unit tests for SystemReadinessEvaluator pure domain logic.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include "lekiwi_motion/system_readiness_state.hpp"

using namespace lekiwi_motion;

/**
 * @class SystemReadinessEvaluatorTest
 * @brief Test fixture initializing baseline valid snapshots for readiness verification.
 */
class SystemReadinessEvaluatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    evaluator_ = std::make_unique<SystemReadinessEvaluator>();

    // Prepare standard valid snapshot at t = 10.0s
    now_sec_ = 10.0;

    local_odom_.valid = true;
    local_odom_.stamp_sec = 9.95; // 50ms latency
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

/**
 * @brief Verifies full readiness (nav and grasp) when stationary and all sensor streams are fresh.
 */
TEST_F(SystemReadinessEvaluatorTest, FullReadinessWhenStationaryAndAllSignalsFresh)
{
  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.nav_ready);
  EXPECT_TRUE(report.grasp_ready);
  EXPECT_TRUE(report.is_stationary);
  EXPECT_EQ(report.grasp_blocker_reason, "Ready");
}

/**
 * @brief Tests that platform motion allows navigation but inhibits precision grasping.
 */
TEST_F(SystemReadinessEvaluatorTest, NavigationReadyWhileMovingGraspNotReady)
{
  // Robot moving at 0.20 m/s linear velocity
  local_odom_.vx = 0.20;

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  // Navigation remains READY so robot can continue moving
  EXPECT_TRUE(report.nav_ready);
  EXPECT_FALSE(report.is_stationary);
  // Grasp must NOT be READY while platform is moving
  EXPECT_FALSE(report.grasp_ready);
  EXPECT_NE(report.grasp_blocker_reason.find("Moving"), std::string::npos);
}

/**
 * @brief Verifies navigation persists during temporary AprilTag occlusion once EKF was seeded.
 */
TEST_F(SystemReadinessEvaluatorTest, NavigationReadyWhenAprilTagLostTemporarily)
{
  // 1. Initial stage: Global EKF converges while robot is stationary
  auto initial_report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(initial_report.grasp_ready);

  // 2. Subsequent stage: Robot moves and loses AprilTag tracking (global odom stale > 0.3s)
  now_sec_ = 12.0;
  local_odom_.stamp_sec = 11.95;
  local_odom_.vx = 0.15;                 // robot in motion
  global_odom_.stamp_sec = 10.0;         // Tag lost for 2 seconds
  tf_chains_.map_to_board_fresh = false; // Board TF lost

  auto moving_report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  // Navigation remains READY due to local odom and prior global seed
  EXPECT_TRUE(moving_report.nav_ready);
  // Grasp blocked because tag is lost and robot is moving
  EXPECT_FALSE(moving_report.grasp_ready);
}

/**
 * @brief Verifies that missing required arm joints blocks grasp readiness without inhibiting nav.
 */
TEST_F(SystemReadinessEvaluatorTest, MissingJointBlocksGraspNotNavigation)
{
  // Missing arm_gripper joint
  joints_.joint_names.erase("arm_gripper");

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  EXPECT_TRUE(report.nav_ready);
  EXPECT_FALSE(report.grasp_ready);
  EXPECT_NE(report.grasp_blocker_reason.find("Arm joints incomplete"), std::string::npos);
}

/**
 * @brief Tests that stale local odometry drops navigation readiness.
 */
TEST_F(SystemReadinessEvaluatorTest, StaleLocalOdomBlocksNavigation)
{
  // Local odometry too stale (> 0.5s)
  local_odom_.stamp_sec = now_sec_ - 1.0;

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  EXPECT_FALSE(report.nav_ready);
}

/**
 * @brief Tests that excessive global position variance blocks precision grasping.
 */
TEST_F(SystemReadinessEvaluatorTest, HighVarianceBlocksGrasp)
{
  // EKF position variance exceeds threshold (0.005 > 0.0012)
  global_odom_.pos_variance = 0.0050;

  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);

  EXPECT_FALSE(report.grasp_ready);
  EXPECT_NE(report.grasp_blocker_reason.find("Global EKF unconverged"), std::string::npos);
}

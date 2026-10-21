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
#include "lekiwi_motion/supervision/system_readiness_state.hpp"

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

/**
 * @brief Tests that reset_global_ekf_seed resets seeding and stationary latch.
 */
TEST_F(SystemReadinessEvaluatorTest, ResetGlobalEkfSeedClearsReadiness)
{
  // 1. Initial convergence seeds the filter
  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.nav_ready);
  EXPECT_TRUE(evaluator_->is_global_ekf_seeded());

  // 2. Calling reset_global_ekf_seed unseeds the filter
  evaluator_->reset_global_ekf_seed();
  EXPECT_FALSE(evaluator_->is_global_ekf_seeded());

  // 3. Evaluation without new convergence must fail nav readiness if require_global_ekf_seed is true
  global_odom_.pos_variance = 0.050; // unconverged
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_FALSE(report.nav_ready);
  EXPECT_FALSE(report.grasp_ready);
}

/**
 * @brief Tests that transient angular spikes (< 0.25s) do not break stationary latch or grasp readiness.
 */
TEST_F(SystemReadinessEvaluatorTest, TransientAngularSpikePreservesStationaryLatch)
{
  // 1. Initial convergence seeds filter and establishes stationary latch
  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.grasp_ready);
  EXPECT_TRUE(report.stationary_latched);

  // 2. Camera switches to actor mode (AprilTag lost / variance rises)
  global_odom_.pos_variance = 0.050; // > 0.0012
  global_odom_.yaw_variance = 0.050; // > 0.0030

  // Verify latch holds grasp_ready when stationary
  now_sec_ = 10.10;
  local_odom_.stamp_sec = 10.08;
  joints_.stamp_sec = 10.08;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.grasp_ready);
  EXPECT_TRUE(report.stationary_latched);

  // 3. Arm movement induces angular reaction spike wz = 0.15 rad/s (> default limit 0.08)
  // First transient sample at t = 10.20s (elapsed 0.0s < 0.25s)
  now_sec_ = 10.20;
  local_odom_.stamp_sec = 10.19;
  joints_.stamp_sec = 10.19;
  local_odom_.wz = 0.15;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.is_stationary);
  EXPECT_TRUE(report.stationary_latched);
  EXPECT_TRUE(report.grasp_ready);

  // Second transient sample at t = 10.30s (elapsed 0.10s < 0.25s)
  now_sec_ = 10.30;
  local_odom_.stamp_sec = 10.29;
  joints_.stamp_sec = 10.29;
  local_odom_.wz = 0.15;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.is_stationary);
  EXPECT_TRUE(report.stationary_latched);
  EXPECT_TRUE(report.grasp_ready);

  // 4. Spike ceases, returning to complete rest at t = 10.35s
  now_sec_ = 10.35;
  local_odom_.stamp_sec = 10.34;
  joints_.stamp_sec = 10.34;
  local_odom_.wz = 0.0;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.is_stationary);
  EXPECT_TRUE(report.stationary_latched);
  EXPECT_TRUE(report.grasp_ready);
}

/**
 * @brief Tests that sustained motion (>= 0.25s) breaks stationary latch and drops grasp readiness.
 */
TEST_F(SystemReadinessEvaluatorTest, SustainedMotionBreaksStationaryLatch)
{
  // 1. Initial convergence seeds filter and latches stationary
  auto report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.grasp_ready);
  EXPECT_TRUE(report.stationary_latched);

  // 2. High variance state (AprilTag absent)
  global_odom_.pos_variance = 0.050;
  global_odom_.yaw_variance = 0.050;

  // 3. Sustained motion begins at t = 10.10s with vx = 0.10 m/s
  now_sec_ = 10.10;
  local_odom_.stamp_sec = 10.09;
  joints_.stamp_sec = 10.09;
  local_odom_.vx = 0.10;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.is_stationary); // First sample within debounce window

  // 4. Sustained motion exceeds debounce duration (t = 10.40s, elapsed = 0.30s >= 0.25s)
  now_sec_ = 10.40;
  local_odom_.stamp_sec = 10.39;
  joints_.stamp_sec = 10.39;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_FALSE(report.is_stationary);
  EXPECT_FALSE(report.stationary_latched);
  EXPECT_FALSE(report.grasp_ready);

  // 5. Even after coming to complete stop, latch remains broken until new EKF convergence
  now_sec_ = 10.50;
  local_odom_.stamp_sec = 10.49;
  joints_.stamp_sec = 10.49;
  local_odom_.vx = 0.0;
  report = evaluator_->evaluate(now_sec_, local_odom_, global_odom_, joints_, tf_chains_);
  EXPECT_TRUE(report.is_stationary);
  EXPECT_FALSE(report.stationary_latched);
  EXPECT_FALSE(report.grasp_ready);
}

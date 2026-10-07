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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "lekiwi_motion/arm/arm_motion_planner.hpp"

namespace lekiwi_motion::test
{

class ArmMotionPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    arm_joints_ = {
      "arm_shoulder_pan",
      "arm_shoulder_lift",
      "arm_elbow_flex",
      "arm_wrist_flex",
      "arm_wrist_roll",
      "arm_gripper"
    };
    GripperConfig gripper_cfg{1.50, 0.00};
    planner_ = std::make_unique<ArmMotionPlanner>(arm_joints_, gripper_cfg, NamedPosesConfig{}, 50.0);
  }

  std::vector<std::string> arm_joints_;
  std::unique_ptr<ArmMotionPlanner> planner_;
};

// ==============================================================================
// 1. Quintic Polynomial Trajectory Interpolation Tests
// ==============================================================================

TEST_F(ArmMotionPlannerTest, BasicTrajectoryGenerationAndC2Continuity)
{
  const std::vector<double> q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> q_target = {0.2, -0.3, 0.4, -0.1, 0.5, 1.5};
  const double duration_sec = 2.0;

  auto traj = planner_->plan_trajectory(q_start, q_target, duration_sec);

  EXPECT_EQ(traj.joint_names, arm_joints_);
  EXPECT_GT(traj.points.size(), 10UL);

  // Check first point (t=0)
  const auto & p0 = traj.points.front();
  EXPECT_EQ(p0.time_from_start.sec, 0);
  EXPECT_EQ(p0.time_from_start.nanosec, 0U);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(p0.positions[i], q_start[i], 1e-6);
    EXPECT_NEAR(p0.velocities[i], 0.0, 1e-6);
    EXPECT_NEAR(p0.accelerations[i], 0.0, 1e-6);
  }

  // Check last point (t=duration_sec)
  const auto & pf = traj.points.back();
  EXPECT_EQ(pf.time_from_start.sec, 2);
  EXPECT_EQ(pf.time_from_start.nanosec, 0U);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(pf.positions[i], q_target[i], 1e-6);
    EXPECT_NEAR(pf.velocities[i], 0.0, 1e-6);
    EXPECT_NEAR(pf.accelerations[i], 0.0, 1e-6);
  }

  // Check internal points monotonically advance time
  double prev_time = -1.0;
  for (const auto & pt : traj.points) {
    double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
    EXPECT_GE(t, prev_time);
    prev_time = t;
  }
}

TEST_F(ArmMotionPlannerTest, InvalidDurationThrows)
{
  const std::vector<double> q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> q_target = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

  EXPECT_THROW(planner_->plan_trajectory(q_start, q_target, 0.0), std::invalid_argument);
  EXPECT_THROW(planner_->plan_trajectory(q_start, q_target, -1.5), std::invalid_argument);
}

TEST_F(ArmMotionPlannerTest, DimensionMismatchThrows)
{
  const std::vector<double> q_start = {0.0, 0.0, 0.0};
  const std::vector<double> q_target = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

  EXPECT_THROW(planner_->plan_trajectory(q_start, q_target, 1.0), std::invalid_argument);

  const std::vector<double> empty_pos = {};
  EXPECT_THROW(planner_->plan_trajectory(empty_pos, empty_pos, 1.0), std::invalid_argument);
}

TEST_F(ArmMotionPlannerTest, CustomSamplingRate)
{
  const std::vector<double> q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> q_target = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  const double duration_sec = 1.0;
  const double rate_hz = 100.0;

  auto traj = planner_->plan_trajectory(q_start, q_target, duration_sec, rate_hz);
  EXPECT_EQ(traj.points.size(), 101UL);
}

// ==============================================================================
// 2. Gripper Clamping & Phase Target Generation Tests
// ==============================================================================

TEST_F(ArmMotionPlannerTest, GripperClampingBounds)
{
  EXPECT_DOUBLE_EQ(planner_->clamp_gripper(-0.5), 0.0);
  EXPECT_DOUBLE_EQ(planner_->clamp_gripper(0.0), 0.0);
  EXPECT_DOUBLE_EQ(planner_->clamp_gripper(0.75), 0.75);
  EXPECT_DOUBLE_EQ(planner_->clamp_gripper(1.50), 1.50);
  EXPECT_DOUBLE_EQ(planner_->clamp_gripper(2.50), 1.50);
}

TEST_F(ArmMotionPlannerTest, BuildPhaseTargetsGeneratesCompleteSequence)
{
  std::vector<double> pick_approach{0.1, 0.2, 0.3, 0.4, 0.5, 1.50};
  std::vector<double> pick_descend{0.1, 0.1, 0.3, 0.4, 0.5, 1.50};
  std::vector<double> place_approach{0.6, 0.7, 0.8, 0.9, 1.0, 0.00};
  std::vector<double> place_descend{0.6, 0.5, 0.8, 0.9, 1.0, 0.00};

  auto phases = planner_->build_phase_targets(
    pick_approach, pick_descend, place_approach, place_descend);

  for (const auto & phase_name : ArmMotionPlanner::kPickAndPlacePhases) {
    EXPECT_TRUE(phases.find(phase_name) != phases.end())
      << "Missing phase: " << phase_name;
  }

  // Gripper states & phase targets
  EXPECT_DOUBLE_EQ(phases.at("APPROACH_PICK").back(), 1.50);
  EXPECT_EQ(phases.at("APPROACH_PICK"), pick_approach);

  EXPECT_DOUBLE_EQ(phases.at("DESCEND_PICK").back(), 1.50);
  EXPECT_EQ(phases.at("DESCEND_PICK"), pick_descend);

  EXPECT_DOUBLE_EQ(phases.at("GRASP").back(), 0.00);
  EXPECT_DOUBLE_EQ(phases.at("GRASP")[0], pick_descend[0]);
  EXPECT_DOUBLE_EQ(phases.at("GRASP")[1], pick_descend[1]);

  EXPECT_DOUBLE_EQ(phases.at("LIFT").back(), 0.00);
  EXPECT_DOUBLE_EQ(phases.at("LIFT")[0], pick_approach[0]);
  EXPECT_DOUBLE_EQ(phases.at("LIFT")[1], pick_approach[1]);

  EXPECT_DOUBLE_EQ(phases.at("APPROACH_PLACE").back(), 0.00);
  EXPECT_DOUBLE_EQ(phases.at("APPROACH_PLACE")[0], place_approach[0]);

  EXPECT_DOUBLE_EQ(phases.at("DESCEND_PLACE").back(), 0.00);
  EXPECT_DOUBLE_EQ(phases.at("DESCEND_PLACE")[0], place_descend[0]);

  EXPECT_DOUBLE_EQ(phases.at("RELEASE").back(), 1.50);
  EXPECT_DOUBLE_EQ(phases.at("RELEASE")[0], place_descend[0]);

  // RETRACT is a dynamic phase: vertically ascending back to place_approach with open gripper
  EXPECT_DOUBLE_EQ(phases.at("RETRACT").back(), 1.50);
  EXPECT_DOUBLE_EQ(phases.at("RETRACT")[0], place_approach[0]);
  EXPECT_DOUBLE_EQ(phases.at("RETRACT")[1], place_approach[1]);
}

TEST_F(ArmMotionPlannerTest, BuildClearTargetsForLeftAndRightBins)
{
  NamedPosesConfig named_cfg;
  named_cfg.home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  named_cfg.stow = {0.0, -1.57, 1.57, 0.75, 0.0, 0.0};
  named_cfg.clear_left = {1.20, -0.60, 0.80, 0.0, 0.0, 0.0};
  named_cfg.clear_right = {-1.20, -0.60, 0.80, 0.0, 0.0, 0.0};
  planner_->set_named_poses(named_cfg);

  std::vector<double> pick_approach{0.1, 0.2, 0.3, 0.4, 0.5, 1.50};
  std::vector<double> pick_descend{0.1, 0.1, 0.3, 0.4, 0.5, 1.50};

  // Test LEFT bin
  auto left_phases = planner_->build_clear_targets(pick_approach, pick_descend, ClearBinSide::LEFT);
  for (const auto & phase_name : ArmMotionPlanner::kClearPhases) {
    EXPECT_TRUE(left_phases.find(phase_name) != left_phases.end())
      << "Missing clear phase: " << phase_name;
  }
  EXPECT_DOUBLE_EQ(left_phases.at("APPROACH_PICK")[1], 0.2);
  EXPECT_DOUBLE_EQ(left_phases.at("DESCEND_PICK")[1], 0.1);
  EXPECT_DOUBLE_EQ(left_phases.at("DROP_CLEAR")[0], 1.20);
  EXPECT_DOUBLE_EQ(left_phases.at("DROP_CLEAR").back(), 0.00); // closed holding piece
  EXPECT_DOUBLE_EQ(left_phases.at("RELEASE_CLEAR").back(), 1.50); // open releasing into bin
  EXPECT_DOUBLE_EQ(left_phases.at("RETRACT_CLEAR")[1], 0.2); // pick approach shoulder lift
  EXPECT_DOUBLE_EQ(left_phases.at("RETRACT_CLEAR").back(), 1.50); // open gripper

  // Test RIGHT bin
  auto right_phases = planner_->build_clear_targets(pick_approach, pick_descend, ClearBinSide::RIGHT);
  EXPECT_DOUBLE_EQ(right_phases.at("DROP_CLEAR")[0], -1.20);
  EXPECT_DOUBLE_EQ(right_phases.at("DROP_CLEAR").back(), 0.00);
  EXPECT_DOUBLE_EQ(right_phases.at("RELEASE_CLEAR").back(), 1.50);
}

TEST_F(ArmMotionPlannerTest, BuildPickTargetsIncludesTransitStowWithClosedGripper)
{
  NamedPosesConfig named_cfg;
  named_cfg.stow = {0.0, -1.57, 1.57, 0.75, 0.0, 0.0};
  planner_->set_named_poses(named_cfg);

  std::vector<double> pick_approach{0.1, 0.2, 0.3, 0.4, 0.5, 1.50};
  std::vector<double> pick_descend{0.1, 0.1, 0.3, 0.4, 0.5, 1.50};

  auto pick_phases = planner_->build_pick_targets(pick_approach, pick_descend);

  EXPECT_EQ(pick_phases.size(), 5u);
  for (const auto & phase_name : ArmMotionPlanner::kPickPhases) {
    EXPECT_TRUE(pick_phases.find(phase_name) != pick_phases.end())
      << "Missing pick phase: " << phase_name;
  }

  EXPECT_DOUBLE_EQ(pick_phases.at("APPROACH_PICK").back(), 1.50);
  EXPECT_DOUBLE_EQ(pick_phases.at("DESCEND_PICK").back(), 1.50);
  EXPECT_DOUBLE_EQ(pick_phases.at("GRASP").back(), 0.00);
  EXPECT_DOUBLE_EQ(pick_phases.at("LIFT").back(), 0.00);

  // RETRACT_STOW must tuck into stow with CLOSED gripper for secure base movement
  EXPECT_DOUBLE_EQ(pick_phases.at("RETRACT_STOW")[1], -1.57);
  EXPECT_DOUBLE_EQ(pick_phases.at("RETRACT_STOW").back(), 0.00);
}

TEST_F(ArmMotionPlannerTest, BuildPlaceTargetsIncludesVerticalRetractAndStow)
{
  NamedPosesConfig named_cfg;
  named_cfg.stow = {0.0, -1.57, 1.57, 0.75, 0.0, 0.0};
  planner_->set_named_poses(named_cfg);

  std::vector<double> place_approach{0.6, 0.7, 0.8, 0.9, 1.0, 0.00};
  std::vector<double> place_descend{0.6, 0.5, 0.8, 0.9, 1.0, 0.00};

  auto place_phases = planner_->build_place_targets(place_approach, place_descend);

  EXPECT_EQ(place_phases.size(), 5u);
  for (const auto & phase_name : ArmMotionPlanner::kPlacePhases) {
    EXPECT_TRUE(place_phases.find(phase_name) != place_phases.end())
      << "Missing place phase: " << phase_name;
  }

  // APPROACH_PLACE starts with closed gripper (carrying piece from stow)
  EXPECT_DOUBLE_EQ(place_phases.at("APPROACH_PLACE").back(), 0.00);
  EXPECT_DOUBLE_EQ(place_phases.at("DESCEND_PLACE").back(), 0.00);
  EXPECT_DOUBLE_EQ(place_phases.at("RELEASE").back(), 1.50);

  // RETRACT ascends vertically with open gripper
  EXPECT_DOUBLE_EQ(place_phases.at("RETRACT")[0], place_approach[0]);
  EXPECT_DOUBLE_EQ(place_phases.at("RETRACT").back(), 1.50);

  // RETRACT_STOW concludes in stow with OPEN gripper
  EXPECT_DOUBLE_EQ(place_phases.at("RETRACT_STOW")[1], -1.57);
  EXPECT_DOUBLE_EQ(place_phases.at("RETRACT_STOW").back(), 1.50);
}

TEST_F(ArmMotionPlannerTest, GetNamedPoseReturnsConfiguredLandmarks)
{
  NamedPosesConfig named_cfg;
  named_cfg.home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  named_cfg.stow = {0.0, -1.57, 1.57, 0.75, 0.0, 0.0};
  named_cfg.clear_left = {1.20, -0.60, 0.80, 0.0, 0.0, 0.0};
  named_cfg.clear_right = {-1.20, -0.60, 0.80, 0.0, 0.0, 0.0};
  planner_->set_named_poses(named_cfg);

  auto home = planner_->get_named_pose("home");
  ASSERT_TRUE(home.has_value());
  EXPECT_DOUBLE_EQ((*home)[0], 0.0);

  auto stow = planner_->get_named_pose("STOW"); // case-insensitive
  ASSERT_TRUE(stow.has_value());
  EXPECT_DOUBLE_EQ((*stow)[1], -1.57);

  auto clear_l = planner_->get_named_pose("clear_left");
  ASSERT_TRUE(clear_l.has_value());
  EXPECT_DOUBLE_EQ((*clear_l)[0], 1.20);

  auto clear_r = planner_->get_named_pose("clear_right");
  ASSERT_TRUE(clear_r.has_value());
  EXPECT_DOUBLE_EQ((*clear_r)[0], -1.20);

  auto unknown = planner_->get_named_pose("non_existent");
  EXPECT_FALSE(unknown.has_value());
}

// ==============================================================================
// 5. Velocity-Governed Trajectory Planning Tests
// ==============================================================================

TEST_F(ArmMotionPlannerTest, CalculateTrajectoryDurationSynchronizedAndClamped)
{
  const std::vector<double> q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> q_target = {1.0, 0.5, 0.2, 0.1, 0.0, 0.0};
  const double max_vel = 1.875;

  // Max delta is 1.0 (joint 0). For quintic polynomial, T = 1.875 * 1.0 / 1.875 = 1.0s
  const double expected_duration = 1.0;
  const double duration = planner_->calculate_trajectory_duration(q_start, q_target, max_vel);
  EXPECT_NEAR(duration, expected_duration, 1e-6);

  // When start equals target, duration clamps to min_duration_sec (default 0.1s)
  const double zero_delta_duration = planner_->calculate_trajectory_duration(q_start, q_start, max_vel);
  EXPECT_DOUBLE_EQ(zero_delta_duration, 0.1);

  // Custom min_duration_sec clamp
  const double custom_clamp = planner_->calculate_trajectory_duration(q_start, q_start, max_vel, 0.25);
  EXPECT_DOUBLE_EQ(custom_clamp, 0.25);

  // Invalid arguments validation
  EXPECT_THROW((void)planner_->calculate_trajectory_duration(q_start, q_target, 0.0), std::invalid_argument);
  EXPECT_THROW((void)planner_->calculate_trajectory_duration(q_start, q_target, -1.0), std::invalid_argument);
  EXPECT_THROW((void)planner_->calculate_trajectory_duration(q_start, q_target, max_vel, 0.0), std::invalid_argument);
  EXPECT_THROW((void)planner_->calculate_trajectory_duration(q_start, q_target, max_vel, -0.1), std::invalid_argument);

  const std::vector<double> q_short = {0.0, 0.0};
  EXPECT_THROW((void)planner_->calculate_trajectory_duration(q_short, q_target, max_vel), std::invalid_argument);
  EXPECT_THROW((void)planner_->calculate_trajectory_duration(q_start, q_short, max_vel), std::invalid_argument);
}

TEST_F(ArmMotionPlannerTest, PlanTrajectoryAtVelocityEnforcesMaxVelocityLimit)
{
  const std::vector<double> q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> q_target = {0.8, -1.2, 0.5, 0.3, -0.6, 1.5};
  const double max_velocity_rad_s = 1.2;

  auto traj = planner_->plan_trajectory_at_velocity(q_start, q_target, max_velocity_rad_s);

  EXPECT_EQ(traj.joint_names, arm_joints_);
  ASSERT_GE(traj.points.size(), 2UL);

  // Verify boundary conditions
  for (size_t i = 0; i < arm_joints_.size(); ++i) {
    EXPECT_NEAR(traj.points.front().positions[i], q_start[i], 1e-4);
    EXPECT_NEAR(traj.points.front().velocities[i], 0.0, 1e-4);
    EXPECT_NEAR(traj.points.front().accelerations[i], 0.0, 1e-4);

    EXPECT_NEAR(traj.points.back().positions[i], q_target[i], 1e-4);
    EXPECT_NEAR(traj.points.back().velocities[i], 0.0, 1e-4);
    EXPECT_NEAR(traj.points.back().accelerations[i], 0.0, 1e-4);
  }

  // Verify velocity limit is strictly respected across all interpolation points
  double observed_peak_vel = 0.0;
  for (const auto & point : traj.points) {
    for (double vel : point.velocities) {
      const double abs_v = std::abs(vel);
      observed_peak_vel = std::max(observed_peak_vel, abs_v);
      EXPECT_LE(abs_v, max_velocity_rad_s + 1e-4);
    }
  }

  // The governing joint (joint 5 with delta = 1.5) must reach close to max_velocity_rad_s at the peak
  EXPECT_NEAR(observed_peak_vel, max_velocity_rad_s, 0.05);

  // Invalid velocity argument throws
  EXPECT_THROW((void)planner_->plan_trajectory_at_velocity(q_start, q_target, 0.0), std::invalid_argument);
  EXPECT_THROW((void)planner_->plan_trajectory_at_velocity(q_start, q_target, -0.5), std::invalid_argument);
}

}  // namespace lekiwi_motion::test


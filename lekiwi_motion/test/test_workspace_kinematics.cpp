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
 * @file test_workspace_kinematics.cpp
 * @brief Unit tests for URDF extraction, analytical inverse kinematics, and multi-tier workspace planning.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <cmath>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>

#include "lekiwi_motion/workspace_kinematics.hpp"
#include "lekiwi_motion/workspace_planner.hpp"

namespace ws = lekiwi_motion::workspace;

/**
 * @class WorkspaceKinematicsTest
 * @brief Test fixture loading URDF model and configuring SO101 analytical solver and planner.
 */
class WorkspaceKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    urdf_ = urdf::parseURDFFile(WORKSPACE_URDF_PATH);
    ASSERT_TRUE(urdf_) << "Failed to parse test URDF: " << WORKSPACE_URDF_PATH;

    joint_names_ = {
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll"};

    std::string err;
    bool ok = ws::extract_kinematics_from_urdf(
        *urdf_, "base_footprint", "gripperframe", joint_names_, model_, err);
    ASSERT_TRUE(ok) << "URDF extraction failed: " << err;

    workspace_.half_w = 0.195;
    workspace_.half_h = 0.195;
    workspace_.edge_clearance = 0.147;
    workspace_.sample_step = 0.025;
    workspace_.max_samples = 257;

    solver_ = std::make_shared<ws::SO101AnalyticalSolver>(model_);
    planner_ = std::make_shared<ws::WorkspacePlanner>(
        std::make_shared<ws::KinematicsModel>(model_), solver_, workspace_);
  }

  urdf::ModelInterfaceSharedPtr urdf_;
  std::vector<std::string> joint_names_;
  ws::KinematicsModel model_;
  ws::WorkspaceConfig workspace_;
  std::shared_ptr<ws::SO101AnalyticalSolver> solver_;
  std::shared_ptr<ws::WorkspacePlanner> planner_;
};

/**
 * @brief Tests Contract 1: Valid URDF link dimension extraction and roundtrip FK/IK consistency.
 */
TEST_F(WorkspaceKinematicsTest, URDFExtractionAndAnalyticalIK)
{
  EXPECT_GT(solver_->link_lengths()[0], 0.08);
  EXPECT_LT(solver_->link_lengths()[0], 0.20);
  EXPECT_GT(solver_->link_lengths()[1], 0.08);
  EXPECT_LT(solver_->link_lengths()[1], 0.20);
  EXPECT_GT(model_.reach_bound, 0.25);
  EXPECT_GT(solver_->reach_bound(), 0.25);

  std::array<double, 5> test_joints{0.0, 0.3, -0.4, 0.1, 0.0};
  const Eigen::Isometry3d fk = ws::forward_kinematics(model_, test_joints);

  const double tx = fk.translation().x();
  const double ty = fk.translation().y();
  const double tz = fk.translation().z();

  const ws::IkResult ik = solver_->solve(tx, ty, tz, 0.0, 0.0);
  ASSERT_TRUE(ik.success) << "IK failed for FK reachable pose. Reason: " << ik.reason;

  const Eigen::Isometry3d fk_from_ik = ws::forward_kinematics(model_, ik.joints);
  const double pos_error = (fk_from_ik.translation() - fk.translation()).norm();
  EXPECT_LT(pos_error, 1e-3) << "IK solution translation residual exceeds 1mm";
}

/**
 * @brief Tests Contract 2 - Tier 0: Robot executes move in-place with zero navigation.
 */
TEST_F(WorkspaceKinematicsTest, PlannerTierZero_ZeroNav)
{
  ws::PlanningRequest req;
  req.pick = ws::Point3D{0.0, -0.10, 0.02};
  req.place = ws::Point3D{0.0, -0.05, 0.02};
  req.pitch = -M_PI_2;

  // Standoff at edge 0 (bottom edge facing +Y)
  ws::BasePose current{0.0, -workspace_.half_h - workspace_.edge_clearance, -0.004, M_PI_2};
  ws::PlanningContext ctx{current};

  const ws::PlanResult plan = planner_->plan(req, ctx, current.z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  EXPECT_EQ(plan.plan_type, ws::PLAN_ZERO_NAV);
  EXPECT_EQ(plan.evaluated_candidates, 1);
  EXPECT_DOUBLE_EQ(plan.pick_base.x, current.x);
  EXPECT_DOUBLE_EQ(plan.pick_base.y, current.y);
  EXPECT_TRUE(plan.place_joints.has_value());
}

/**
 * @brief Tests Contract 2 - Tier 1: Single Standoff Base reaches both Pick and Place squares.
 */
TEST_F(WorkspaceKinematicsTest, PlannerTierOne_SingleBase)
{
  ws::PlanningRequest req;
  req.pick = ws::Point3D{0.0, -0.10, 0.02};
  req.place = ws::Point3D{0.0, -0.05, 0.02};
  req.pitch = -M_PI_2;

  // No current base provided (or far away)
  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = planner_->plan(req, ctx, base_z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  EXPECT_EQ(plan.plan_type, ws::PLAN_SINGLE_BASE);
  EXPECT_DOUBLE_EQ(plan.pick_base.x, plan.place_base.x);
  EXPECT_DOUBLE_EQ(plan.pick_base.y, plan.place_base.y);
  EXPECT_DOUBLE_EQ(plan.pick_base.yaw, plan.place_base.yaw);
  EXPECT_TRUE(plan.place_joints.has_value());
}

/**
 * @brief Tests Contract 2 - Tier 2: Dual Standoff Bases for distant squares (e.g. A1 to H8).
 */
TEST_F(WorkspaceKinematicsTest, PlannerTierTwo_DualBase)
{
  ws::PlanningRequest req;
  // A1 corner to H8 corner (~45cm apart, exceeding single-base arm reach)
  req.pick = ws::Point3D{-0.16, -0.16, 0.02};
  req.place = ws::Point3D{0.16, 0.16, 0.02};
  req.pitch = -M_PI_2;

  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = planner_->plan(req, ctx, base_z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  EXPECT_EQ(plan.plan_type, ws::PLAN_DUAL_BASE);
  // Pick base and place base must be distinct standoff locations
  EXPECT_NE(plan.pick_base.x, plan.place_base.x);
  EXPECT_TRUE(plan.place_joints.has_value());
}

/**
 * @brief Tests Contract 2 - Capture Tier 0: In-place capture execution with zero navigation.
 */
TEST_F(WorkspaceKinematicsTest, PlannerCapture_TierZero)
{
  ws::PlanningRequest req;
  req.clear = ws::Point3D{0.0, -0.05, 0.02};
  req.pick = ws::Point3D{0.0, -0.10, 0.02};
  req.place = ws::Point3D{0.0, -0.05, 0.02};
  req.pitch = -M_PI_2;
  req.is_capture = true;

  ws::BasePose current{0.0, -workspace_.half_h - workspace_.edge_clearance, -0.004, M_PI_2};
  ws::PlanningContext ctx{current};

  const ws::PlanResult plan = planner_->plan(req, ctx, current.z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  EXPECT_EQ(plan.plan_type, ws::PLAN_CAPTURE_ZERO_NAV);
  EXPECT_TRUE(plan.clear_joints.has_value());
  EXPECT_TRUE(plan.place_joints.has_value());
  EXPECT_DOUBLE_EQ(plan.clear_base.x, current.x);
  EXPECT_DOUBLE_EQ(plan.pick_base.x, current.x);
}

/**
 * @brief Tests Contract 2 - Capture Tier 1: Single Standoff Base reaches Clear, Pick, and Place.
 */
TEST_F(WorkspaceKinematicsTest, PlannerCapture_SingleBase)
{
  ws::PlanningRequest req;
  req.clear = ws::Point3D{0.0, -0.05, 0.02};
  req.pick = ws::Point3D{0.0, -0.10, 0.02};
  req.place = ws::Point3D{0.0, -0.05, 0.02};
  req.pitch = -M_PI_2;
  req.is_capture = true;

  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = planner_->plan(req, ctx, base_z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  EXPECT_EQ(plan.plan_type, ws::PLAN_CAPTURE_SINGLE_BASE);
  EXPECT_TRUE(plan.clear_joints.has_value());
  EXPECT_TRUE(plan.place_joints.has_value());
  EXPECT_DOUBLE_EQ(plan.clear_base.x, plan.pick_base.x);
  EXPECT_DOUBLE_EQ(plan.clear_base.y, plan.pick_base.y);
  EXPECT_DOUBLE_EQ(plan.pick_base.x, plan.place_base.x);
}

/**
 * @brief Tests Contract 2 - Capture Tier 3: Distant diagonal capture requiring multiple standoff bases.
 */
TEST_F(WorkspaceKinematicsTest, PlannerCapture_TripleBase_DistantDiagonal)
{
  ws::PlanningRequest req;
  // Pick at A1 corner, Clear and Place at H8 corner (~45cm apart)
  req.pick = ws::Point3D{-0.16, -0.16, 0.02};
  req.place = ws::Point3D{0.16, 0.16, 0.02};
  req.clear = req.place;
  req.pitch = -M_PI_2;
  req.is_capture = true;

  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = planner_->plan(req, ctx, base_z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  EXPECT_EQ(plan.plan_type, ws::PLAN_CAPTURE_TRIPLE_BASE);
  EXPECT_TRUE(plan.clear_joints.has_value());
  EXPECT_TRUE(plan.place_joints.has_value());
  // Pick base must differ from Clear/Place base
  EXPECT_NE(plan.pick_base.x, plan.clear_base.x);
  // Normal capture: Clear base and Place base are identical
  EXPECT_DOUBLE_EQ(plan.clear_base.x, plan.place_base.x);
  EXPECT_DOUBLE_EQ(plan.clear_base.y, plan.place_base.y);
}

/**
 * @brief Tests Contract 2 - Infeasible query: Out-of-reach target fails gracefully with BASE_STANDOFF_EXHAUSTED.
 */
TEST_F(WorkspaceKinematicsTest, PlannerInfeasible)
{
  ws::PlanningRequest req;
  req.pick = ws::Point3D{10.0, 10.0, 0.0};
  req.place = ws::Point3D{10.0, 10.1, 0.0};
  req.pitch = -M_PI_2;

  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = planner_->plan(req, ctx, base_z);
  EXPECT_FALSE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::BASE_STANDOFF_EXHAUSTED);
  EXPECT_NE(plan.message.find("No feasible"), std::string::npos);
}

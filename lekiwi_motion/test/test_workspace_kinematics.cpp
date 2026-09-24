// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>

#include "lekiwi_motion/workspace_kinematics.hpp"
#include "lekiwi_motion/workspace_planner.hpp"

namespace ws = lekiwi_motion::workspace;

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

// Contract 1: URDF extracts valid link dimensions and Analytical IK matches Forward Kinematics
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

// Contract 2 - Tier 0: Robot currently in place can execute move with ZERO base motion
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

// Contract 2 - Tier 1: Single Standoff Base reaches both Pick and Place
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

// Contract 2 - Tier 2: Dual Standoff Bases for diametrically opposite squares (e.g. A1 to H8)
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

// Contract 2 - Capture Tier 0: Robot currently in place can execute capture with ZERO base motion
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

// Contract 2 - Capture Tier 1: Single Standoff Base covers Clear, Pick, and Place
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

// Contract 2 - Capture Tier 3: Diametrically opposite squares (e.g. A1 takes H8)
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

// Contract 2 - Capture: En Passant where Clear square differs from Place square
TEST_F(WorkspaceKinematicsTest, PlannerCapture_EnPassant)
{
  ws::PlanningRequest req;
  // White pawn on e5 (-0.025, 0.025), captures black pawn on d5 (-0.05, 0.025), lands on d6 (-0.05, 0.05)
  req.pick = ws::Point3D{-0.025, 0.025, 0.02};
  req.clear = ws::Point3D{-0.05, 0.025, 0.02};
  req.place = ws::Point3D{-0.05, 0.05, 0.02};
  req.pitch = -M_PI_2;
  req.is_capture = true;

  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = planner_->plan(req, ctx, base_z);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.status, ws::FeasibilityStatus::SUCCESS);
  // All three squares are adjacent, so single base easily reaches all three
  EXPECT_EQ(plan.plan_type, ws::PLAN_CAPTURE_SINGLE_BASE);
  EXPECT_TRUE(plan.clear_joints.has_value());
  EXPECT_TRUE(plan.place_joints.has_value());
}

// Contract 2 - Infeasible: Target outside board reach
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

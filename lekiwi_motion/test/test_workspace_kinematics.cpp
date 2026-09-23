// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>

#include "lekiwi_control/workspace_kinematics.hpp"

namespace ws = lekiwi_control::workspace;

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
  }

  urdf::ModelInterfaceSharedPtr urdf_;
  std::vector<std::string> joint_names_;
  ws::KinematicsModel model_;
  ws::WorkspaceConfig workspace_;
};

// Contract 1: URDF extracts valid link dimensions and Analytical IK matches Forward Kinematics
TEST_F(WorkspaceKinematicsTest, URDFExtractionAndAnalyticalIK)
{
  EXPECT_GT(model_.link_lengths[0], 0.08);
  EXPECT_LT(model_.link_lengths[0], 0.20);
  EXPECT_GT(model_.link_lengths[1], 0.08);
  EXPECT_LT(model_.link_lengths[1], 0.20);
  EXPECT_GT(model_.reach_bound, 0.25);

  std::array<double, 5> test_joints{0.0, 0.3, -0.4, 0.1, 0.0};
  const Eigen::Isometry3d fk = ws::forward_kinematics(model_, test_joints);

  const double tx = fk.translation().x();
  const double ty = fk.translation().y();
  const double tz = fk.translation().z();

  const ws::IkResult ik = ws::solve_analytical_ik(tx, ty, tz, model_, 0.0, 0.0);
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

  const ws::PlanResult plan = ws::plan_move(req, ctx, current.z, model_, workspace_);
  EXPECT_TRUE(plan.feasible);
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

  const ws::PlanResult plan = ws::plan_move(req, ctx, base_z, model_, workspace_);
  EXPECT_TRUE(plan.feasible);
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

  const ws::PlanResult plan = ws::plan_move(req, ctx, base_z, model_, workspace_);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.plan_type, ws::PLAN_DUAL_BASE);
  // Pick base and place base must be distinct standoff locations
  EXPECT_NE(plan.pick_base.x, plan.place_base.x);
  EXPECT_TRUE(plan.place_joints.has_value());
}

// Contract 2 - Capture: Evaluates only Pick position, onboard drop is ignored
TEST_F(WorkspaceKinematicsTest, PlannerCapture_PickOnly)
{
  ws::PlanningRequest req;
  req.pick = ws::Point3D{0.0, -0.10, 0.02};
  req.pitch = -M_PI_2;
  req.is_capture = true;

  ws::PlanningContext ctx;
  const double base_z = -0.004;

  const ws::PlanResult plan = ws::plan_move(req, ctx, base_z, model_, workspace_);
  EXPECT_TRUE(plan.feasible);
  EXPECT_EQ(plan.plan_type, ws::PLAN_SINGLE_BASE);
  EXPECT_FALSE(plan.place_joints.has_value());
  EXPECT_NE(plan.message.find("Capture"), std::string::npos);
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

  const ws::PlanResult plan = ws::plan_move(req, ctx, base_z, model_, workspace_);
  EXPECT_FALSE(plan.feasible);
  EXPECT_NE(plan.message.find("No feasible"), std::string::npos);
}

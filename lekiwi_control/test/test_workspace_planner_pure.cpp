// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>

#include "lekiwi_control/workspace/kinematics_engine.hpp"
#include "lekiwi_control/workspace/types.hpp"
#include "lekiwi_control/workspace/urdf_kinematics_loader.hpp"
#include "lekiwi_control/workspace/workspace_planner.hpp"

namespace ws = lekiwi_control::workspace;

class WorkspacePlannerPureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    arm_ = ws::get_default_lekiwi_kinematics();
    workspace_.half_w = 0.195;
    workspace_.half_h = 0.195;
    workspace_.edge_clearance = 0.085;
    workspace_.sample_step = 0.025;
    workspace_.max_samples = 25;
    workspace_.default_pitch = -1.57079632679;
    workspace_.default_roll = 0.0;
  }

  ws::KinematicsConfig arm_;
  ws::WorkspaceConfig workspace_;
};

TEST_F(WorkspacePlannerPureTest, RejectsNonfiniteInputs)
{
  double nan_val = std::numeric_limits<double>::quiet_NaN();
  auto ik = ws::solve_analytical_ik(
      nan_val, 0.0, 0.04, arm_, workspace_.default_pitch, workspace_.default_roll);
  EXPECT_FALSE(ik.success);
  EXPECT_EQ(ik.reason, "IK input must be finite");

  ws::PlanningRequest req;
  req.pick = {nan_val, 0.0, 0.04};
  req.place = {0.1, 0.0, 0.04};
  req.pitch = workspace_.default_pitch;
  req.is_capture = false;

  auto plan = ws::plan_move(req, ws::PlanningContext{}, -0.004, arm_, workspace_);
  EXPECT_FALSE(plan.feasible);
  EXPECT_NE(plan.message.find("nonfinite"), std::string::npos);
}

TEST_F(WorkspacePlannerPureTest, IkFkRoundtripAgreement)
{
  std::vector<std::array<double, 3>> test_targets = {
      {0.22, 0.0, 0.04},
      {0.24, -0.03, 0.05}};

  for (const auto &target : test_targets)
  {
    auto ik = ws::solve_analytical_ik(
        target[0], target[1], target[2], arm_, workspace_.default_pitch, 0.0);
    ASSERT_TRUE(ik.success) << "Failed for target: " << ik.reason;

    auto [fk_x, fk_y, fk_z, fk_pitch] = ws::forward_kinematics_2d(
        ik.joints[0], ik.joints[1], ik.joints[2], ik.joints[3], arm_);

    EXPECT_NEAR(fk_x, target[0], 1e-4);
    EXPECT_NEAR(fk_y, target[1], 1e-4);
    EXPECT_NEAR(fk_z, target[2], 1e-4);
    EXPECT_NEAR(fk_pitch, workspace_.default_pitch, 1e-4);

    auto [valid, reason] = ws::check_joint_limits(ik.joints, arm_);
    EXPECT_TRUE(valid) << reason;
  }
}

TEST_F(WorkspacePlannerPureTest, StandoffPoseSelection)
{
  double base_z = -0.004;
  constexpr double nominal_reach = 0.245;
  double dist_edge = workspace_.half_h - 0.145; // 0.050
  double expected_standoff = std::max(workspace_.edge_clearance, nominal_reach - dist_edge);

  auto [base_south, edge_south] = ws::compute_standoff_pose(0.0, -0.145, base_z, workspace_);
  EXPECT_EQ(edge_south, "SOUTH");
  EXPECT_NEAR(base_south.y, -workspace_.half_h - expected_standoff, 1e-4);
  EXPECT_NEAR(base_south.z, base_z, 1e-5);
  EXPECT_NEAR(base_south.yaw, M_PI_2, 1e-5);

  auto [base_north, edge_north] = ws::compute_standoff_pose(0.0, 0.145, base_z, workspace_);
  EXPECT_EQ(edge_north, "NORTH");
  EXPECT_NEAR(base_north.y, workspace_.half_h + expected_standoff, 1e-4);

  auto [base_west, edge_west] = ws::compute_standoff_pose(-0.145, 0.0, base_z, workspace_);
  EXPECT_EQ(edge_west, "WEST");
  EXPECT_NEAR(base_west.x, -workspace_.half_w - expected_standoff, 1e-4);

  auto [base_east, edge_east] = ws::compute_standoff_pose(0.145, 0.0, base_z, workspace_);
  EXPECT_EQ(edge_east, "EAST");
  EXPECT_NEAR(base_east.x, workspace_.half_w + expected_standoff, 1e-4);

  // Center target: standoff is clamped to min edge_clearance
  auto [base_center, edge_center] = ws::compute_standoff_pose(0.0, 0.0, base_z, workspace_);
  EXPECT_NEAR(std::abs(base_center.y), workspace_.half_h + workspace_.edge_clearance, 1e-4);
}

TEST_F(WorkspacePlannerPureTest, TierSelectionZeroNavAndSingleBase)
{
  double base_z = -0.004;

  // 1. ZERO_NAV
  ws::PlanningRequest req_zero;
  req_zero.pick = {0.22, 0.0, 0.04};
  req_zero.place = {0.24, 0.0, 0.04};
  req_zero.pitch = workspace_.default_pitch;
  req_zero.is_capture = false;

  ws::PlanningContext ctx_zero;
  ctx_zero.current_base = ws::BasePose{0.0, 0.0, base_z, 0.0};

  auto plan_zero = ws::plan_move(req_zero, ctx_zero, base_z, arm_, workspace_);
  EXPECT_TRUE(plan_zero.feasible);
  EXPECT_EQ(plan_zero.plan_type, ws::PLAN_ZERO_NAV);

  // 2. SINGLE_BASE
  ws::PlanningRequest req_single;
  req_single.pick = {0.0, -0.18, 0.04};
  req_single.place = {0.01, -0.18, 0.04};
  req_single.pitch = workspace_.default_pitch;
  req_single.is_capture = false;

  auto plan_single = ws::plan_move(req_single, ws::PlanningContext{}, base_z, arm_, workspace_);
  EXPECT_TRUE(plan_single.feasible);
  EXPECT_EQ(plan_single.plan_type, ws::PLAN_SINGLE_BASE);

  // 3. CAPTURE
  ws::PlanningRequest req_cap;
  req_cap.pick = {0.0, -0.145, 0.04};
  req_cap.pitch = workspace_.default_pitch;
  req_cap.is_capture = true;

  auto plan_cap = ws::plan_move(req_cap, ws::PlanningContext{}, base_z, arm_, workspace_);
  EXPECT_TRUE(plan_cap.feasible);
  EXPECT_EQ(plan_cap.plan_type, ws::PLAN_SINGLE_BASE);
  EXPECT_FALSE(plan_cap.place_joints.has_value());
}

TEST_F(WorkspacePlannerPureTest, UrdfModelExtraction)
{
  std::string urdf_path = "/root/docker_ws/lekiwi_ros2/lekiwi_description/urdf/duykhongcay_lekiwi.urdf";
  auto model = urdf::parseURDFFile(urdf_path);
  ASSERT_TRUE(model != nullptr) << "Failed to parse URDF file: " << urdf_path;

  ws::KinematicsConfig extracted = ws::extract_kinematics_from_urdf(*model);
  EXPECT_TRUE(extracted.is_valid());

  // Link lengths from URDF
  EXPECT_NEAR(extracted.link_lengths[0], 0.115998, 1e-3);
  EXPECT_NEAR(extracted.link_lengths[1], 0.135000, 1e-3);
  EXPECT_NEAR(extracted.link_lengths[2], 0.099802, 1e-3);

  // Base offset from shoulder pan joint
  EXPECT_NEAR(extracted.base_offset[0], 0.0461807, 1e-3);
  EXPECT_NEAR(extracted.base_offset[1], 0.0, 1e-3);
  EXPECT_NEAR(extracted.base_offset[2], 0.1696, 1e-3);

  // Joint limits within URDF hard limits
  for (size_t i = 0; i < extracted.joint_names.size(); ++i)
  {
    auto joint = model->getJoint(extracted.joint_names[i]);
    ASSERT_TRUE(joint && joint->limits);
    EXPECT_GE(extracted.lower_limits[i], joint->limits->lower);
    EXPECT_LE(extracted.upper_limits[i], joint->limits->upper);
  }
}

TEST_F(WorkspacePlannerPureTest, CheckUserMoveWithEdgeClearance)
{
  double base_z = -0.004;
  ws::PlanningRequest req;
  req.pick = {0.05, -0.10, 0.02};
  req.place = {0.05, 0.00, 0.02};
  req.pitch = workspace_.default_pitch;
  req.is_capture = false;

  auto plan = ws::plan_move(req, ws::PlanningContext{}, base_z, arm_, workspace_);
  EXPECT_TRUE(plan.feasible) << "Plan failed: " << plan.message;
  EXPECT_GT(plan.plan_type, 0);
}

TEST_F(WorkspacePlannerPureTest, FullChessboard64SquaresReachable)
{
  double base_z = -0.004;
  // A standard 39cm chessboard has 8 files (a-h) and 8 ranks (1-8).
  // Center is (0,0), square spacing is approximately 0.045m (4.5cm).
  // Checking all 64 centers from -0.1575m to +0.1575m in steps of 0.045m
  std::vector<double> coords;
  for (int i = 0; i < 8; ++i)
  {
    coords.push_back(-0.1575 + i * 0.045);
  }

  int reachable_count = 0;
  for (double x : coords)
  {
    for (double y : coords)
    {
      auto [base, edge] = ws::compute_standoff_pose(x, y, base_z, workspace_);
      ws::Point3D target{x, y, 0.02};
      auto ik = ws::solve_at_base(target, base, workspace_.default_pitch, arm_, workspace_);
      if (ik.success)
      {
        reachable_count++;
      }
      else
      {
        ADD_FAILURE() << "Square at (" << x << ", " << y << ") unreachable from " << edge
                      << " edge: " << ik.reason;
      }
    }
  }
  EXPECT_EQ(reachable_count, 64);
}


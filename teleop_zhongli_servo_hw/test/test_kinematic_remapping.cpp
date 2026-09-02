/**
 * @file test_kinematic_remapping.cpp
 * @brief Unit tests for KinematicRemapper (Direct, Min-Max, Centered-Scale, Scale-Offset).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "teleop_zhongli_servo_hw/kinematic_remapper.hpp"

using namespace teleop_zhongli_servo_hw;

TEST(TestKinematicRemapper, DirectRemapAndClamping)
{
  JointRemapConfig cfg;
  cfg.joint_name = "arm_shoulder_pan";
  cfg.mode = RemapMode::DIRECT;
  cfg.follower_min = -1.57;
  cfg.follower_max = 1.57;

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure({cfg}));
  EXPECT_EQ(remapper.size(), 1u);

  // 1. Within range
  EXPECT_NEAR(remapper.remap_joint(0, 0.0), 0.0, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, 1.0), 1.0, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, -1.0), -1.0, 1e-6);

  // 2. Exceeding upper follower bound
  EXPECT_NEAR(remapper.remap_joint(0, 2.5), 1.57, 1e-6);

  // 3. Exceeding lower follower bound
  EXPECT_NEAR(remapper.remap_joint(0, -2.5), -1.57, 1e-6);
}

TEST(TestKinematicRemapper, MinMaxGripperMapping)
{
  // uArm leader gripper (0 ~ 45 deg, 0.0 to 0.785398 rad)
  // LeKiwi follower gripper (0 ~ 90 deg, 0.0 to 1.570796 rad)
  JointRemapConfig cfg;
  cfg.joint_name = "arm_gripper";
  cfg.mode = RemapMode::MIN_MAX;
  cfg.leader_min = 0.0;
  cfg.leader_max = 0.785398;
  cfg.follower_min = 0.0;
  cfg.follower_max = 1.570796;

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure({cfg}));

  // 0% leader -> 0% follower
  EXPECT_NEAR(remapper.remap_joint(0, 0.0), 0.0, 1e-5);

  // 100% leader -> 100% follower
  EXPECT_NEAR(remapper.remap_joint(0, 0.785398), 1.570796, 1e-5);

  // 50% leader -> 50% follower
  const double mid_leader = 0.5 * (cfg.leader_min + cfg.leader_max);
  const double mid_follower = 0.5 * (cfg.follower_min + cfg.follower_max);
  EXPECT_NEAR(remapper.remap_joint(0, mid_leader), mid_follower, 1e-5);

  // Clamped bounds for inputs outside leader range
  EXPECT_NEAR(remapper.remap_joint(0, -0.5), 0.0, 1e-5);
  EXPECT_NEAR(remapper.remap_joint(0, 1.5), 1.570796, 1e-5);
}

TEST(TestKinematicRemapper, MinMaxInvertedRange)
{
  // Leader: 0.0 -> 1.0, Follower: 2.0 -> 0.0 (inverted)
  JointRemapConfig cfg;
  cfg.joint_name = "inverted_joint";
  cfg.mode = RemapMode::MIN_MAX;
  cfg.leader_min = 0.0;
  cfg.leader_max = 1.0;
  cfg.follower_min = 2.0;
  cfg.follower_max = 0.0;

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure({cfg}));

  EXPECT_NEAR(remapper.remap_joint(0, 0.0), 2.0, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, 1.0), 0.0, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, 0.5), 1.0, 1e-6);
}

TEST(TestKinematicRemapper, MinMaxZeroSpanDivisionSafe)
{
  // Degenerate case: leader_min == leader_max
  JointRemapConfig cfg;
  cfg.joint_name = "degenerate_joint";
  cfg.mode = RemapMode::MIN_MAX;
  cfg.leader_min = 1.0;
  cfg.leader_max = 1.0;
  cfg.follower_min = -0.5;
  cfg.follower_max = 0.5;

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure({cfg}));

  // Should not divide by zero or crash
  EXPECT_NEAR(remapper.remap_joint(0, 1.0), -0.5, 1e-6);
}

TEST(TestKinematicRemapper, CenteredScaleZeroAnchor)
{
  // Asymmetric spans:
  // Leader: [-1.2, +1.2]
  // Follower: [-2.4, +1.8]
  JointRemapConfig cfg;
  cfg.joint_name = "arm_shoulder_pan";
  cfg.mode = RemapMode::CENTERED_SCALE;
  cfg.leader_min = -1.2;
  cfg.leader_max = 1.2;
  cfg.follower_min = -2.4;
  cfg.follower_max = 1.8;

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure({cfg}));

  // Point 0.0 rad MUST stay 0.0 rad (Home anchor preserved)
  EXPECT_NEAR(remapper.remap_joint(0, 0.0), 0.0, 1e-6);

  // Positive branch: scale = 1.8 / 1.2 = 1.5
  EXPECT_NEAR(remapper.remap_joint(0, 1.2), 1.8, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, 0.6), 0.9, 1e-6);

  // Negative branch: scale = |-2.4| / |-1.2| = 2.0
  EXPECT_NEAR(remapper.remap_joint(0, -1.2), -2.4, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, -0.6), -1.2, 1e-6);

  // Clamp checks
  EXPECT_NEAR(remapper.remap_joint(0, 2.0), 1.8, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, -3.0), -2.4, 1e-6);
}

TEST(TestKinematicRemapper, ScaleOffsetLinearEquation)
{
  // theta_F = k * theta_L + b
  JointRemapConfig cfg;
  cfg.joint_name = "arm_shoulder_lift";
  cfg.mode = RemapMode::SCALE_OFFSET;
  cfg.scale = 1.25;
  cfg.offset = 0.05;
  cfg.follower_min = -3.14;
  cfg.follower_max = 3.14;

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure({cfg}));

  // theta_L = 0.0 => 0.05
  EXPECT_NEAR(remapper.remap_joint(0, 0.0), 0.05, 1e-6);

  // theta_L = 1.0 => 1.25 * 1.0 + 0.05 = 1.30
  EXPECT_NEAR(remapper.remap_joint(0, 1.0), 1.30, 1e-6);

  // theta_L = -1.0 => 1.25 * (-1.0) + 0.05 = -1.20
  EXPECT_NEAR(remapper.remap_joint(0, -1.0), -1.20, 1e-6);

  // Clamp checks
  EXPECT_NEAR(remapper.remap_joint(0, 10.0), 3.14, 1e-6);
  EXPECT_NEAR(remapper.remap_joint(0, -10.0), -3.14, 1e-6);
}

TEST(TestKinematicRemapper, MultiJointRemapAll)
{
  std::vector<JointRemapConfig> configs;

  // Joint 0: Direct
  JointRemapConfig j0;
  j0.joint_name = "j0_direct";
  j0.mode = RemapMode::DIRECT;
  j0.follower_min = -1.0;
  j0.follower_max = 1.0;
  configs.push_back(j0);

  // Joint 1: Scale Offset
  JointRemapConfig j1;
  j1.joint_name = "j1_scale_offset";
  j1.mode = RemapMode::SCALE_OFFSET;
  j1.scale = 2.0;
  j1.offset = 0.1;
  j1.follower_min = -5.0;
  j1.follower_max = 5.0;
  configs.push_back(j1);

  // Joint 2: Min Max
  JointRemapConfig j2;
  j2.joint_name = "j2_min_max";
  j2.mode = RemapMode::MIN_MAX;
  j2.leader_min = 0.0;
  j2.leader_max = 1.0;
  j2.follower_min = 0.0;
  j2.follower_max = 2.0;
  configs.push_back(j2);

  KinematicRemapper remapper;
  EXPECT_TRUE(remapper.configure(configs));
  EXPECT_EQ(remapper.size(), 3u);

  EXPECT_EQ(remapper.find_joint_index("j1_scale_offset"), 1);
  EXPECT_EQ(remapper.find_joint_index("nonexistent"), -1);

  const std::vector<double> leader_in = {0.5, 0.5, 0.5};
  const std::vector<double> follower_out = remapper.remap_all(leader_in);

  ASSERT_EQ(follower_out.size(), 3u);
  EXPECT_NEAR(follower_out[0], 0.5, 1e-6);             // 1:1
  EXPECT_NEAR(follower_out[1], 2.0 * 0.5 + 0.1, 1e-6); // 1.1
  EXPECT_NEAR(follower_out[2], 1.0, 1e-6);             // 50% of [0, 2] = 1.0
}

TEST(TestKinematicRemapper, ModeStringConversions)
{
  EXPECT_EQ(KinematicRemapper::string_to_mode("direct"), RemapMode::DIRECT);
  EXPECT_EQ(KinematicRemapper::string_to_mode("DIRECT"), RemapMode::DIRECT);
  EXPECT_EQ(KinematicRemapper::string_to_mode("min_max"), RemapMode::MIN_MAX);
  EXPECT_EQ(KinematicRemapper::string_to_mode("centered_scale"), RemapMode::CENTERED_SCALE);
  EXPECT_EQ(KinematicRemapper::string_to_mode("scale_offset"), RemapMode::SCALE_OFFSET);
  EXPECT_EQ(KinematicRemapper::string_to_mode("invalid_mode"), RemapMode::DIRECT);

  EXPECT_EQ(KinematicRemapper::mode_to_string(RemapMode::DIRECT), "direct");
  EXPECT_EQ(KinematicRemapper::mode_to_string(RemapMode::MIN_MAX), "min_max");
  EXPECT_EQ(KinematicRemapper::mode_to_string(RemapMode::CENTERED_SCALE), "centered_scale");
  EXPECT_EQ(KinematicRemapper::mode_to_string(RemapMode::SCALE_OFFSET), "scale_offset");
}

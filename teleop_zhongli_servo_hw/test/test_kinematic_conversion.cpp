/**
 * @file test_kinematic_conversion.cpp
 * @brief Unit tests for Zhongli kinematic conversion math (SI Radians, coupled joints).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <cmath>
#include <unordered_map>
#include "teleop_zhongli_servo_hw/zhongli_protocol.hpp"

using namespace teleop_zhongli_servo_hw;

TEST(TestZhongliKinematics, SingleServoZeroMidpoint)
{
  // Joint shoulder_pan driven 1-to-1 by servo 0
  JointKinematicConfig joint;
  joint.joint_name = "shoulder_pan";
  joint.sources = {{0, 1.0, -1.0}}; // sign = -1.0

  std::unordered_map<uint8_t, ServoConfig> configs;
  ServoConfig s0;
  s0.id = 0;
  // 270 deg / 2000 pwm span
  s0.rad_per_pwm = (270.0 * M_PI / 180.0) / 2000.0;
  configs[0] = s0;

  // Midpoint 1500 PWM must yield exactly 0.0 radians
  std::unordered_map<uint8_t, int> pwms;
  pwms[0] = 1500;

  double rad = 0.0;
  std::string err;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(joint, configs, pwms, &rad, &err));
  EXPECT_NEAR(rad, 0.0, 1e-6);

  // Deviation of +500 PWM with sign -1.0 => negative radians
  pwms[0] = 2000;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(joint, configs, pwms, &rad, &err));
  const double expected_rad = -500.0 * s0.rad_per_pwm;
  EXPECT_NEAR(rad, expected_rad, 1e-6);
}

TEST(TestZhongliKinematics, CoupledDualServoWristRoll)
{
  // Joint wrist_roll driven by servo 5 and servo 3
  // Equivalent to uArm: ((5, -1.0), (3, -1.0))
  // With normalized weights w5 = 0.5, w3 = 0.5
  JointKinematicConfig wrist_roll;
  wrist_roll.joint_name = "wrist_roll";
  wrist_roll.sources = {
      {5, 0.5, -1.0},
      {3, 0.5, -1.0}};

  std::unordered_map<uint8_t, ServoConfig> configs;
  ServoConfig s;
  s.rad_per_pwm = (270.0 * M_PI / 180.0) / 2000.0;
  configs[3] = s;
  configs[5] = s;

  std::unordered_map<uint8_t, int> pwms;
  // Both at midpoint => 0.0 rad
  pwms[3] = 1500;
  pwms[5] = 1500;

  double rad = 0.0;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(wrist_roll, configs, pwms, &rad));
  EXPECT_NEAR(rad, 0.0, 1e-6);

  // Both move together by +200 PWM
  pwms[3] = 1700;
  pwms[5] = 1700;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(wrist_roll, configs, pwms, &rad));
  const double expected = (0.5 * 200.0 * s.rad_per_pwm * -1.0) + (0.5 * 200.0 * s.rad_per_pwm * -1.0);
  EXPECT_NEAR(rad, expected, 1e-6);

  // Differential motion: servo 5 moves +100, servo 3 moves -100 => cancel out to 0.0
  pwms[5] = 1600;
  pwms[3] = 1400;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(wrist_roll, configs, pwms, &rad));
  EXPECT_NEAR(rad, 0.0, 1e-6);
}

TEST(TestZhongliKinematics, JointLimitClamping)
{
  JointKinematicConfig joint;
  joint.joint_name = "clamped_joint";
  joint.sources = {{0, 1.0, 1.0}};
  joint.min_limit_rad = -0.5;
  joint.max_limit_rad = 0.5;

  std::unordered_map<uint8_t, ServoConfig> configs;
  ServoConfig s0;
  s0.rad_per_pwm = 0.01; // large scale
  configs[0] = s0;

  std::unordered_map<uint8_t, int> pwms;
  pwms[0] = 2000; // delta = +500 => raw rad = 5.0 rad

  double rad = 0.0;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(joint, configs, pwms, &rad));
  EXPECT_NEAR(rad, 0.5, 1e-6); // Clamped at upper limit
}

TEST(TestZhongliKinematics, ServoPwmRangeClamping)
{
  JointKinematicConfig joint;
  joint.joint_name = "shoulder_pan";
  joint.sources = {{0, 1.0, 1.0}};
  joint.min_limit_rad = -M_PI;
  joint.max_limit_rad = M_PI;

  std::unordered_map<uint8_t, ServoConfig> configs;
  ServoConfig s0;
  s0.id = 0;
  s0.min_pwm = 1000;
  s0.max_pwm = 2000;
  s0.rad_per_pwm = 0.001; // 0.001 rad per pwm
  configs[0] = s0;

  std::unordered_map<uint8_t, int> pwms;

  // Case 1: Within limits (1700 PWM) => delta = +200 => +0.2 rad
  pwms[0] = 1700;
  double rad = 0.0;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(joint, configs, pwms, &rad));
  EXPECT_NEAR(rad, 0.2, 1e-6);

  // Case 2: Exceeding max_pwm (2200 PWM > max 2000) => clamped to 2000 => delta = +500 => +0.5 rad
  pwms[0] = 2200;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(joint, configs, pwms, &rad));
  EXPECT_NEAR(rad, 0.5, 1e-6);

  // Case 3: Exceeding min_pwm (800 PWM < min 1000) => clamped to 1000 => delta = -500 => -0.5 rad
  pwms[0] = 800;
  EXPECT_TRUE(ZhongliProtocol::calculate_joint_position(joint, configs, pwms, &rad));
  EXPECT_NEAR(rad, -0.5, 1e-6);
}

TEST(TestZhongliKinematics, LowPassFilterConvergence)
{
  const double alpha = 0.2;
  double y = 0.0;
  const double target_step = 1.0;

  // First sample initialization
  y = target_step; // init at first step
  EXPECT_DOUBLE_EQ(y, 1.0);

  // New step input = 0.0
  const double step_zero = 0.0;
  y = alpha * step_zero + (1.0 - alpha) * y; // tick 1: 0.2*0 + 0.8*1.0 = 0.8
  EXPECT_NEAR(y, 0.8, 1e-6);

  y = alpha * step_zero + (1.0 - alpha) * y; // tick 2: 0.8 * 0.8 = 0.64
  EXPECT_NEAR(y, 0.64, 1e-6);

  // After 50 cycles with alpha=0.2, should converge towards 0.0
  for (int i = 0; i < 50; ++i)
  {
    y = alpha * step_zero + (1.0 - alpha) * y;
  }
  EXPECT_NEAR(y, 0.0, 1e-4);
}

TEST(TestZhongliKinematics, DirectIdenticalJointMapping)
{
  const std::vector<std::string> leader_joints = {
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
      "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"};

  const std::vector<std::string> follower_joints = {
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
      "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"};

  std::unordered_map<std::string, int> leader_indices;
  for (size_t i = 0; i < leader_joints.size(); ++i)
  {
    leader_indices[leader_joints[i]] = static_cast<int>(i);
  }

  // 1. Direct 1:1 match
  std::vector<int> mapped_indices(follower_joints.size(), -1);
  for (size_t i = 0; i < follower_joints.size(); ++i)
  {
    const auto &fj = follower_joints[i];
    auto it = leader_indices.find(fj);
    ASSERT_NE(it, leader_indices.end());
    mapped_indices[i] = it->second;
  }

  for (size_t i = 0; i < follower_joints.size(); ++i)
  {
    EXPECT_EQ(mapped_indices[i], static_cast<int>(i));
  }

  // 2. Missing/mismatched joint detection
  const std::string invalid_fj = "arm_unknown_joint";
  EXPECT_EQ(leader_indices.find(invalid_fj), leader_indices.end());
}

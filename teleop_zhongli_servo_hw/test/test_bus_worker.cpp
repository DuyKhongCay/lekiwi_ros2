/**
 * @file test_bus_worker.cpp
 * @brief Unit tests for ZhongliBusWorker lifecycle and RealtimeBuffer behavior.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include "teleop_zhongli_servo_hw/zhongli_bus_worker.hpp"

using namespace teleop_zhongli_servo_hw;

TEST(TestZhongliBusWorker, InitAndBufferState)
{
  ZhongliBusWorker worker;

  std::vector<uint8_t> servo_ids = {0, 1, 2, 3, 4, 5, 6};
  std::unordered_map<uint8_t, ServoConfig> servo_configs;
  for (uint8_t id : servo_ids)
  {
    ServoConfig cfg;
    cfg.id = id;
    servo_configs[id] = cfg;
  }

  std::vector<JointKinematicConfig> joint_configs = {
      {"arm_shoulder_pan", {{0, 1.0, -1.0}}, -M_PI, M_PI},
      {"arm_shoulder_lift", {{1, 1.0, 1.0}}, -M_PI, M_PI},
      {"arm_elbow_flex", {{2, 1.0, 1.0}}, -M_PI, M_PI},
  };

  std::string err;
  EXPECT_TRUE(worker.init(servo_ids, joint_configs, servo_configs, &err));

  // Verify RT snapshot read is valid pointer and initial values are sane
  const auto *snapshot = worker.read_state_rt();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_FALSE(snapshot->valid);
  EXPECT_EQ(snapshot->joint_positions_rad.size(), 3U);
  EXPECT_EQ(snapshot->joint_names.size(), 3U);
  EXPECT_EQ(snapshot->servo_feedbacks.size(), 7U);
  EXPECT_EQ(snapshot->joint_names[0], "arm_shoulder_pan");

  // Non-RT read for diagnostics
  const auto *non_rt_snapshot = worker.read_state_non_rt();
  ASSERT_NE(non_rt_snapshot, nullptr);
  EXPECT_FALSE(non_rt_snapshot->valid);

  // Worker should not be running yet
  EXPECT_FALSE(worker.is_running());
  EXPECT_EQ(worker.consecutive_errors(), 0U);
}

TEST(TestZhongliBusWorker, InvalidPortHandling)
{
  ZhongliBusWorker worker;
  std::vector<uint8_t> servo_ids = {0};
  std::vector<JointKinematicConfig> joint_configs = {{"joint_0", {{0, 1.0, 1.0}}, -M_PI, M_PI}};
  std::unordered_map<uint8_t, ServoConfig> servo_configs;

  EXPECT_TRUE(worker.init(servo_ids, joint_configs, servo_configs));

  std::string err;
  // Connecting to a non-existent device port must fail cleanly without throwing
  EXPECT_FALSE(worker.configure("/dev/non_existent_uarm_port_12345", 1000000, 20, 300, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(worker.activate(&err));
}


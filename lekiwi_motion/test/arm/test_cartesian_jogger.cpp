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
 * @file test_cartesian_jogger.cpp
 * @brief Unit tests for pure POCO CartesianJogger.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <string>
#include <vector>

#include "lekiwi_motion/arm/cartesian_jogger.hpp"
#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace ws = lekiwi_motion::workspace;
namespace arm = lekiwi_motion::arm;

class CartesianJoggerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
#ifdef WORKSPACE_URDF_PATH
    urdf_ = urdf::parseURDFFile(WORKSPACE_URDF_PATH);
    ASSERT_TRUE(urdf_) << "Failed to parse test URDF: " << WORKSPACE_URDF_PATH;

    joint_names_ = {
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
      "arm_wrist_flex", "arm_wrist_roll"};

    std::string err;
    bool ok = ws::extract_kinematics_from_urdf(
      *urdf_, "base_footprint", "gripperframe", joint_names_, model_, err);
    ASSERT_TRUE(ok) << "URDF extraction failed: " << err;

    solver_ = std::make_shared<ws::SO101AnalyticalSolver>(model_);
    jogger_ = std::make_unique<arm::CartesianJogger>(solver_, model_);
#else
    FAIL() << "WORKSPACE_URDF_PATH not defined";
#endif
  }

  std::shared_ptr<urdf::ModelInterface> urdf_;
  std::vector<std::string> joint_names_;
  ws::KinematicsModel model_;
  std::shared_ptr<ws::SO101AnalyticalSolver> solver_;
  std::unique_ptr<arm::CartesianJogger> jogger_;
};

TEST_F(CartesianJoggerTest, LifecycleInitializationFkSyncAndZeroDeltaHold)
{
  // Null solver throws
  EXPECT_THROW(arm::CartesianJogger(nullptr, model_), std::invalid_argument);

  // Uninitialized state
  EXPECT_FALSE(jogger_->is_initialized());

  // Initialization and FK Sync
  const std::array<double, 5> nominal_joints{0.0, -0.5, 1.2, 0.3, 0.0};
  jogger_->sync_from_fk(nominal_joints);

  EXPECT_TRUE(jogger_->is_initialized());
  const auto pos = jogger_->current_position();
  EXPECT_TRUE(pos.is_finite());
  EXPECT_GT(pos.x, 0.05);

  const auto joints = jogger_->current_joints();
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(joints[i], nominal_joints[i], 1e-4);
  }

  // Zero delta hold maintains position without drift
  arm::CartesianDelta zero_delta{};
  auto res = jogger_->process_jog(zero_delta);
  EXPECT_TRUE(res.success);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(res.joints[i], nominal_joints[i], 1e-5);
  }
}

TEST_F(CartesianJoggerTest, SmallDisplacementJog)
{
  const std::array<double, 5> nominal_joints{0.0, -0.5, 1.2, 0.3, 0.0};
  jogger_->sync_from_fk(nominal_joints);
  const auto initial_pos = jogger_->current_position();

  arm::CartesianDelta delta{};
  delta.dx = 0.005;  // 5mm forward

  auto res = jogger_->process_jog(delta);
  EXPECT_TRUE(res.success) << "Reason: " << res.reason;
  EXPECT_NEAR(jogger_->current_position().x, initial_pos.x + 0.005, 1e-4);
}

TEST_F(CartesianJoggerTest, OutOfReachRejectionRetainsPose)
{
  const std::array<double, 5> nominal_joints{0.0, -0.5, 1.2, 0.3, 0.0};
  jogger_->sync_from_fk(nominal_joints);
  const auto initial_pos = jogger_->current_position();

  arm::CartesianDelta delta{};
  delta.dx = 2.0;  // 2 meters forward (physically impossible)

  auto res = jogger_->process_jog(delta);
  EXPECT_FALSE(res.success);
  EXPECT_FALSE(res.reason.empty());

  // Target position must NOT drift
  EXPECT_DOUBLE_EQ(jogger_->current_position().x, initial_pos.x);
  EXPECT_DOUBLE_EQ(jogger_->current_position().y, initial_pos.y);
  EXPECT_DOUBLE_EQ(jogger_->current_position().z, initial_pos.z);
}

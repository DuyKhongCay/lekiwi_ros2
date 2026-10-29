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
 * @file test_joint_motion_filter.cpp
 * @brief Unit tests for pure POCO JointMotionFilter.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <array>

#include "lekiwi_motion/arm/joint_motion_filter.hpp"

namespace arm = lekiwi_motion::arm;

TEST(JointMotionFilterTest, LifecyclePassThroughResetAndBumplessSync)
{
  arm::JointMotionFilter<5> filter(0.85);
  EXPECT_FALSE(filter.is_initialized());

  // Initial pass-through without spike
  const std::array<double, 5> initial_targets{0.1, -0.2, 0.3, -0.4, 0.5};
  const auto state = filter.update(initial_targets, 0.02);

  EXPECT_TRUE(filter.is_initialized());
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(state.positions[i], initial_targets[i]);
    EXPECT_DOUBLE_EQ(state.velocities[i], 0.0);
  }

  // Reset clears history
  filter.reset();
  EXPECT_FALSE(filter.is_initialized());

  const std::array<double, 5> after_reset{5.0, 5.0, 5.0, 5.0, 5.0};
  const auto state_reset = filter.update(after_reset, 0.02);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(state_reset.positions[i], 5.0);
    EXPECT_DOUBLE_EQ(state_reset.velocities[i], 0.0);
  }

  // Sync state bumpless
  const std::array<double, 5> physical{0.1, 0.2, -0.3, 0.4, -0.5};
  filter.sync(physical);
  EXPECT_TRUE(filter.is_initialized());

  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(filter.current_positions()[i], physical[i]);
  }

  const std::array<double, 5> next_target{0.101, 0.2, -0.3, 0.4, -0.5};
  const auto state_synced = filter.update(next_target, 0.02);
  EXPECT_NEAR(state_synced.positions[0], 0.10085, 1e-4);
  EXPECT_GT(state_synced.velocities[0], 0.0);
  EXPECT_LT(state_synced.velocities[0], 0.1);
}

TEST(JointMotionFilterTest, ExponentialSmoothingCalculation)
{
  arm::JointMotionFilter<5> filter(0.5);  // alpha = 0.5

  const std::array<double, 5> t0{1.0, 1.0, 1.0, 1.0, 1.0};
  filter.update(t0, 0.02);

  const std::array<double, 5> t1{2.0, 2.0, 2.0, 2.0, 2.0};
  const auto state1 = filter.update(t1, 0.02);

  // Expected position: 0.5 * 2.0 + 0.5 * 1.0 = 1.5
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(state1.positions[i], 1.5);
    // Velocity: (1.5 - 1.0) / 0.02 = 25.0
    EXPECT_NEAR(state1.velocities[i], 25.0, 1e-6);
  }
}

TEST(JointMotionFilterTest, VelocityRateLimiting)
{
  // Filter with alpha=1.0 (pure passthrough) but max_velocity = 2.0 rad/s
  arm::JointMotionFilter<5> filter(1.0, 2.0);

  const std::array<double, 5> t0{0.0, 0.0, 0.0, 0.0, 0.0};
  filter.update(t0, 0.02);

  // Attempt massive jump: target = 1.0 rad in 0.02s (would be 50 rad/s)
  const std::array<double, 5> t1{1.0, 1.0, 1.0, 1.0, 1.0};
  const auto state1 = filter.update(t1, 0.02);

  // Max allowed step = 2.0 * 0.02 = 0.04 rad
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(state1.positions[i], 0.04);
    EXPECT_NEAR(state1.velocities[i], 2.0, 1e-6);
  }
}


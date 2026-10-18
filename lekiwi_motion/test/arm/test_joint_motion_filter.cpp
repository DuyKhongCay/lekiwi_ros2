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

TEST(JointMotionFilterTest, InitialPassThroughWithoutSpike)
{
  arm::JointMotionFilter<6> filter(0.85);
  EXPECT_FALSE(filter.is_initialized());

  const std::array<double, 6> initial_targets{0.1, -0.2, 0.3, -0.4, 0.5, 1.0};
  const auto state = filter.update(initial_targets, 0.02);

  EXPECT_TRUE(filter.is_initialized());
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state.positions[i], initial_targets[i]);
    EXPECT_DOUBLE_EQ(state.velocities[i], 0.0);
  }
}

TEST(JointMotionFilterTest, ExponentialSmoothingCalculation)
{
  arm::JointMotionFilter<6> filter(0.5);  // alpha = 0.5

  const std::array<double, 6> t0{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  filter.update(t0, 0.02);

  const std::array<double, 6> t1{2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  const auto state1 = filter.update(t1, 0.02);

  // Expected position: 0.5 * 2.0 + 0.5 * 1.0 = 1.5
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state1.positions[i], 1.5);
    // Velocity: (1.5 - 1.0) / 0.02 = 25.0
    EXPECT_NEAR(state1.velocities[i], 25.0, 1e-6);
  }
}

TEST(JointMotionFilterTest, ResetClearsHistory)
{
  arm::JointMotionFilter<6> filter(0.85);

  const std::array<double, 6> t0{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  filter.update(t0, 0.02);
  EXPECT_TRUE(filter.is_initialized());

  filter.reset();
  EXPECT_FALSE(filter.is_initialized());

  const std::array<double, 6> t1{5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  const auto state1 = filter.update(t1, 0.02);

  // After reset, first sample initializes cleanly without blending from t0
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state1.positions[i], 5.0);
    EXPECT_DOUBLE_EQ(state1.velocities[i], 0.0);
  }
}


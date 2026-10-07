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
 * @file test_torque_command_state.cpp
 * @brief Unit tests for TorqueCommandState domain logic and joint group slicing.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include "lekiwi_motion/supervision/torque_manager_node.hpp"

using lekiwi_motion::TorqueCommandState;
using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;

/**
 * @brief Tests that applying torque settings to one joint group preserves states in other groups.
 */
TEST(TorqueCommandState, PreservesOtherGroup)
{
  TorqueCommandState state({"a1", "a2"}, {"b1", "b2"});
  EXPECT_EQ(state.apply(Request::TARGET_ALL, true), (std::vector<double>{1, 1, 1, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false), (std::vector<double>{0, 0, 1, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_BASE, false), (std::vector<double>{0, 0, 0, 0}));
  EXPECT_EQ(state.apply(Request::TARGET_ARM, true), (std::vector<double>{1, 1, 0, 0}));
}

/**
 * @brief Verifies that reset() restores all joint group torque states to 1.0 (enabled).
 */
TEST(TorqueCommandState, ResetReinitializesToAllEnabled)
{
  TorqueCommandState state({"a"}, {"b"});
  state.apply(Request::TARGET_ALL, false);
  EXPECT_EQ(state.current_states(), (std::vector<double>{0, 0}));
  state.reset();
  EXPECT_EQ(state.current_states(), (std::vector<double>{1, 1}));
}

/**
 * @brief Verifies that invalid target identifiers throw std::invalid_argument without mutating state.
 */
TEST(TorqueCommandState, RejectsInvalidTargetWithoutChangingState)
{
  TorqueCommandState state({"a"}, {"b"});
  state.apply(Request::TARGET_ALL, true);
  EXPECT_THROW(state.apply(99, false), std::invalid_argument);
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false), (std::vector<double>{0, 1}));
}

/**
 * @brief Ensures empty joint collections in constructor throw std::invalid_argument.
 */
TEST(TorqueCommandState, RejectsEmptyConfiguration)
{
  EXPECT_THROW((TorqueCommandState({}, {"b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a"}, {})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({}, {})), std::invalid_argument);
}

/**
 * @brief Tests that is_group_enabled accurately reports active joints across group mutations.
 */
TEST(TorqueCommandState, IsGroupEnabledAndToggles)
{
  TorqueCommandState state({"a"}, {"b"});
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ARM));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_BASE));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ALL));

  // Disable arm
  state.apply(Request::TARGET_ARM, false);
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_ARM));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_BASE));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ALL));

  // Disable base
  state.apply(Request::TARGET_BASE, false);
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_BASE));
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_ALL));

  // Re-enable arm
  state.apply(Request::TARGET_ARM, true);
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ARM));
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_BASE));
}

/**
 * @brief Verifies partition count accessors for arm and total joints.
 */
TEST(TorqueCommandState, JointCounts)
{
  TorqueCommandState state({"a1", "a2", "a3"}, {"b1", "b2"});
  EXPECT_EQ(state.arm_count(), 3);
  EXPECT_EQ(state.total_count(), 5);
}

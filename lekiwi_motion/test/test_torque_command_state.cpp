// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include "lekiwi_motion/torque_manager_node.hpp"

using lekiwi_motion::TorqueCommandState;
using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;

// Initial state defaults to enabled (1.0) so partial commands can be executed immediately.
TEST(TorqueCommandState, AllowsPartialCommandsOnStartup)
{
  TorqueCommandState state({"a"}, {"b"});
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false), (std::vector<double>{0, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_BASE, false), (std::vector<double>{0, 0}));
  EXPECT_EQ(state.apply(Request::TARGET_ALL, true), (std::vector<double>{1, 1}));
}

// Slice division preserves other group without side-effects.
TEST(TorqueCommandState, PreservesOtherGroup)
{
  TorqueCommandState state({"a1", "a2"}, {"b1", "b2"});
  EXPECT_EQ(state.apply(Request::TARGET_ALL, true), (std::vector<double>{1, 1, 1, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false), (std::vector<double>{0, 0, 1, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_BASE, false), (std::vector<double>{0, 0, 0, 0}));
  EXPECT_EQ(state.apply(Request::TARGET_ARM, true), (std::vector<double>{1, 1, 0, 0}));
}

// Reset re-initializes states to 1.0
TEST(TorqueCommandState, ResetReinitializesToAllEnabled)
{
  TorqueCommandState state({"a"}, {"b"});
  state.apply(Request::TARGET_ALL, false);
  EXPECT_EQ(state.current_states(), (std::vector<double>{0, 0}));
  state.reset();
  EXPECT_EQ(state.current_states(), (std::vector<double>{1, 1}));
}

// Invalid targets must not mutate state.
TEST(TorqueCommandState, RejectsInvalidTargetWithoutChangingState)
{
  TorqueCommandState state({"a"}, {"b"});
  state.apply(Request::TARGET_ALL, true);
  EXPECT_THROW(state.apply(99, false), std::invalid_argument);
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false), (std::vector<double>{0, 1}));
}

// Empty joint groups must fail at construction.
TEST(TorqueCommandState, RejectsEmptyConfiguration)
{
  EXPECT_THROW((TorqueCommandState({}, {"b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a"}, {})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({}, {})), std::invalid_argument);
}

// is_group_enabled accurately reflects current target group state using std::any_of
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

// Verifies counts and multi-joint group slicing
TEST(TorqueCommandState, SlicingAndCounts)
{
  TorqueCommandState state({"a1", "a2", "a3"}, {"b1", "b2"});
  EXPECT_EQ(state.arm_count(), 3);
  EXPECT_EQ(state.total_count(), 5);
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false), (std::vector<double>{0, 0, 0, 1, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_BASE, false), (std::vector<double>{0, 0, 0, 0, 0}));
}

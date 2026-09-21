// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include "lekiwi_control/torque_command_state.hpp"

using lekiwi_control::TorqueCommandState;
using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;

// Initial state defaults to enabled (1.0) so partial commands can be executed immediately.
TEST(TorqueCommandState, AllowsPartialCommandsOnStartup)
{
  TorqueCommandState state({"a", "b"}, {"a"}, {"b"});
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false, true), (std::vector<double>{0, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_BASE, false, true), (std::vector<double>{0, 0}));
  EXPECT_EQ(state.apply(Request::TARGET_ALL, true, true), (std::vector<double>{1, 1}));
}

// Interleaved controller ordering must not alter group selection.
TEST(TorqueCommandState, PreservesOtherGroupByName)
{
  TorqueCommandState state({"b2", "a1", "b1", "a2"}, {"a1", "a2"}, {"b1", "b2"});
  EXPECT_EQ(state.apply(Request::TARGET_ALL, true, true), (std::vector<double>{1, 1, 1, 1}));
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false, true), (std::vector<double>{1, 0, 1, 0}));
  EXPECT_EQ(state.apply(Request::TARGET_BASE, false, true), (std::vector<double>{0, 0, 0, 0}));
  EXPECT_EQ(state.apply(Request::TARGET_ARM, true, true), (std::vector<double>{0, 1, 0, 1}));
}

// Controller disappearance invalidates assumptions even after reconnection.
TEST(TorqueCommandState, RequiresReinitializationAfterControllerLoss)
{
  TorqueCommandState state({"a", "b"}, {"a"}, {"b"});
  state.apply(Request::TARGET_ALL, true, true);
  EXPECT_THROW(state.apply(Request::TARGET_BASE, false, false), std::runtime_error);
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false, true), (std::vector<double>{0, 1}));
  EXPECT_THROW(state.apply(Request::TARGET_ALL, true, false), std::runtime_error);
  EXPECT_EQ(state.apply(Request::TARGET_ALL, false, true), (std::vector<double>{0, 0}));
}

// Invalid targets must not mutate a previously valid command vector.
TEST(TorqueCommandState, RejectsInvalidTargetWithoutChangingState)
{
  TorqueCommandState state({"a", "b"}, {"a"}, {"b"});
  state.apply(Request::TARGET_ALL, true, true);
  EXPECT_THROW(state.apply(99, false, true), std::invalid_argument);
  EXPECT_EQ(state.apply(Request::TARGET_ARM, false, true), (std::vector<double>{0, 1}));
}

// Invalid or ambiguous joint mappings must fail before a node can publish.
TEST(TorqueCommandState, RejectsInvalidConfiguration)
{
  EXPECT_THROW((TorqueCommandState({"a", "a"}, {"a"}, {"b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a", "b"}, {"a"}, {"a", "b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a", "b"}, {"a", "a"}, {"b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a", "b"}, {"a"}, {"b", "b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a", "b", "c"}, {"a"}, {"b"})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a"}, {"a"}, {})), std::invalid_argument);
  EXPECT_THROW((TorqueCommandState({"a", ""}, {"a"}, {""})), std::invalid_argument);
}

// is_group_enabled accurately reflects current target group state
TEST(TorqueCommandState, IsGroupEnabledAndToggles)
{
  TorqueCommandState state({"a", "b"}, {"a"}, {"b"});
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ARM));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_BASE));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ALL));

  // Disable arm
  state.apply(Request::TARGET_ARM, false, true);
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_ARM));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_BASE));
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ALL));

  // Disable base
  state.apply(Request::TARGET_BASE, false, true);
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_BASE));
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_ALL));

  // Re-enable arm
  state.apply(Request::TARGET_ARM, true, true);
  EXPECT_TRUE(state.is_group_enabled(Request::TARGET_ARM));
  EXPECT_FALSE(state.is_group_enabled(Request::TARGET_BASE));
}

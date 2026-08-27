// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "gstreamer/gst_pipe.hpp"

using namespace std::chrono_literals;

TEST(SampleGate, WaitsForActiveCallback)
{
  auto gate = std::make_shared<lekiwi_perception::SampleGate>();
  {
    lekiwi_perception::CallbackGuard callback(gate);
    ASSERT_TRUE(static_cast<bool>(callback));
    gate->close();
    EXPECT_FALSE(gate->wait_drained(1ms));
  }
  EXPECT_TRUE(gate->wait_drained(1ms));
  lekiwi_perception::CallbackGuard rejected(gate);
  EXPECT_FALSE(static_cast<bool>(rejected));
}

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
  EXPECT_TRUE(gate->with_callback([&gate]() {
    gate->close();
    EXPECT_FALSE(gate->wait_drained(1ms));
  }));
  EXPECT_TRUE(gate->wait_drained(1ms));
  EXPECT_FALSE(gate->with_callback([]() {}));
}

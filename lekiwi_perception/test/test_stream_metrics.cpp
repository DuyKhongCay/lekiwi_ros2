// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include "gstreamer/pipe_builder.hpp"
#include "stream_metrics_tracker.hpp"

namespace
{

using lekiwi_perception::StreamId;
using lekiwi_perception::StreamMetricsTracker;

TEST(StreamMetricsTracker, ComputesLatencyAndRates)
{
  StreamMetricsTracker tracker;
  bool warn_skew = false;
  const int64_t stamp = 1'000'000'000;
  const int64_t now = 1'020'000'000;  // 20ms latency

  tracker.record_frame(StreamId::kStereoLeft, stamp, now, 35.0, warn_skew);
  EXPECT_FALSE(warn_skew);

  const auto snapshot = tracker.snapshot();
  EXPECT_NEAR(snapshot.publish_latency_ms[0], 20.0F, 0.01F);
}

TEST(StreamMetricsTracker, MatchesLeRobotTriplets)
{
  StreamMetricsTracker tracker;
  bool warn_skew = false;
  const int64_t base_stamp = 2'000'000'000;

  // Record 3 matched camera frames within 5ms of each other
  tracker.record_frame(StreamId::kStereoRight, base_stamp, base_stamp + 10'000'000, 35.0, warn_skew);
  EXPECT_FALSE(warn_skew);
  tracker.record_frame(StreamId::kUsbWrist, base_stamp + 2'000'000, base_stamp + 10'000'000, 35.0, warn_skew);
  EXPECT_FALSE(warn_skew);
  tracker.record_frame(StreamId::kUsbSide, base_stamp + 4'000'000, base_stamp + 10'000'000, 35.0, warn_skew);
  EXPECT_FALSE(warn_skew);

  const auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.lerobot_triplet_match_count, 1U);
  EXPECT_EQ(snapshot.lerobot_triplet_discarded_frame_count, 0U);
  EXPECT_NEAR(snapshot.lerobot_skew_p50_ms, 4.0F, 0.01F);
}

TEST(StreamMetricsTracker, DiscardsSkewedFrames)
{
  StreamMetricsTracker tracker;
  bool warn_skew = false;
  const int64_t base_stamp = 3'000'000'000;

  // Right camera frame at 0ms, wrist at 50ms (skew > 35ms warning threshold)
  tracker.record_frame(StreamId::kStereoRight, base_stamp, base_stamp + 60'000'000, 35.0, warn_skew);
  tracker.record_frame(StreamId::kUsbWrist, base_stamp + 50'000'000, base_stamp + 60'000'000, 35.0, warn_skew);
  tracker.record_frame(StreamId::kUsbSide, base_stamp + 52'000'000, base_stamp + 60'000'000, 35.0, warn_skew);

  EXPECT_TRUE(warn_skew);
  const auto snapshot = tracker.snapshot();
  EXPECT_GE(snapshot.lerobot_triplet_discarded_frame_count, 1U);
}

}  // namespace

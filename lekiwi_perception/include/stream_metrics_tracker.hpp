// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__STREAM_METRICS_TRACKER_HPP_
#define LEKIWI_PERCEPTION__STREAM_METRICS_TRACKER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "gstreamer/pipe_builder.hpp"

namespace lekiwi_perception
{

struct MetricsSnapshot
{
  std::array<float, 4U> rates{};
  std::array<float, 4U> publish_latency_ms{};
  float lerobot_skew_p50_ms{0.0F};
  float lerobot_skew_p95_ms{0.0F};
  uint64_t lerobot_triplet_match_count{0};
  uint64_t lerobot_triplet_discarded_frame_count{0};
};

class StreamMetricsTracker
{
public:
  static constexpr std::size_t kStreamCount = 4U;
  static constexpr std::size_t kLeRobotStreamCount = 3U;
  static constexpr std::size_t kTripletQueueCapacity = 8U;
  static constexpr std::size_t kMaxSkewSamples = 600U;

  StreamMetricsTracker();

  void reset();
  void record_frame(
    StreamId stream,
    int64_t stamp_ns,
    int64_t now_ns,
    double skew_warning_ms,
    bool & warn_skew);

  [[nodiscard]] MetricsSnapshot snapshot();

private:
  mutable std::mutex mutex_;
  std::array<uint64_t, kStreamCount> published_frames_{};
  std::array<uint64_t, kStreamCount> prev_published_frames_{};
  std::array<float, kStreamCount> publish_latency_ms_{};
  std::array<std::deque<int64_t>, kLeRobotStreamCount> lerobot_pending_{};
  std::array<float, kStreamCount> rates_{};
  std::vector<double> skew_samples_ms_;
  uint64_t lerobot_triplet_match_count_{0};
  uint64_t lerobot_triplet_discarded_frame_count_{0};
  std::chrono::steady_clock::time_point prev_rate_time_;
};

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__STREAM_METRICS_TRACKER_HPP_

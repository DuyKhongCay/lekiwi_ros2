// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "stream_metrics_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace lekiwi_perception
{
namespace
{

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::clamp(fraction, 0.0, 1.0) * static_cast<double>(values.size() - 1U));
  return values[index];
}

}  // namespace

StreamMetricsTracker::StreamMetricsTracker()
{
  reset();
}

void StreamMetricsTracker::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  published_frames_.fill(0);
  prev_published_frames_.fill(0);
  publish_latency_ms_.fill(0.0F);
  rates_.fill(0.0F);
  for (auto & queue : lerobot_pending_) {
    queue.clear();
  }
  skew_samples_ms_.clear();
  lerobot_triplet_match_count_ = 0;
  lerobot_triplet_discarded_frame_count_ = 0;
  prev_rate_time_ = std::chrono::steady_clock::now();
}

void StreamMetricsTracker::record_frame(
  StreamId stream,
  int64_t stamp_ns,
  int64_t now_ns,
  double skew_warning_ms,
  bool & warn_skew)
{
  warn_skew = false;
  std::lock_guard<std::mutex> lock(mutex_);
  const auto index = stream_index(stream);
  ++published_frames_[index];
  publish_latency_ms_[index] = static_cast<float>(
    std::max<int64_t>(0, now_ns - stamp_ns) / 1'000'000.0);

  if (stream != StreamId::kStereoLeft) {
    const auto lerobot_index = index - stream_index(StreamId::kStereoRight);
    auto & pending = lerobot_pending_[lerobot_index];
    pending.push_back(stamp_ns);
    if (pending.size() > kTripletQueueCapacity) {
      pending.pop_front();
      ++lerobot_triplet_discarded_frame_count_;
    }
    while (std::all_of(
        lerobot_pending_.begin(), lerobot_pending_.end(),
        [](const auto & queue) {return !queue.empty();}))
    {
      std::size_t oldest = 0U;
      int64_t minimum = lerobot_pending_[0].front();
      int64_t maximum = minimum;
      for (std::size_t queue = 1U; queue < kLeRobotStreamCount; ++queue) {
        const int64_t value = lerobot_pending_[queue].front();
        if (value < minimum) {
          minimum = value;
          oldest = queue;
        }
        maximum = std::max(maximum, value);
      }
      const double skew_ms = static_cast<double>(maximum - minimum) / 1'000'000.0;
      if (skew_ms > skew_warning_ms) {
        lerobot_pending_[oldest].pop_front();
        ++lerobot_triplet_discarded_frame_count_;
        warn_skew = true;
        continue;
      }
      for (auto & queue : lerobot_pending_) {
        queue.pop_front();
      }
      ++lerobot_triplet_match_count_;
      skew_samples_ms_.push_back(skew_ms);
      if (skew_samples_ms_.size() > kMaxSkewSamples) {
        skew_samples_ms_.erase(skew_samples_ms_.begin());
      }
    }
  }
}

MetricsSnapshot StreamMetricsTracker::snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  const double seconds = std::chrono::duration<double>(
    now_steady - prev_rate_time_).count();
  if (seconds >= 0.1) {
    for (std::size_t index = 0; index < kStreamCount; ++index) {
      rates_[index] = static_cast<float>(
        (published_frames_[index] - prev_published_frames_[index]) / seconds);
      prev_published_frames_[index] = published_frames_[index];
    }
    prev_rate_time_ = now_steady;
  }

  MetricsSnapshot snapshot;
  snapshot.rates = rates_;
  snapshot.publish_latency_ms = publish_latency_ms_;
  snapshot.lerobot_skew_p50_ms = static_cast<float>(percentile(skew_samples_ms_, 0.50));
  snapshot.lerobot_skew_p95_ms = static_cast<float>(percentile(skew_samples_ms_, 0.95));
  snapshot.lerobot_triplet_match_count = lerobot_triplet_match_count_;
  snapshot.lerobot_triplet_discarded_frame_count = lerobot_triplet_discarded_frame_count_;
  return snapshot;
}

}  // namespace lekiwi_perception

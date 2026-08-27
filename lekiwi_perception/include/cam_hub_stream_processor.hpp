// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__CAM_HUB_STREAM_PROCESSOR_HPP_
#define LEKIWI_PERCEPTION__CAM_HUB_STREAM_PROCESSOR_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gstreamer/gst_pipe.hpp"
#include "gstreamer/pipe_builder.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace lekiwi_perception
{

class CamHubStreamProcessor
{
public:
  static constexpr std::size_t kStreamCount = 4U;
  static constexpr std::size_t kLeRobotStreamCount = 3U;

  using ImagePublisher = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>;
  using InfoPublisher = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>;

  struct Bindings
  {
    rclcpp::Logger logger;
    rclcpp::Clock::SharedPtr clock;
    std::array<CamStreamConfig, kStreamCount> streams;
    std::array<sensor_msgs::msg::CameraInfo, kStreamCount> camera_infos;
    std::array<ImagePublisher::SharedPtr, kStreamCount> image_pubs;
    std::array<InfoPublisher::SharedPtr, kStreamCount> info_pubs;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fen_pub;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub;
    ImagePublisher::SharedPtr debug_image_pub;
    InfoPublisher::SharedPtr debug_info_pub;
    int64_t gst_to_ros_offset_ns{0};
    double skew_warning_ms{35.0};
  };

  struct MetricsSnapshot
  {
    std::array<float, kStreamCount> rates{};
    std::array<float, kStreamCount> publish_latency_ms{};
    float lerobot_skew_p50_ms{0.0F};
    float lerobot_skew_p95_ms{0.0F};
    uint64_t lerobot_triplet_match_count{0};
    uint64_t lerobot_triplet_discarded_frame_count{0};
  };

  explicit CamHubStreamProcessor(Bindings bindings);

  void handle_sample(StreamId stream, GstAppSink * sink, GstElement * pipeline);
  void reset_metrics();
  [[nodiscard]] MetricsSnapshot snapshot_metrics();

private:
  static constexpr std::size_t kTripletQueueCapacity = 8U;

  struct Metrics
  {
    std::array<uint64_t, kStreamCount> published_frames{};
    std::array<uint64_t, kStreamCount> prev_published_frames{};
    std::array<float, kStreamCount> publish_latency_ms{};
    std::array<std::deque<int64_t>, kLeRobotStreamCount> lerobot_pending;
    std::array<float, kStreamCount> rates{};
    std::vector<double> skew_samples_ms;
    uint64_t lerobot_triplet_match_count{0};
    uint64_t lerobot_triplet_discarded_frame_count{0};
    std::chrono::steady_clock::time_point prev_rate_time;
  };

  void publish_hailo(GstSample * sample, GstElement * pipeline);
  void publish_rgb(StreamId stream, GstSample * sample, GstElement * pipeline);
  [[nodiscard]] builtin_interfaces::msg::Time sample_stamp(
    GstSample * sample, GstElement * pipeline) const;
  void publish_camera_info(StreamId stream, const builtin_interfaces::msg::Time & stamp);
  void publish_debug_camera_info(const builtin_interfaces::msg::Time & stamp);
  void record_published_frame(StreamId stream, int64_t stamp_ns);

  Bindings bindings_;
  std::mutex metrics_mutex_;
  std::string last_logged_fen_;
  Metrics metrics_;
};

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__CAM_HUB_STREAM_PROCESSOR_HPP_

// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__CAM_HUB_STREAM_PROCESSOR_HPP_
#define LEKIWI_PERCEPTION__CAM_HUB_STREAM_PROCESSOR_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "gstreamer/gst_pipe.hpp"
#include "gstreamer/pipe_builder.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "stream_metrics_tracker.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace lekiwi_perception
{

class CamHubStreamProcessor
{
public:
  static constexpr std::size_t kStreamCount = 4U;

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

  explicit CamHubStreamProcessor(Bindings bindings);

  void handle_sample(StreamId stream, GstAppSink * sink, GstElement * pipeline);
  void reset_metrics();
  [[nodiscard]] MetricsSnapshot snapshot_metrics();

private:
  void publish_hailo(GstSample * sample, GstElement * pipeline);
  void publish_rgb(StreamId stream, GstSample * sample, GstElement * pipeline);
  [[nodiscard]] builtin_interfaces::msg::Time sample_stamp(
    GstSample * sample, GstElement * pipeline) const;
  void publish_camera_info(StreamId stream, const builtin_interfaces::msg::Time & stamp);
  void publish_debug_camera_info(const builtin_interfaces::msg::Time & stamp);
  void record_published_frame(StreamId stream, int64_t stamp_ns);

  Bindings bindings_;
  std::string last_logged_fen_;
  StreamMetricsTracker metrics_tracker_;
};

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__CAM_HUB_STREAM_PROCESSOR_HPP_

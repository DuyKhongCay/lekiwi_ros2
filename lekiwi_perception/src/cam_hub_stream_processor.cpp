// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "cam_hub_stream_processor.hpp"

#include <gst/video/video.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <memory>
#include <utility>

#include "gst_hailo_meta.hpp"
#include "hailo/chess_vision_mapper.hpp"
#include "rclcpp/qos.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

namespace lekiwi_perception
{
namespace
{

constexpr std::size_t stream_index(StreamId stream)
{
  return static_cast<std::size_t>(stream);
}

const char * stream_name(StreamId stream)
{
  switch (stream) {
    case StreamId::kStereoLeft:
      return "stereo_left";
    case StreamId::kStereoRight:
      return "stereo_right";
    case StreamId::kUsbWrist:
      return "usb_wrist";
    case StreamId::kUsbSide:
      return "usb_side";
  }
  return "unknown";
}

class GstMappedSample
{
public:
  explicit GstMappedSample(GstSample * sample)
  : sample_(sample)
  {
    buffer_ = sample_ != nullptr ? gst_sample_get_buffer(sample_) : nullptr;
    mapped_ = buffer_ != nullptr && gst_buffer_map(buffer_, &map_, GST_MAP_READ);
  }

  ~GstMappedSample()
  {
    if (mapped_) {
      gst_buffer_unmap(buffer_, &map_);
    }
    if (sample_ != nullptr) {
      gst_sample_unref(sample_);
    }
  }

  GstMappedSample(const GstMappedSample &) = delete;
  GstMappedSample & operator=(const GstMappedSample &) = delete;

  [[nodiscard]] bool valid() const noexcept {return mapped_;}
  [[nodiscard]] GstBuffer * buffer() const noexcept {return buffer_;}
  [[nodiscard]] const uint8_t * data() const noexcept {return map_.data;}
  [[nodiscard]] std::size_t size() const noexcept {return map_.size;}

private:
  GstSample * sample_{nullptr};
  GstBuffer * buffer_{nullptr};
  GstMapInfo map_{};
  bool mapped_{false};
};

bool make_debug_bgr(
  const GstVideoInfo & info,
  const uint8_t * data,
  std::size_t size,
  cv::Mat & output)
{
  const auto width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&info));
  const auto height = static_cast<uint32_t>(GST_VIDEO_INFO_HEIGHT(&info));
  if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
    return false;
  }
  const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
  const auto offset = static_cast<std::size_t>(GST_VIDEO_INFO_PLANE_OFFSET(&info, 0));
  if (stride < static_cast<int>(width * 3U) ||
    offset + static_cast<std::size_t>(stride) * height > size)
  {
    return false;
  }
  const cv::Mat rgb(
    static_cast<int>(height), static_cast<int>(width), CV_8UC3,
    const_cast<uint8_t *>(data + offset), static_cast<std::size_t>(stride));
  cv::cvtColor(rgb, output, cv::COLOR_RGB2BGR);
  return true;
}

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

CamHubStreamProcessor::CamHubStreamProcessor(Bindings bindings)
: bindings_(std::move(bindings))
{
  reset_metrics();
}

void CamHubStreamProcessor::reset_metrics()
{
  std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
  metrics_ = Metrics{};
  metrics_.prev_rate_time = std::chrono::steady_clock::now();
}

CamHubStreamProcessor::MetricsSnapshot CamHubStreamProcessor::snapshot_metrics()
{
  std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  const double seconds = std::chrono::duration<double>(
    now_steady - metrics_.prev_rate_time).count();
  if (seconds >= 0.1) {
    for (std::size_t index = 0; index < kStreamCount; ++index) {
      metrics_.rates[index] = static_cast<float>(
        (metrics_.published_frames[index] - metrics_.prev_published_frames[index]) / seconds);
      metrics_.prev_published_frames[index] = metrics_.published_frames[index];
    }
    metrics_.prev_rate_time = now_steady;
  }

  MetricsSnapshot snapshot;
  snapshot.rates = metrics_.rates;
  snapshot.publish_latency_ms = metrics_.publish_latency_ms;
  snapshot.lerobot_skew_p50_ms = static_cast<float>(percentile(metrics_.skew_samples_ms, 0.50));
  snapshot.lerobot_skew_p95_ms = static_cast<float>(percentile(metrics_.skew_samples_ms, 0.95));
  snapshot.lerobot_triplet_match_count = metrics_.lerobot_triplet_match_count;
  snapshot.lerobot_triplet_discarded_frame_count = metrics_.lerobot_triplet_discarded_frame_count;
  return snapshot;
}
void CamHubStreamProcessor::handle_sample(
  StreamId stream,
  GstAppSink * sink,
  GstElement * pipeline)
{
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (stream == StreamId::kStereoLeft) {
    publish_hailo(sample, pipeline);
  } else {
    publish_rgb(stream, sample, pipeline);
  }
}

void CamHubStreamProcessor::publish_hailo(GstSample * sample, GstElement * pipeline)
{
  GstMappedSample mapped(sample);
  if (!mapped.valid()) {
    return;
  }

  GstCaps * caps = gst_sample_get_caps(sample);
  GstVideoInfo video_info;
  gst_video_info_init(&video_info);
  if (caps == nullptr || !gst_video_info_from_caps(&video_info, caps)) {
    RCLCPP_ERROR_THROTTLE(
      bindings_.logger, *bindings_.clock, 5000, "Hailo appsink has invalid video caps");
    return;
  }

  const auto roi = get_hailo_main_roi(mapped.buffer());
  hailo::ChessboardState state;
  if (!roi || !hailo::ChessVisionMapper::decode_hailo_metadata(roi, state)) {
    return;
  }

  const auto stamp = sample_stamp(sample, pipeline);
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = bindings_.streams[stream_index(StreamId::kStereoLeft)].frame_id;

  if (!state.fen.empty()) {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = state.fen;
    bindings_.fen_pub->publish(std::move(message));
    if (state.fen != last_logged_fen_) {
      last_logged_fen_ = state.fen;
      RCLCPP_INFO(
        bindings_.logger, "Board state changed: %d pieces | FEN: %s",
        state.num_pieces, state.fen.c_str());
    }
  }

  auto detections = std::make_unique<vision_msgs::msg::Detection2DArray>();
  detections->header = header;
  const double width = static_cast<double>(GST_VIDEO_INFO_WIDTH(&video_info));
  const double height = static_cast<double>(GST_VIDEO_INFO_HEIGHT(&video_info));
  for (const auto & piece : state.pieces) {
    vision_msgs::msg::Detection2D detection;
    detection.header = header;
    detection.bbox.center.position.x =
      static_cast<double>(piece.bbox.x + piece.bbox.width / 2.0F) * width;
    detection.bbox.center.position.y =
      static_cast<double>(piece.bbox.y + piece.bbox.height / 2.0F) * height;
    detection.bbox.size_x = static_cast<double>(piece.bbox.width) * width;
    detection.bbox.size_y = static_cast<double>(piece.bbox.height) * height;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = piece.label;
    hypothesis.hypothesis.score = piece.confidence;
    detection.results.push_back(std::move(hypothesis));
    detections->detections.push_back(std::move(detection));
  }
  bindings_.detections_pub->publish(std::move(detections));

  const std::size_t debug_subscribers = bindings_.debug_image_pub ?
    bindings_.debug_image_pub->get_subscription_count() +
    bindings_.debug_image_pub->get_intra_process_subscription_count() : 0U;
  if (debug_subscribers > 0U) {
    cv::Mat debug_bgr;
    if (!make_debug_bgr(video_info, mapped.data(), mapped.size(), debug_bgr)) {
      RCLCPP_ERROR_THROTTLE(
        bindings_.logger, *bindings_.clock, 5000, "Hailo debug appsink did not negotiate RGB");
    } else {
      hailo::ChessVisionMapper::draw_chessboard_overlay(
        debug_bgr, state.grid_points_norm, state.poly_points_norm);
      hailo::ChessVisionMapper::draw_piece_detections(debug_bgr, state.pieces);

      auto debug = std::make_unique<sensor_msgs::msg::Image>();
      debug->header = header;
      debug->height = static_cast<uint32_t>(debug_bgr.rows);
      debug->width = static_cast<uint32_t>(debug_bgr.cols);
      debug->encoding = "bgr8";
      debug->is_bigendian = false;
      debug->step = static_cast<uint32_t>(debug_bgr.cols * 3);
      debug->data.assign(debug_bgr.datastart, debug_bgr.dataend);
      bindings_.debug_image_pub->publish(std::move(debug));
      publish_debug_camera_info(stamp);
    }
  }

  record_published_frame(StreamId::kStereoLeft, rclcpp::Time(stamp).nanoseconds());
}

void CamHubStreamProcessor::publish_rgb(
  StreamId stream,
  GstSample * sample,
  GstElement * pipeline)
{
  GstMappedSample mapped(sample);
  if (!mapped.valid()) {
    return;
  }

  GstCaps * caps = gst_sample_get_caps(sample);
  GstVideoInfo video_info;
  gst_video_info_init(&video_info);
  if (caps == nullptr || !gst_video_info_from_caps(&video_info, caps) ||
    GST_VIDEO_INFO_FORMAT(&video_info) != GST_VIDEO_FORMAT_RGB)
  {
    RCLCPP_ERROR_THROTTLE(
      bindings_.logger, *bindings_.clock, 5000, "%s appsink did not negotiate RGB", stream_name(stream));
    return;
  }

  const auto stamp = sample_stamp(sample, pipeline);
  const auto width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&video_info));
  const auto height = static_cast<uint32_t>(GST_VIDEO_INFO_HEIGHT(&video_info));
  const int src_stride = GST_VIDEO_INFO_PLANE_STRIDE(&video_info, 0);
  const auto src_offset = static_cast<std::size_t>(GST_VIDEO_INFO_PLANE_OFFSET(&video_info, 0));
  const auto target_stride = width * 3U;
  if (src_stride < static_cast<int>(target_stride) ||
    src_offset + static_cast<std::size_t>(src_stride) * height > mapped.size())
  {
    RCLCPP_ERROR_THROTTLE(bindings_.logger, *bindings_.clock, 5000, "Rejected invalid RGB layout");
    return;
  }

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header.stamp = stamp;
  message->header.frame_id = bindings_.streams[stream_index(stream)].frame_id;
  message->width = width;
  message->height = height;
  message->encoding = "rgb8";
  message->is_bigendian = false;
  message->step = target_stride;
  message->data.resize(static_cast<std::size_t>(target_stride) * height);
  const uint8_t * source = mapped.data() + src_offset;
  for (uint32_t row = 0; row < height; ++row) {
    std::memcpy(
      message->data.data() + static_cast<std::size_t>(row) * target_stride,
      source + static_cast<std::size_t>(row) * src_stride,
      target_stride);
  }

  bindings_.image_pubs[stream_index(stream)]->publish(std::move(message));
  record_published_frame(stream, rclcpp::Time(stamp).nanoseconds());
  publish_camera_info(stream, stamp);
}

builtin_interfaces::msg::Time CamHubStreamProcessor::sample_stamp(
  GstSample * sample,
  GstElement * pipeline) const
{
  GstClockTime absolute_time = GST_CLOCK_TIME_NONE;
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (buffer != nullptr && GST_BUFFER_PTS_IS_VALID(buffer)) {
    const GstSegment * segment = gst_sample_get_segment(sample);
    GstClockTime running_time = GST_BUFFER_PTS(buffer);
    if (segment != nullptr && segment->format == GST_FORMAT_TIME) {
      running_time = gst_segment_to_running_time(
        segment, GST_FORMAT_TIME, GST_BUFFER_PTS(buffer));
    }
    const GstClockTime base_time = gst_element_get_base_time(pipeline);
    if (GST_CLOCK_TIME_IS_VALID(running_time) && GST_CLOCK_TIME_IS_VALID(base_time)) {
      absolute_time = base_time + running_time;
    }
  }
  if (!GST_CLOCK_TIME_IS_VALID(absolute_time)) {
    return bindings_.clock->now();
  }

  const int64_t stamp_ns = static_cast<int64_t>(absolute_time) + bindings_.gst_to_ros_offset_ns;
  if (stamp_ns <= 0) {
    return bindings_.clock->now();
  }
  return static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(stamp_ns, RCL_SYSTEM_TIME));
}

void CamHubStreamProcessor::publish_camera_info(
  StreamId stream,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto index = stream_index(stream);
  auto message = std::make_unique<sensor_msgs::msg::CameraInfo>(bindings_.camera_infos[index]);
  message->header.stamp = stamp;
  bindings_.info_pubs[index]->publish(std::move(message));
}

void CamHubStreamProcessor::publish_debug_camera_info(
  const builtin_interfaces::msg::Time & stamp)
{
  auto message = std::make_unique<sensor_msgs::msg::CameraInfo>(
    bindings_.camera_infos[stream_index(StreamId::kStereoLeft)]);
  message->header.stamp = stamp;
  bindings_.debug_info_pub->publish(std::move(message));
}

void CamHubStreamProcessor::record_published_frame(StreamId stream, int64_t stamp_ns)
{
  double skew_ms = 0.0;
  bool warn_skew = false;
  {
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
    const auto index = stream_index(stream);
    ++metrics_.published_frames[index];
    metrics_.publish_latency_ms[index] = static_cast<float>(std::max<int64_t>(
        0, bindings_.clock->now().nanoseconds() - stamp_ns) / 1'000'000.0);

    if (stream != StreamId::kStereoLeft) {
      const auto lerobot_index = index - stream_index(StreamId::kStereoRight);
      auto & pending = metrics_.lerobot_pending[lerobot_index];
      pending.push_back(stamp_ns);
      if (pending.size() > kTripletQueueCapacity) {
        pending.pop_front();
        ++metrics_.lerobot_triplet_discarded_frame_count;
      }
      while (std::all_of(
          metrics_.lerobot_pending.begin(), metrics_.lerobot_pending.end(),
          [](const auto & queue) {return !queue.empty();}))
      {
        std::size_t oldest = 0U;
        int64_t minimum = metrics_.lerobot_pending[0].front();
        int64_t maximum = minimum;
        for (std::size_t queue = 1U; queue < kLeRobotStreamCount; ++queue) {
          const int64_t value = metrics_.lerobot_pending[queue].front();
          if (value < minimum) {
            minimum = value;
            oldest = queue;
          }
          maximum = std::max(maximum, value);
        }
        skew_ms = static_cast<double>(maximum - minimum) / 1'000'000.0;
        if (skew_ms > bindings_.skew_warning_ms) {
          metrics_.lerobot_pending[oldest].pop_front();
          ++metrics_.lerobot_triplet_discarded_frame_count;
          warn_skew = true;
          continue;
        }
        for (auto & queue : metrics_.lerobot_pending) {
          queue.pop_front();
        }
        ++metrics_.lerobot_triplet_match_count;
        metrics_.skew_samples_ms.push_back(skew_ms);
        if (metrics_.skew_samples_ms.size() > 600U) {
          metrics_.skew_samples_ms.erase(metrics_.skew_samples_ms.begin());
        }
      }
    }
  }
  if (warn_skew) {
    RCLCPP_WARN_THROTTLE(
      bindings_.logger, *bindings_.clock, 5000,
      "LeRobot frame exceeded %.2f ms triplet skew tolerance", bindings_.skew_warning_ms);
  }
}


}  // namespace lekiwi_perception

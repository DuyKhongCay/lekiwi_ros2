// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

/// @file cam_hub_stream_processor.cpp
/// @brief Stream processor for the LeKiwi multi-camera hub GStreamer pipeline.
///
/// Handles incoming GstSamples from each camera stream (stereo left/right,
/// gripper, top) and routes them to the appropriate publishing path:
///   - Stereo-left → Hailo AI inference path (chess piece detection + FEN)
///   - All others  → Raw RGB image publishing path
///
/// Each published frame is timestamped by converting GStreamer pipeline clock
/// to ROS system time, and per-stream metrics (frame count, latency, skew)
/// are tracked for diagnostics.

#include "cam_hub_stream_processor.hpp"

#include <gst/video/video.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <memory>
#include <utility>

#include "gst_hailo_meta.hpp"
#include "gstreamer/gst_mapped_sample.hpp"
#include "hailo/chess_vision_mapper.hpp"
#include "rclcpp/qos.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

namespace lekiwi_perception
{

/// @brief Construct the stream processor with pre-wired ROS publishers and config.
/// @param bindings  Aggregated struct of logger, clock, publishers, camera info,
///                  and timing calibration parameters — transferred by move.
CamHubStreamProcessor::CamHubStreamProcessor(Bindings bindings)
: bindings_(std::move(bindings))
{
  reset_metrics();
}

/// @brief Zero-out all per-stream frame counters and latency accumulators.
void CamHubStreamProcessor::reset_metrics()
{
  metrics_tracker_.reset();
}

/// @brief Return a snapshot of current per-stream metrics (frame count, fps,
///        average latency, maximum skew) without resetting them.
MetricsSnapshot CamHubStreamProcessor::snapshot_metrics()
{
  return metrics_tracker_.snapshot();
}

/// @brief Top-level entry point called from the GStreamer appsink "new-sample"
///        callback for every camera frame.
///
/// Routes the sample based on stream identity:
///   - kStereoLeft → publish_hailo()  (AI inference results + optional debug overlay)
///   - All others  → publish_rgb()    (raw RGB sensor_msgs::Image)
///
/// @param stream    Identity of the originating camera stream.
/// @param sink      GStreamer appsink element that produced the sample.
/// @param pipeline  Top-level GstPipeline, needed for clock base-time lookup.
void CamHubStreamProcessor::handle_sample(
  StreamId stream,
  GstAppSink * sink,
  GstElement * pipeline)
{
  // Pull the latest sample buffer from the appsink queue
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (stream == StreamId::kStereoLeft) {
    publish_hailo(sample, pipeline);
  } else {
    publish_rgb(stream, sample, pipeline);
  }
}

/// @brief Process a stereo-left frame through the Hailo AI inference path.
///
/// Workflow:
///  1. Map the GstSample buffer to CPU-accessible memory.
///  2. Extract Hailo ROI metadata attached by the inference element.
///  3. Decode the metadata into a ChessboardState (FEN string + piece bboxes).
///  4. Publish the FEN string on the dedicated topic (only when it changes).
///  5. Convert pieces to a Detection2DArray with pixel-space bounding boxes.
///  6. If any subscriber is listening on the debug image topic, render a BGR
///     overlay showing the detected chessboard grid and piece bounding boxes.
///
/// @param sample    GstSample pulled from the Hailo appsink.
/// @param pipeline  Top-level pipeline for timestamp base-time conversion.
void CamHubStreamProcessor::publish_hailo(GstSample * sample, GstElement * pipeline)
{
  // --- Step 1: Map the sample buffer for read access ---
  GstMappedSample mapped(sample);
  if (!mapped.valid()) {
    return;
  }

  // --- Step 2: Parse video caps to get frame dimensions ---
  GstCaps * caps = gst_sample_get_caps(sample);
  GstVideoInfo video_info;
  gst_video_info_init(&video_info);
  if (caps == nullptr || !gst_video_info_from_caps(&video_info, caps)) {
    RCLCPP_ERROR_THROTTLE(
      bindings_.logger, *bindings_.clock, 5000, "Hailo appsink has invalid video caps");
    return;
  }

  // --- Step 3: Extract Hailo inference metadata from the GstBuffer ---
  // get_hailo_main_roi() returns the top-level Region-of-Interest containing
  // all detection sub-ROIs attached by the Hailo post-processing element.
  const auto roi = get_hailo_main_roi(mapped.buffer());
  hailo::ChessboardState state;
  if (!roi || !hailo::ChessVisionMapper::decode_hailo_metadata(roi, state)) {
    return;  // No valid inference results in this frame
  }

  // --- Step 4: Build a common header (timestamp + frame_id) ---
  const auto stamp = sample_stamp(sample, pipeline);
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = bindings_.streams[stream_index(StreamId::kStereoLeft)].frame_id;

  // --- Step 5: Publish FEN string when board state changes ---
  if (!state.fen.empty()) {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = state.fen;
    bindings_.fen_pub->publish(std::move(message));
    // Log only on state transition to avoid flooding the console
    if (state.fen != last_logged_fen_) {
      last_logged_fen_ = state.fen;
      RCLCPP_INFO(
        bindings_.logger, "Board state changed: %d pieces | FEN: %s",
        state.num_pieces, state.fen.c_str());
    }
  }

  // --- Step 6: Convert piece detections to Detection2DArray ---
  // Bounding boxes from Hailo are normalised [0,1]; we scale them to pixel
  // coordinates so downstream nodes receive absolute positions directly.
  auto detections = std::make_unique<vision_msgs::msg::Detection2DArray>();
  detections->header = header;
  const double width = static_cast<double>(GST_VIDEO_INFO_WIDTH(&video_info));
  const double height = static_cast<double>(GST_VIDEO_INFO_HEIGHT(&video_info));
  for (const auto & piece : state.pieces) {
    vision_msgs::msg::Detection2D detection;
    detection.header = header;
    // Center of the bounding box in pixel coordinates
    detection.bbox.center.position.x =
      static_cast<double>(piece.bbox.x + piece.bbox.width / 2.0F) * width;
    detection.bbox.center.position.y =
      static_cast<double>(piece.bbox.y + piece.bbox.height / 2.0F) * height;
    // Full width/height of the bounding box in pixels
    detection.bbox.size_x = static_cast<double>(piece.bbox.width) * width;
    detection.bbox.size_y = static_cast<double>(piece.bbox.height) * height;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = piece.label;       // e.g. "wK", "bP"
    hypothesis.hypothesis.score = piece.confidence;      // [0.0, 1.0]
    detection.results.push_back(std::move(hypothesis));
    detections->detections.push_back(std::move(detection));
  }
  bindings_.detections_pub->publish(std::move(detections));

  // --- Step 7: Publish debug overlay image (lazy — only if subscribers exist) ---
  const std::size_t debug_subscribers = bindings_.debug_image_pub ?
    bindings_.debug_image_pub->get_subscription_count() +
    bindings_.debug_image_pub->get_intra_process_subscription_count() : 0U;
  if (debug_subscribers > 0U) {
    // Convert the raw buffer to a BGR cv::Mat for OpenCV drawing operations
    cv::Mat debug_bgr;
    if (!make_debug_bgr(video_info, mapped.data(), mapped.size(), debug_bgr)) {
      RCLCPP_ERROR_THROTTLE(
        bindings_.logger, *bindings_.clock, 5000, "Hailo debug appsink did not negotiate RGB");
    } else {
      // Draw the detected chessboard grid and bounding boxes onto the image
      hailo::ChessVisionMapper::draw_chessboard_overlay(
        debug_bgr, state.grid_points_norm, state.poly_points_norm);
      hailo::ChessVisionMapper::draw_piece_detections(debug_bgr, state.pieces);

      // Wrap the cv::Mat into a sensor_msgs::Image and publish
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

  // Track this frame in the per-stream metrics (fps, latency, skew)
  record_published_frame(StreamId::kStereoLeft, rclcpp::Time(stamp).nanoseconds());
}

/// @brief Publish a raw RGB camera frame as a sensor_msgs::msg::Image.
///
/// Used for all non-Hailo streams (stereo-right, gripper, top).
/// The GStreamer buffer may have padding bytes at the end of each row
/// (src_stride > width*3), so we perform a row-by-row copy to produce
/// a tightly-packed ROS Image message.
///
/// @param stream    Camera stream identity (determines which publisher to use).
/// @param sample    GstSample pulled from the appsink.
/// @param pipeline  Top-level pipeline for timestamp base-time conversion.
void CamHubStreamProcessor::publish_rgb(
  StreamId stream,
  GstSample * sample,
  GstElement * pipeline)
{
  // Map the GstSample buffer for CPU read access
  GstMappedSample mapped(sample);
  if (!mapped.valid()) {
    return;
  }

  // Validate that the negotiated format is RGB (not NV12, YUV, etc.)
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

  // Extract frame dimensions and stride layout
  const auto stamp = sample_stamp(sample, pipeline);
  const auto width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&video_info));
  const auto height = static_cast<uint32_t>(GST_VIDEO_INFO_HEIGHT(&video_info));
  const int src_stride = GST_VIDEO_INFO_PLANE_STRIDE(&video_info, 0);       // bytes per row (may include padding)
  const auto src_offset = static_cast<std::size_t>(GST_VIDEO_INFO_PLANE_OFFSET(&video_info, 0));
  const auto target_stride = width * 3U;                                     // tightly-packed RGB: 3 bytes/pixel

  // Sanity check: ensure the source layout is large enough
  if (src_stride < static_cast<int>(target_stride) ||
    src_offset + static_cast<std::size_t>(src_stride) * height > mapped.size())
  {
    RCLCPP_ERROR_THROTTLE(bindings_.logger, *bindings_.clock, 5000, "Rejected invalid RGB layout");
    return;
  }

  // Build the sensor_msgs::Image with tightly-packed row data
  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header.stamp = stamp;
  message->header.frame_id = bindings_.streams[stream_index(stream)].frame_id;
  message->width = width;
  message->height = height;
  message->encoding = "rgb8";
  message->is_bigendian = false;
  message->step = target_stride;
  message->data.resize(static_cast<std::size_t>(target_stride) * height);

  // Row-by-row copy: strip any padding bytes between GStreamer rows
  const uint8_t * source = mapped.data() + src_offset;
  for (uint32_t row = 0; row < height; ++row) {
    std::memcpy(
      message->data.data() + static_cast<std::size_t>(row) * target_stride,
      source + static_cast<std::size_t>(row) * src_stride,
      target_stride);
  }

  // Publish the image and its corresponding CameraInfo
  bindings_.image_pubs[stream_index(stream)]->publish(std::move(message));
  record_published_frame(stream, rclcpp::Time(stamp).nanoseconds());
  publish_camera_info(stream, stamp);
}

/// @brief Convert a GstSample's PTS (presentation timestamp) to a ROS Time.
///
/// Timestamp conversion pipeline:
///   GstBuffer PTS  →  running_time (via segment)  →  absolute_time (+ base_time)
///   →  ROS system time (+ gst_to_ros_offset_ns calibration offset)
///
/// Falls back to rclcpp::Clock::now() if the buffer has no valid PTS or if the
/// resulting timestamp is non-positive (e.g. before clock calibration completes).
///
/// @param sample    GstSample containing the buffer with a PTS.
/// @param pipeline  GstPipeline whose base_time anchors the running clock.
/// @return          Equivalent builtin_interfaces::msg::Time in ROS system time.
builtin_interfaces::msg::Time CamHubStreamProcessor::sample_stamp(
  GstSample * sample,
  GstElement * pipeline) const
{
  GstClockTime absolute_time = GST_CLOCK_TIME_NONE;
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (buffer != nullptr && GST_BUFFER_PTS_IS_VALID(buffer)) {
    // Convert buffer PTS to running_time using the sample's segment mapping.
    // If no segment is available, fall back to using PTS directly.
    const GstSegment * segment = gst_sample_get_segment(sample);
    GstClockTime running_time = GST_BUFFER_PTS(buffer);
    if (segment != nullptr && segment->format == GST_FORMAT_TIME) {
      running_time = gst_segment_to_running_time(
        segment, GST_FORMAT_TIME, GST_BUFFER_PTS(buffer));
    }
    // Add the pipeline's base_time to get an absolute GStreamer clock value
    const GstClockTime base_time = gst_element_get_base_time(pipeline);
    if (GST_CLOCK_TIME_IS_VALID(running_time) && GST_CLOCK_TIME_IS_VALID(base_time)) {
      absolute_time = base_time + running_time;
    }
  }
  // Fallback: use wall-clock if GStreamer timestamp is unavailable
  if (!GST_CLOCK_TIME_IS_VALID(absolute_time)) {
    return bindings_.clock->now();
  }

  // Apply the pre-computed offset that aligns GStreamer's monotonic clock
  // with the ROS system (wall) clock. This offset is calibrated once at
  // pipeline startup by sampling both clocks simultaneously.
  const int64_t stamp_ns = static_cast<int64_t>(absolute_time) + bindings_.gst_to_ros_offset_ns;
  if (stamp_ns <= 0) {
    return bindings_.clock->now();  // Safety: reject negative / pre-epoch stamps
  }
  return static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(stamp_ns, RCL_SYSTEM_TIME));
}

/// @brief Publish a CameraInfo message for a given camera stream.
///
/// Clones the pre-loaded CameraInfo template (intrinsics, distortion, etc.)
/// and stamps it with the current frame's timestamp before publishing.
///
/// @param stream  Camera stream identity.
/// @param stamp   Timestamp matching the associated Image message.
void CamHubStreamProcessor::publish_camera_info(
  StreamId stream,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto index = stream_index(stream);
  auto message = std::make_unique<sensor_msgs::msg::CameraInfo>(bindings_.camera_infos[index]);
  message->header.stamp = stamp;
  bindings_.info_pubs[index]->publish(std::move(message));
}

/// @brief Publish a CameraInfo message for the debug overlay image.
///
/// Uses the stereo-left camera's intrinsics since the debug overlay is
/// derived from the stereo-left Hailo inference frame.
///
/// @param stamp  Timestamp matching the associated debug Image message.
void CamHubStreamProcessor::publish_debug_camera_info(
  const builtin_interfaces::msg::Time & stamp)
{
  auto message = std::make_unique<sensor_msgs::msg::CameraInfo>(
    bindings_.camera_infos[stream_index(StreamId::kStereoLeft)]);
  message->header.stamp = stamp;
  bindings_.debug_info_pub->publish(std::move(message));
}

/// @brief Record frame timing in the per-stream metrics tracker and emit a
///        throttled warning if the inter-stream timestamp skew exceeds the
///        configured tolerance.
///
/// This is called after every successful publish_hailo() or publish_rgb().
/// The metrics tracker accumulates frame counts, computes FPS, and detects
/// when the timestamp gap between the most recent frames of different streams
/// exceeds skew_warning_ms — indicating a synchronisation problem.
///
/// @param stream    Stream identity of the just-published frame.
/// @param stamp_ns  Frame timestamp in nanoseconds (ROS system time).
void CamHubStreamProcessor::record_published_frame(StreamId stream, int64_t stamp_ns)
{
  bool warn_skew = false;
  metrics_tracker_.record_frame(
    stream, stamp_ns, bindings_.clock->now().nanoseconds(),
    bindings_.skew_warning_ms, warn_skew);
  if (warn_skew) {
    RCLCPP_WARN_THROTTLE(
      bindings_.logger, *bindings_.clock, 5000,
      "LeRobot frame exceeded %.2f ms triplet skew tolerance", bindings_.skew_warning_ms);
  }
}

}  // namespace lekiwi_perception

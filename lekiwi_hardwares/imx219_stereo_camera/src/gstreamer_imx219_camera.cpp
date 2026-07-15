// Copyright 2026 LeKiwi Labs. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// GStreamer-based IMX219 Stereo Camera Node
//
// This node replaces the direct libcamera C++ API with a GStreamer pipeline
// using the "libcamerasrc" plugin. Two separate GStreamer pipelines are created,
// one per camera, and frames are delivered via the GstAppSink "new-sample" signal.
//
// Hardware synchronization is attempted via the "extra-controls" property of
// libcamerasrc (passes rpi SyncMode control to the underlying libcamera runtime).
//
// Camera selection is done via "camera-index" property (integer) instead of
// operating on libcamera::Camera pointers directly.
// ============================================================================

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <queue>
#include <condition_variable>
#include <thread>
#include <cmath>
#include <algorithm>
#include <functional>
#include <unistd.h>
#include <cstdlib>
#include <numeric>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <sstream>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

// OpenCV Headers
#include <opencv2/opencv.hpp>

// GStreamer Headers
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

// Base Class
#include "imx219_stereo_camera/base_camera_node.hpp"

using namespace std::chrono_literals;

namespace lekiwi_cameras {

// Hardware-specific constants for the IMX219 sensor
namespace imx219_gst_hardware {
  constexpr int  SENSOR_WIDTH    = 1640;
  constexpr int  SENSOR_HEIGHT   = 1232;
  constexpr int  BIT_DEPTH       = 10;
  // TF frame IDs fixed to the robot's mechanical design
  constexpr char LEFT_FRAME_ID[]  = "stereo_left_optical";
  constexpr char RIGHT_FRAME_ID[] = "stereo_right_optical";
  // Empty URL = no calibration file; node will publish an empty CameraInfo
  constexpr char LEFT_INFO_URL[]  = "";
  constexpr char RIGHT_INFO_URL[] = "";
  // libcamera Raspberry Pi sync mode values (mirrors controls::rpi::SyncMode enum)
  constexpr int SYNC_MODE_SERVER = 1;
  constexpr int SYNC_MODE_CLIENT = 2;
}

// Configurable runtime settings of the stereo camera node
struct CameraConfig {
  std::string left_camera_name;
  std::string right_camera_name;
  int         width;
  int         height;
  double      fps;
  int         rotation;
  bool        publish_raw;
  bool        publish_compressed;
  bool        enable_benchmark;
  std::string benchmark_csv_path;
  bool        enable_hw_sync;
};

// Benchmark helper class to track latency, FPS, drop rate and jitter.
// Uses CLOCK_MONOTONIC because GStreamer pipeline clock is based on monotonic time.
class CameraBenchmark {
public:
  CameraBenchmark() {
    reset();
  }

  ~CameraBenchmark() {
    closeCSV();
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    last_publish_time_ = std::chrono::steady_clock::time_point();
    published_count_ = 0;
    dropped_queue_count_ = 0;
    dropped_sync_count_ = 0;
    latencies_.clear();
    jitters_.clear();
    start_time_ = std::chrono::steady_clock::now();
  }

  void openCSV(const std::string &filepath) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
    csv_file_.open(filepath, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
      csv_file_ << "frame_index,gst_pts_ns,latency_ms,jitter_ms\n";
    }
  }

  void closeCSV() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

  // Record a published frame. gst_pts_ns is the GStreamer PTS from the buffer.
  // Latency is computed as the difference between CLOCK_MONOTONIC now and PTS.
  void recordPublish(uint64_t gst_pts_ns) {
    auto now_steady = std::chrono::steady_clock::now();

    // Read CLOCK_MONOTONIC to correlate with GStreamer pipeline timestamps
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_mono_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;

    double latency_ms = 0.0;
    // Only compute latency when PTS is valid (not GST_CLOCK_TIME_NONE)
    if (gst_pts_ns != static_cast<uint64_t>(-1) && now_mono_ns >= gst_pts_ns) {
      latency_ms = static_cast<double>(now_mono_ns - gst_pts_ns) / 1000000.0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latencies_.push_back(latency_ms);

    double jitter_ms = 0.0;
    if (last_publish_time_.time_since_epoch().count() > 0) {
      jitter_ms = std::chrono::duration<double, std::milli>(now_steady - last_publish_time_).count();
      jitters_.push_back(jitter_ms);
    }
    last_publish_time_ = now_steady;
    published_count_++;

    if (csv_file_.is_open()) {
      csv_file_ << published_count_ << "," << gst_pts_ns << ","
                << std::fixed << std::setprecision(3) << latency_ms << ","
                << jitter_ms << "\n";
    }
  }

  void recordQueueDrop() {
    std::lock_guard<std::mutex> lock(mutex_);
    dropped_queue_count_++;
  }

  void recordSyncDrop() {
    std::lock_guard<std::mutex> lock(mutex_);
    dropped_sync_count_++;
  }

  bool shouldReport(double report_interval_sec = 5.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();
    return elapsed >= report_interval_sec;
  }

  void report(rclcpp::Logger logger, double target_fps) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed <= 0.0) return;

    double actual_fps = published_count_ / elapsed;
    uint64_t total_dropped = dropped_queue_count_ + dropped_sync_count_;
    uint64_t total_frames = published_count_ + total_dropped;
    double drop_rate = total_frames > 0 ?
        (static_cast<double>(total_dropped) / total_frames) * 100.0 : 0.0;
    double sync_drop_rate = total_frames > 0 ?
        (static_cast<double>(dropped_sync_count_) / total_frames) * 100.0 : 0.0;
    double queue_drop_rate = total_frames > 0 ?
        (static_cast<double>(dropped_queue_count_) / total_frames) * 100.0 : 0.0;

    double avg_latency = 0.0, max_latency = 0.0;
    if (!latencies_.empty()) {
      double sum = std::accumulate(latencies_.begin(), latencies_.end(), 0.0);
      avg_latency = sum / latencies_.size();
      max_latency = *std::max_element(latencies_.begin(), latencies_.end());
    }

    double jitter_std_dev = 0.0;
    double expected_period_ms = 1000.0 / target_fps;
    if (!jitters_.empty()) {
      double sum = std::accumulate(jitters_.begin(), jitters_.end(), 0.0);
      double mean_period = sum / jitters_.size();
      double sq_sum = 0.0;
      for (double val : jitters_) {
        sq_sum += (val - mean_period) * (val - mean_period);
      }
      jitter_std_dev = std::sqrt(sq_sum / jitters_.size());
    }

    RCLCPP_INFO(logger,
      "\n============= CAMERA BENCHMARK REPORT (%.1fs) =============\n"
      "  Target FPS: %.1f | Actual FPS: %.2f\n"
      "  Published: %lu | Total Dropped: %lu (Sync drop: %lu, Queue drop: %lu)\n"
      "  Drop Rate: %.2f%% (Sync: %.2f%%, Queue: %.2f%%)\n"
      "  Latency (End-to-End): Avg: %.2f ms | Max: %.2f ms\n"
      "  Jitter (Std Dev of Period): %.2f ms (Target period: %.2f ms)\n"
      "===========================================================",
      elapsed, target_fps, actual_fps, published_count_, total_dropped,
      dropped_sync_count_, dropped_queue_count_, drop_rate, sync_drop_rate,
      queue_drop_rate, avg_latency, max_latency, jitter_std_dev, expected_period_ms);

    // Reset statistics after reporting
    last_publish_time_ = std::chrono::steady_clock::time_point();
    published_count_ = 0;
    dropped_queue_count_ = 0;
    dropped_sync_count_ = 0;
    latencies_.clear();
    jitters_.clear();
    start_time_ = std::chrono::steady_clock::now();
  }

private:
  std::mutex mutex_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  uint64_t published_count_;
  uint64_t dropped_queue_count_;
  uint64_t dropped_sync_count_;
  std::vector<double> latencies_;
  std::vector<double> jitters_;
  std::ofstream csv_file_;
};

// Struct to hold frames temporarily for software timestamp synchronization
struct PendingFrame {
  cv::Mat image;
  uint64_t timestamp_ns;
};

// GstSingleCamera manages one GStreamer pipeline per physical camera.
// The pipeline uses libcamerasrc (which wraps libcamera runtime internally)
// and delivers frames to a GstAppSink via the "new-sample" signal.
class GstSingleCamera {
public:
  using FrameCallback = std::function<void(const cv::Mat &image, uint64_t timestamp_ns)>;

  GstSingleCamera(
    rclcpp::Node * node,
    const std::string & camera_name,
    int width,
    int height,
    double fps,
    int rotation,
    int sync_mode,
    bool enable_hw_sync,
    FrameCallback callback)
  : node_(node),
    camera_name_(camera_name),
    width_(width),
    height_(height),
    fps_(fps),
    rotation_(rotation),
    sync_mode_(sync_mode),
    enable_hw_sync_(enable_hw_sync),
    callback_(callback),
    pipeline_(nullptr),
    appsink_(nullptr),
    is_initialized_(false),
    is_started_(false)
  {}

  ~GstSingleCamera() {
    stop();
  }

  // Build and initialize the GStreamer pipeline.
  // Returns true on success, false otherwise.
  bool initialize() {
    if (is_initialized_) return true;

    // Build pipeline description string:
    // libcamerasrc produces BGRx frames (ISP output),
    // videoconvert converts to BGR (CV_8UC3 compatible).
    std::ostringstream pipeline_ss;
    pipeline_ss
      << "libcamerasrc name=src camera-name=\"" << camera_name_ << "\" ! "
      << "video/x-raw,format=BGRx"
      << ",width=" << width_
      << ",height=" << height_
      << ",framerate=" << static_cast<int>(fps_) << "/1 ! "
      << "videoconvert ! "
      << "video/x-raw,format=BGR ! "
      << "appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false";

    const std::string pipeline_str = pipeline_ss.str();
    RCLCPP_INFO(node_->get_logger(),
      "[GstSingleCamera %s] Pipeline: %s", camera_name_.c_str(), pipeline_str.c_str());

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline_ || error) {
      RCLCPP_ERROR(node_->get_logger(),
        "[GstSingleCamera %s] Failed to create pipeline: %s",
        camera_name_.c_str(), error ? error->message : "unknown error");
      if (error) g_error_free(error);
      return false;
    }

    // Retrieve the libcamerasrc element by name to set extra-controls
    GstElement * src_element = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (!src_element) {
      RCLCPP_ERROR(node_->get_logger(),
        "[GstSingleCamera %s] Could not find 'src' element in pipeline", camera_name_.c_str());
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      return false;
    }

    // Set sensor resolution for Full FOV (e.g. 1640x1232, 10-bit)
    GstStructure * sensor_cfg = gst_structure_new(
      "sensor-config",
      "width", G_TYPE_INT, imx219_gst_hardware::SENSOR_WIDTH,
      "height", G_TYPE_INT, imx219_gst_hardware::SENSOR_HEIGHT,
      "depth", G_TYPE_INT, imx219_gst_hardware::BIT_DEPTH,
      nullptr);
    if (sensor_cfg) {
      g_object_set(G_OBJECT(src_element), "sensor-config", sensor_cfg, nullptr);
      gst_structure_free(sensor_cfg);
      RCLCPP_INFO(node_->get_logger(),
        "[GstSingleCamera %s] Configured sensor resolution for Full FOV: %dx%d (%d-bit)",
        camera_name_.c_str(),
        imx219_gst_hardware::SENSOR_WIDTH,
        imx219_gst_hardware::SENSOR_HEIGHT,
        imx219_gst_hardware::BIT_DEPTH);
    }

    // Set hardware sync mode via extra-controls if enabled and supported.
    // The control key "rpi_SyncMode" is the GstStructure representation
    // of the Raspberry Pi-specific libcamera control rpi::SyncMode.
    if (enable_hw_sync_) {
      if (g_object_class_find_property(G_OBJECT_GET_CLASS(src_element), "extra-controls")) {
        GstStructure * controls = gst_structure_new(
          "controls",
          "rpi_SyncMode", G_TYPE_INT, sync_mode_,
          nullptr);
        if (controls) {
          g_object_set(G_OBJECT(src_element), "extra-controls", controls, nullptr);
          gst_structure_free(controls);
          RCLCPP_INFO(node_->get_logger(),
            "[GstSingleCamera %s] Hardware SyncMode set to %d", camera_name_.c_str(), sync_mode_);
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "[GstSingleCamera %s] Failed to create extra-controls structure", camera_name_.c_str());
        }
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "[GstSingleCamera %s] extra-controls property not supported by GStreamer plugin; "
          "hardware sync will not be applied", camera_name_.c_str());
      }
    }
    gst_object_unref(src_element);

    // Retrieve the appsink element by name
    GstElement * sink_element = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!sink_element) {
      RCLCPP_ERROR(node_->get_logger(),
        "[GstSingleCamera %s] Could not find 'sink' element in pipeline", camera_name_.c_str());
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      return false;
    }
    appsink_ = GST_APP_SINK(sink_element);

    // Connect new-sample signal to deliver frames to the callback
    g_signal_connect(appsink_, "new-sample",
      G_CALLBACK(GstSingleCamera::onNewSampleStatic), this);

    is_initialized_ = true;
    return true;
  }

  // Transition the pipeline to PLAYING state to start capturing
  bool start() {
    if (!is_initialized_) {
      RCLCPP_ERROR(node_->get_logger(),
        "[GstSingleCamera %s] Cannot start: not initialized", camera_name_.c_str());
      return false;
    }
    if (is_started_) return true;

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(node_->get_logger(),
        "[GstSingleCamera %s] Failed to set pipeline to PLAYING", camera_name_.c_str());
      return false;
    }
    is_started_ = true;
    RCLCPP_INFO(node_->get_logger(),
      "[GstSingleCamera %s] Pipeline started (PLAYING)", camera_name_.c_str());
    return true;
  }

  // Gracefully stop the pipeline and release all GStreamer resources
  void stop() {
    if (!is_initialized_) return;
    if (pipeline_) {
      // Send EOS downstream and wait briefly before forcing NULL state
      gst_element_send_event(pipeline_, gst_event_new_eos());
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      if (appsink_) {
        // appsink_ is owned by the pipeline bin; unreffing pipeline covers it
        appsink_ = nullptr;
      }
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    is_initialized_ = false;
    is_started_ = false;
    RCLCPP_INFO(node_->get_logger(),
      "[GstSingleCamera %s] Pipeline stopped and released", camera_name_.c_str());
  }

private:
  // Static trampoline required by GObject signal mechanism
  static GstFlowReturn onNewSampleStatic(GstAppSink * sink, gpointer user_data) {
    auto * self = static_cast<GstSingleCamera *>(user_data);
    return self->onNewSample(sink);
  }

  // Called by GStreamer in the streaming thread each time a new frame is ready
  GstFlowReturn onNewSample(GstAppSink * sink) {
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
      return GST_FLOW_ERROR;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    // Read the GStreamer Presentation Timestamp (PTS).
    // libcamerasrc sets PTS based on the libcamera hardware timestamp,
    // which starts at 0 (running time of the pipeline).
    // We add the pipeline base_time to reconstruct the absolute system CLOCK_MONOTONIC timestamp.
    uint64_t timestamp_ns = GST_BUFFER_PTS(buffer);
    if (GST_CLOCK_TIME_IS_VALID(timestamp_ns)) {
      GstClockTime base_time = gst_element_get_base_time(pipeline_);
      if (GST_CLOCK_TIME_IS_VALID(base_time)) {
        timestamp_ns += base_time;
      }
    } else {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      timestamp_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
    }

    // Map the GstBuffer into CPU-accessible memory (read-only)
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),
        "[GstSingleCamera %s] Failed to map GstBuffer", camera_name_.c_str());
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    // Create a cv::Mat referencing the mapped data directly (zero-copy view)
    cv::Mat raw_img(height_, width_, CV_8UC3, map_info.data);

    // Clone immediately so GstBuffer can be released as fast as possible
    cv::Mat cloned_img = raw_img.clone();

    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);

    // Deliver the cloned frame and its timestamp to the node callback
    callback_(cloned_img, timestamp_ns);

    return GST_FLOW_OK;
  }

  rclcpp::Node * node_;
  std::string camera_name_;
  int width_;
  int height_;
  double fps_;
  int rotation_;
  int sync_mode_;
  bool enable_hw_sync_;
  FrameCallback callback_;

  GstElement * pipeline_;
  GstAppSink * appsink_;
  bool is_initialized_;
  bool is_started_;
};

// FrameSynchronizer matches frames from left and right cameras based on timestamps.
// A matched pair is emitted when both cameras produce a frame within sync_threshold_ms.
class FrameSynchronizer {
public:
  enum MatchResult {
    MATCH_SUCCESS,
    AWAITING_FRAME,
    OUT_OF_SYNC_DISCARD
  };

  using MatchCallback = std::function<void(
      const cv::Mat &left_img, const cv::Mat &right_img, uint64_t timestamp_ns)>;
  using DropCallback = std::function<void()>;

  FrameSynchronizer(
    double sync_threshold_ms,
    MatchCallback match_callback,
    DropCallback drop_callback = nullptr)
  : sync_threshold_ms_(sync_threshold_ms),
    match_callback_(match_callback),
    drop_callback_(drop_callback)
  {}

  // Add a frame from the left (server/master) camera
  void addLeftFrame(const cv::Mat &image, uint64_t timestamp_ns) {
    auto new_frame = std::make_unique<PendingFrame>(PendingFrame{image, timestamp_ns});

    cv::Mat left, right;
    uint64_t matched_ts = 0;
    MatchResult res;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_left_ = std::move(new_frame);
      res = tryMatch(left, right, matched_ts);
    }

    if (res == MATCH_SUCCESS) {
      match_callback_(left, right, matched_ts);
    } else if (res == OUT_OF_SYNC_DISCARD) {
      if (drop_callback_) drop_callback_();
    }
  }

  // Add a frame from the right (client/slave) camera
  void addRightFrame(const cv::Mat &image, uint64_t timestamp_ns) {
    auto new_frame = std::make_unique<PendingFrame>(PendingFrame{image, timestamp_ns});

    cv::Mat left, right;
    uint64_t matched_ts = 0;
    MatchResult res;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_right_ = std::move(new_frame);
      res = tryMatch(left, right, matched_ts);
    }

    if (res == MATCH_SUCCESS) {
      match_callback_(left, right, matched_ts);
    } else if (res == OUT_OF_SYNC_DISCARD) {
      if (drop_callback_) drop_callback_();
    }
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_left_.reset();
    pending_right_.reset();
  }

private:
  // Must be called with mutex_ held.
  // Returns MATCH_SUCCESS and populates left/right/timestamp_ns if a pair is found.
  MatchResult tryMatch(cv::Mat &left, cv::Mat &right, uint64_t &timestamp_ns) {
    if (!pending_left_ || !pending_right_) {
      return AWAITING_FRAME;
    }

    int64_t diff_ns = std::abs(
        static_cast<int64_t>(pending_left_->timestamp_ns - pending_right_->timestamp_ns));
    double diff_ms = diff_ns / 1000000.0;

    if (diff_ms <= sync_threshold_ms_) {
      // Pair is synchronized: use left (master) timestamp as the reference
      left = pending_left_->image;
      right = pending_right_->image;
      timestamp_ns = pending_left_->timestamp_ns;
      pending_left_.reset();
      pending_right_.reset();
      return MATCH_SUCCESS;
    } else {
      // Discard the older frame and wait for the next one from that camera
      if (pending_left_->timestamp_ns < pending_right_->timestamp_ns) {
        pending_left_.reset();
      } else {
        pending_right_.reset();
      }
      return OUT_OF_SYNC_DISCARD;
    }
  }

  double sync_threshold_ms_;
  MatchCallback match_callback_;
  DropCallback drop_callback_;
  std::mutex mutex_;
  std::unique_ptr<PendingFrame> pending_left_;
  std::unique_ptr<PendingFrame> pending_right_;
};

// Struct holding a synchronized stereo pair queued for publishing
struct StereoPair {
  cv::Mat left;
  cv::Mat right;
  uint64_t timestamp_ns;
};

// Main ROS2 Node: manages GStreamer pipelines, publishers, and the publish worker thread.
class GStreamerIMX219Camera : public BaseCameraNode {
public:
  explicit GStreamerIMX219Camera(const rclcpp::NodeOptions & options)
  : BaseCameraNode("gstreamer_imx219_camera", options)
  {
    // Initialize GStreamer runtime once before any pipeline is created
    gst_init(nullptr, nullptr);
    RCLCPP_INFO(this->get_logger(), "GStreamer version: %s", gst_version_string());

    // Declare and retrieve runtime parameters
    cfg_.left_camera_name   = this->declare_parameter<std::string>("left_camera_name",
        "/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10");
    cfg_.right_camera_name  = this->declare_parameter<std::string>("right_camera_name",
        "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10");
    cfg_.width              = this->declare_parameter<int>("width", 640);
    cfg_.height             = this->declare_parameter<int>("height", 480);
    cfg_.fps                = this->declare_parameter<double>("fps", 30.0);
    cfg_.rotation           = this->declare_parameter<int>("rotation", 180);
    cfg_.publish_raw        = this->declare_parameter<bool>("publish_raw", false);
    cfg_.publish_compressed = this->declare_parameter<bool>("publish_compressed", true);
    cfg_.enable_benchmark   = this->declare_parameter<bool>("enable_benchmark", false);
    cfg_.benchmark_csv_path = this->declare_parameter<std::string>("benchmark_csv_path", "");
    cfg_.enable_hw_sync     = this->declare_parameter<bool>("enable_hw_sync", true);

    // sync_threshold is half the frame period (same derivation as libcamera node)
    const double sync_threshold_ms = 0.5 * (1000.0 / cfg_.fps);

    if (cfg_.enable_benchmark) {
      benchmark_ = std::make_unique<CameraBenchmark>();
      if (!cfg_.benchmark_csv_path.empty()) {
        benchmark_->openCSV(cfg_.benchmark_csv_path);
        RCLCPP_INFO(this->get_logger(),
          "Logging benchmark metrics to CSV: %s", cfg_.benchmark_csv_path.c_str());
      }
    }
    // Populate base class parameters
    common_params_.width  = cfg_.width;
    common_params_.height = cfg_.height;
    common_params_.fps    = cfg_.fps;

    RCLCPP_INFO(this->get_logger(), "Initializing GStreamer IMX219 Stereo Camera Component");
    RCLCPP_INFO(this->get_logger(),
      "Params: LeftName=%s, RightName=%s, Size=%dx%d, FPS=%.1f, "
      "SyncThreshold=%.1fms, Rotation=%d, HwSync=%d, "
      "PublishRaw=%d, PublishCompressed=%d, Benchmark=%d",
      cfg_.left_camera_name.c_str(), cfg_.right_camera_name.c_str(),
      cfg_.width, cfg_.height, cfg_.fps,
      sync_threshold_ms, cfg_.rotation, cfg_.enable_hw_sync,
      cfg_.publish_raw, cfg_.publish_compressed, cfg_.enable_benchmark);

    // Initialize publishers
    if (cfg_.publish_raw) {
      left_image_pub_  = image_transport::create_publisher(
          this, "/cameras/stereo_left/image_raw");
      right_image_pub_ = image_transport::create_publisher(
          this, "/cameras/stereo_right/image_raw");
    }

    if (cfg_.publish_compressed) {
      left_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/cameras/stereo_left/image_compressed", 10);
      right_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/cameras/stereo_right/image_compressed", 10);
    }

    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/cameras/stereo_left/camera_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/cameras/stereo_right/camera_info", 10);

    // Initialize Camera Info Managers
    left_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, "stereo_left", imx219_gst_hardware::LEFT_INFO_URL);
    right_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, "stereo_right", imx219_gst_hardware::RIGHT_INFO_URL);

    // Initialize FrameSynchronizer: on match, push to publish queue
    synchronizer_ = std::make_unique<FrameSynchronizer>(
      sync_threshold_ms,
      [this](const cv::Mat &left, const cv::Mat &right, uint64_t ts) {
        {
          std::lock_guard<std::mutex> lock(this->queue_mutex_);
          if (this->publish_queue_.size() < 5) {
            this->publish_queue_.push(StereoPair{left, right, ts});
            this->queue_cv_.notify_one();
          } else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Publish queue overflow! Dropping frame.");
            if (this->benchmark_) {
              this->benchmark_->recordQueueDrop();
            }
          }
        }
      },
      [this]() {
        if (this->benchmark_) {
          this->benchmark_->recordSyncDrop();
        }
      });

    // Start background publishing worker thread
    worker_running_ = true;
    worker_thread_ = std::thread(&GStreamerIMX219Camera::workerLoop, this);

    // Initialize and start GStreamer pipelines
    if (!startCameras()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start camera pipelines");
      throw std::runtime_error(
          "GStreamerIMX219Camera: failed to initialize camera hardware");
    }
  }

  ~GStreamerIMX219Camera() override {
    stopCameras();

    // Signal worker thread to exit and join
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      worker_running_ = false;
      queue_cv_.notify_all();
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    if (benchmark_) {
      benchmark_->closeCSV();
    }
  }

private:
  // Create and start GstSingleCamera instances for left and right cameras
  bool startCameras() {
    RCLCPP_INFO(this->get_logger(), "Using Left Camera ID: %s", cfg_.left_camera_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Right Camera ID: %s", cfg_.right_camera_name.c_str());

    left_camera_ = std::make_unique<GstSingleCamera>(
      this,
      cfg_.left_camera_name,
      cfg_.width, cfg_.height, cfg_.fps, cfg_.rotation,
      imx219_gst_hardware::SYNC_MODE_SERVER,
      cfg_.enable_hw_sync,
      [this](const cv::Mat &img, uint64_t ts) {
        synchronizer_->addLeftFrame(img, ts);
      });

    right_camera_ = std::make_unique<GstSingleCamera>(
      this,
      cfg_.right_camera_name,
      cfg_.width, cfg_.height, cfg_.fps, cfg_.rotation,
      imx219_gst_hardware::SYNC_MODE_CLIENT,
      cfg_.enable_hw_sync,
      [this](const cv::Mat &img, uint64_t ts) {
        synchronizer_->addRightFrame(img, ts);
      });

    if (!left_camera_->initialize() || !right_camera_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize one or both camera pipelines");
      return false;
    }

    if (!left_camera_->start() || !right_camera_->start()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start one or both camera pipelines");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Both camera pipelines running");
    return true;
  }

  // Stop and destroy GStreamer pipelines
  void stopCameras() {
    if (left_camera_) {
      left_camera_->stop();
      left_camera_.reset();
    }
    if (right_camera_) {
      right_camera_->stop();
      right_camera_.reset();
    }
    if (synchronizer_) {
      synchronizer_->clear();
    }
  }

  // Background thread: dequeues synchronized pairs and calls publishStereo()
  void workerLoop() {
    while (worker_running_) {
      StereoPair pair;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock,
          [this]() { return !publish_queue_.empty() || !worker_running_; });
        if (!worker_running_ && publish_queue_.empty()) {
          break;
        }
        pair = std::move(publish_queue_.front());
        publish_queue_.pop();
      }

      publishStereo(pair.left, pair.right);

      if (benchmark_) {
        benchmark_->recordPublish(pair.timestamp_ns);
        if (benchmark_->shouldReport(5.0)) {
          benchmark_->report(this->get_logger(), cfg_.fps);
        }
      }
    }
  }

  // Publish CameraInfo (calibrated or empty)
  void publishCameraInfo(
    const cv::Mat & frame,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & info_pub,
    std::unique_ptr<camera_info_manager::CameraInfoManager> & info_manager,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
  {
    if (info_manager && info_manager->isCalibrated()) {
      auto info_msg = info_manager->getCameraInfo();
      info_msg.header.stamp = stamp;
      info_msg.header.frame_id = frame_id;
      info_pub->publish(info_msg);
    } else {
      sensor_msgs::msg::CameraInfo info_msg;
      info_msg.header.stamp = stamp;
      info_msg.header.frame_id = frame_id;
      info_msg.width = frame.cols;
      info_msg.height = frame.rows;
      info_pub->publish(info_msg);
    }
  }

  // Process and publish a synchronized stereo pair
  void publishStereo(const cv::Mat &left_raw, const cv::Mat &right_raw) {
    cv::Mat left_processed = left_raw;
    cv::Mat right_processed = right_raw;

    // Software rotation for 90/270 degrees only.
    // 180-degree flip is handled at the hardware level via libcamerasrc
    // (libcamera orientation control) — no software rotation needed.
    if (cfg_.rotation == 90) {
      cv::rotate(left_raw,  left_processed,  cv::ROTATE_90_CLOCKWISE);
      cv::rotate(right_raw, right_processed, cv::ROTATE_90_CLOCKWISE);
    } else if (cfg_.rotation == 270) {
      cv::rotate(left_raw,  left_processed,  cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::rotate(right_raw, right_processed, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    // Resize only if the captured size doesn't match the target output size
    cv::Mat left_resized  = left_processed;
    cv::Mat right_resized = right_processed;
    if (left_processed.cols != cfg_.width || left_processed.rows != cfg_.height) {
      cv::resize(left_processed,  left_resized,  cv::Size(cfg_.width, cfg_.height));
    }
    if (right_processed.cols != cfg_.width || right_processed.rows != cfg_.height) {
      cv::resize(right_processed, right_resized, cv::Size(cfg_.width, cfg_.height));
    }

    // Debug: save one JPEG per second to workspace for visual monitoring
    static auto last_save_time = std::chrono::steady_clock::now();
    auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(
            now_time - last_save_time).count() >= 1) {
      cv::imwrite("/home/duykhongcay/ros2_ws/gst_debug_left.jpg",  left_resized);
      cv::imwrite("/home/duykhongcay/ros2_ws/gst_debug_right.jpg", right_resized);
      last_save_time = now_time;
    }

    auto stamp = this->now();

    if (cfg_.publish_raw) {
      publishImageAndInfo(left_resized,  left_image_pub_,  left_info_pub_,
          left_info_manager_,  imx219_gst_hardware::LEFT_FRAME_ID,  stamp);
      publishImageAndInfo(right_resized, right_image_pub_, right_info_pub_,
          right_info_manager_, imx219_gst_hardware::RIGHT_FRAME_ID, stamp);
    } else {
      // Still publish CameraInfo even when raw image publishing is disabled
      publishCameraInfo(left_resized,  left_info_pub_,  left_info_manager_,
          imx219_gst_hardware::LEFT_FRAME_ID,  stamp);
      publishCameraInfo(right_resized, right_info_pub_, right_info_manager_,
          imx219_gst_hardware::RIGHT_FRAME_ID, stamp);
    }

    if (cfg_.publish_compressed) {
      std::vector<uchar> left_buf, right_buf;
      const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 80};

      cv::imencode(".jpg", left_resized,  left_buf,  encode_params);
      cv::imencode(".jpg", right_resized, right_buf, encode_params);

      auto left_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      left_msg->header.stamp    = stamp;
      left_msg->header.frame_id = imx219_gst_hardware::LEFT_FRAME_ID;
      left_msg->format          = "jpeg";
      left_msg->data            = std::move(left_buf);

      auto right_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      right_msg->header.stamp    = stamp;
      right_msg->header.frame_id = imx219_gst_hardware::RIGHT_FRAME_ID;
      right_msg->format          = "jpeg";
      right_msg->data            = std::move(right_buf);

      left_compressed_pub_->publish(*left_msg);
      right_compressed_pub_->publish(*right_msg);
    }
  }

  // Camera configuration parameter structure
  CameraConfig cfg_;

  // Performance benchmark helper
  std::unique_ptr<CameraBenchmark> benchmark_;

  // GStreamer camera pipeline wrappers
  std::unique_ptr<GstSingleCamera> left_camera_;
  std::unique_ptr<GstSingleCamera> right_camera_;

  // Frame synchronization manager
  std::unique_ptr<FrameSynchronizer> synchronizer_;

  // ROS2 Publishers
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  // Camera calibration configuration managers
  std::unique_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;

  // Background publishing worker thread and queue
  std::queue<StereoPair> publish_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread worker_thread_;
  bool worker_running_;
};

}  // namespace lekiwi_cameras

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_cameras::GStreamerIMX219Camera)



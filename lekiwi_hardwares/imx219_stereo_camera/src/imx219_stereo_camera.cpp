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
#include <sys/mman.h>
#include <unistd.h>
#include <cstdlib>
#include <numeric>
#include <stdexcept>
#include <fstream>
#include <iomanip>

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

// Libcamera Headers
#include <libcamera/libcamera.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/formats.h>

// Base Class
#include "imx219_stereo_camera/base_camera_node.hpp"

using namespace std::chrono_literals;
using namespace libcamera;

namespace lekiwi_cameras {

// Hardware specific constants for IMX219 camera
namespace imx219_hardware {
  constexpr int  SENSOR_WIDTH    = 1640;
  constexpr int  SENSOR_HEIGHT   = 1232;
  constexpr int  BIT_DEPTH       = 10;
  // TF frame IDs are fixed to the robot's mechanical design
  constexpr char LEFT_FRAME_ID[]  = "stereo_left_optical";
  constexpr char RIGHT_FRAME_ID[] = "stereo_right_optical";
  // Empty URL = no calibration file; node will publish an empty CameraInfo
  constexpr char LEFT_INFO_URL[]  = "";
  constexpr char RIGHT_INFO_URL[] = "";
}

// Configurable runtime settings of the stereo camera node
struct CameraConfig {
  int         server_idx;
  int         client_idx;
  int         width;
  int         height;
  double      fps;
  int         rotation;
  bool        publish_raw;
  bool        publish_compressed;
  bool        enable_benchmark;
  std::string benchmark_csv_path;
};

// Benchmark helper class to track latency, FPS, drop rate and jitter
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
      csv_file_ << "frame_index,hw_timestamp_ns,latency_ms,jitter_ms\n";
    }
  }

  void closeCSV() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

  void recordPublish(uint64_t hw_timestamp_ns) {
    auto now_steady = std::chrono::steady_clock::now();
    
    // Read CLOCK_BOOTTIME clock which libcamera timestamps use
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    uint64_t now_boot_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;

    double latency_ms = 0.0;
    if (now_boot_ns >= hw_timestamp_ns) {
      latency_ms = static_cast<double>(now_boot_ns - hw_timestamp_ns) / 1000000.0;
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
      csv_file_ << published_count_ << "," << hw_timestamp_ns << "," 
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
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    return elapsed >= report_interval_sec;
  }

  void report(rclcpp::Logger logger, double target_fps) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed <= 0.0) return;

    double actual_fps = published_count_ / elapsed;
    uint64_t total_dropped = dropped_queue_count_ + dropped_sync_count_;
    uint64_t total_frames = published_count_ + total_dropped;
    double drop_rate = total_frames > 0 ? (static_cast<double>(total_dropped) / total_frames) * 100.0 : 0.0;
    double sync_drop_rate = total_frames > 0 ? (static_cast<double>(dropped_sync_count_) / total_frames) * 100.0 : 0.0;
    double queue_drop_rate = total_frames > 0 ? (static_cast<double>(dropped_queue_count_) / total_frames) * 100.0 : 0.0;

    double avg_latency = 0.0;
    double max_latency = 0.0;
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

// Struct to store mapped CPU virtual memory address for each FrameBuffer plane
struct MappedBuffer {
  void *mem;
  size_t size;
};

// Struct to hold frames temporarily for software timestamp synchronization
struct PendingFrame {
  cv::Mat image;
  uint64_t timestamp_ns;
};

// IMX219Single handles low-level camera management (Libcamera API, memory mapping, request queuing)
class IMX219Single {
public:
  using FrameCallback = std::function<void(const cv::Mat &image, uint64_t timestamp_ns)>;

  IMX219Single(rclcpp::Node *node, std::shared_ptr<Camera> camera, int width, int height, double fps, int rotation, FrameCallback callback)
      : node_(node), camera_(camera), width_(width), height_(height), fps_(fps), rotation_(rotation), callback_(callback), is_initialized_(false), is_started_(false) {}

  ~IMX219Single() {
    stop();
  }

  // Initialize camera pipeline resources
  bool initialize() {
    if (is_initialized_) return true;

    if (camera_->acquire() < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to acquire camera");
      return false;
    }

    std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({StreamRole::VideoRecording});
    if (!config) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to generate camera configuration");
      release();
      return false;
    }

    // Set configuration resolution and pixel format
    const Size hardware_size(width_, height_);
    StreamConfiguration &stream_cfg = config->at(0);
    stream_cfg.size = hardware_size;
    stream_cfg.pixelFormat = formats::BGR888;
    stream_cfg.bufferCount = 4;

    // Apply hardware orientation (180 degrees flip) at sensor/ISP level if configured
    if (rotation_ == 180) {
      config->orientation = Orientation::Rotate180;
    }

    // Configure sensor output resolution to Full FOV using constants
    SensorConfiguration sensor_cfg;
    sensor_cfg.outputSize = Size(imx219_hardware::SENSOR_WIDTH, imx219_hardware::SENSOR_HEIGHT);
    sensor_cfg.bitDepth = imx219_hardware::BIT_DEPTH;
    config->sensorConfig = sensor_cfg;
    RCLCPP_INFO(node_->get_logger(), "Requesting hardware sensor mode: %dx%d @ %d-bit", 
                imx219_hardware::SENSOR_WIDTH, imx219_hardware::SENSOR_HEIGHT, imx219_hardware::BIT_DEPTH);

    CameraConfiguration::Status status = config->validate();
    if (status == CameraConfiguration::Invalid) {
      RCLCPP_ERROR(node_->get_logger(), "Camera configuration is invalid. Sensor mode may not be supported.");
      release();
      return false;
    } else if (status == CameraConfiguration::Adjusted) {
      RCLCPP_WARN(node_->get_logger(), "Camera configuration was adjusted by libcamera.");
      if (config->sensorConfig) {
        RCLCPP_WARN(node_->get_logger(), "Adjusted sensor mode to: %dx%d @ %d-bit",
                    config->sensorConfig->outputSize.width,
                    config->sensorConfig->outputSize.height,
                    config->sensorConfig->bitDepth);
      }
    }

    if (camera_->configure(config.get()) < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to configure camera");
      release();
      return false;
    }

    stream_cfg_ = config->at(0);

    // Allocate frame buffers
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_cfg_.stream()) < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to allocate framebuffers");
      release();
      return false;
    }

    // Map buffers to CPU memory space
    if (!mapBuffers(stream_cfg_.stream())) {
      release();
      return false;
    }

    // Create and prepare request packets
    Stream *stream = stream_cfg_.stream();
    const auto &buffers = allocator_->buffers(stream);
    for (size_t i = 0; i < buffers.size(); ++i) {
      std::unique_ptr<Request> request = camera_->createRequest();
      if (!request) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create request");
        release();
        return false;
      }
      request->addBuffer(stream, buffers[i].get());
      requests_.push_back(std::move(request));
    }

    // Connect request complete signal
    camera_->requestCompleted.connect(this, &IMX219Single::onRequestComplete);
    is_initialized_ = true;
    return true;
  }

  // Start the camera and apply synchronization mode control parameters
  bool start(int sync_mode) {
    if (!is_initialized_) {
      RCLCPP_ERROR(node_->get_logger(), "Camera not initialized before starting");
      return false;
    }
    if (is_started_) return true;

    ControlList ctrls;
    // Set Frame Duration Limits (FPS)
    int64_t frame_time_us = static_cast<int64_t>(1000000.0 / fps_);
    int64_t limits[2] = {frame_time_us, frame_time_us};
    ctrls.set(controls::FrameDurationLimits, Span<const int64_t, 2>(limits, 2));

    // Set hardware synchronization mode (Raspberry Pi specific controls)
    ctrls.set(controls::rpi::SyncMode, sync_mode);

    if (camera_->start(&ctrls) < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to start camera");
      return false;
    }

    is_started_ = true;
    return true;
  }

  // Queue initial requests to camera pipeline
  void queueRequests() {
    if (!is_started_) return;
    for (auto &req : requests_) {
      camera_->queueRequest(req.get());
    }
  }

  // Safely stop and release camera resources
  void stop() {
    if (is_started_) {
      camera_->stop();
      is_started_ = false;
    }
    if (is_initialized_) {
      camera_->requestCompleted.disconnect(this, &IMX219Single::onRequestComplete);
      requests_.clear();
      if (allocator_) {
        allocator_->free(stream_cfg_.stream());
        allocator_.reset();
      }
      unmapBuffers();
      camera_->release();
      is_initialized_ = false;
    }
  }

private:
  bool mapBuffers(Stream *stream) {
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);
    for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
      std::vector<MappedBuffer> planes;
      for (const FrameBuffer::Plane &plane : buffer->planes()) {
        int fd = plane.fd.get();
        void *mem = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, fd, plane.offset);
        if (mem == MAP_FAILED) {
          RCLCPP_ERROR(node_->get_logger(), "mmap plane failed: %s", strerror(errno));
          return false;
        }
        planes.push_back({mem, plane.length});
      }
      mapped_buffers_[buffer.get()] = planes;
    }
    return true;
  }

  void unmapBuffers() {
    for (auto const &pair : mapped_buffers_) {
      for (auto const &plane : pair.second) {
        munmap(plane.mem, plane.size);
      }
    }
    mapped_buffers_.clear();
  }

  void release() {
    unmapBuffers();
    if (camera_) {
      camera_->release();
    }
    is_initialized_ = false;
    is_started_ = false;
  }

  void onRequestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled) {
      return;
    }

    Stream *stream = stream_cfg_.stream();
    FrameBuffer *buffer = request->buffers().at(stream);
    uint64_t ts_ns = buffer->metadata().timestamp;

    auto const &planes = mapped_buffers_[buffer];
    if (!planes.empty()) {
      // Create cv::Mat referencing the mapped buffer directly (no copy yet), passing stride to handle padding
      cv::Mat raw_img(stream_cfg_.size.height, stream_cfg_.size.width, CV_8UC3, planes[0].mem, stream_cfg_.stride);
      
      // Clone the image immediately so we can release the camera hardware buffer as fast as possible
      cv::Mat cloned_img = raw_img.clone();

      // Queue request back to camera pipeline immediately
      request->reuse(Request::ReuseBuffers);
      camera_->queueRequest(request);

      // Pass the cloned frame to callback for further processing (resize, rotate, publish)
      callback_(cloned_img, ts_ns);
    } else {
      // Queue request back to camera pipeline if buffer mapping was empty
      request->reuse(Request::ReuseBuffers);
      camera_->queueRequest(request);
    }
  }

  rclcpp::Node *node_;
  std::shared_ptr<Camera> camera_;
  int width_;
  int height_;
  double fps_;
  int rotation_;
  FrameCallback callback_;
  bool is_initialized_;
  bool is_started_;

  StreamConfiguration stream_cfg_;
  std::unique_ptr<FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<Request>> requests_;
  std::map<FrameBuffer *, std::vector<MappedBuffer>> mapped_buffers_;
};

// FrameSynchronizer matches frames from Server and Client cameras based on timestamps
class FrameSynchronizer {
public:
  enum MatchResult {
    MATCH_SUCCESS,
    AWAITING_FRAME,
    OUT_OF_SYNC_DISCARD
  };

  using MatchCallback = std::function<void(const cv::Mat &left_img, const cv::Mat &right_img, uint64_t timestamp_ns)>;
  using DropCallback = std::function<void()>;

  FrameSynchronizer(double sync_threshold_ms, MatchCallback match_callback, DropCallback drop_callback = nullptr)
      : sync_threshold_ms_(sync_threshold_ms), match_callback_(match_callback), drop_callback_(drop_callback) {}

  // Receives Server frame, performs memory clone outside of the mutex lock to avoid performance bottleneck
  void addServerFrame(const cv::Mat &image, uint64_t timestamp_ns) {
    auto new_frame = std::make_unique<PendingFrame>(PendingFrame{image, timestamp_ns});

    cv::Mat left, right;
    uint64_t matched_ts = 0;
    MatchResult res;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_server_ = std::move(new_frame);
      res = tryMatch(left, right, matched_ts);
    }

    if (res == MATCH_SUCCESS) {
      match_callback_(left, right, matched_ts);
    } else if (res == OUT_OF_SYNC_DISCARD) {
      if (drop_callback_) drop_callback_();
    }
  }

  // Receives Client frame, performs memory clone outside of the mutex lock to avoid performance bottleneck
  void addClientFrame(const cv::Mat &image, uint64_t timestamp_ns) {
    auto new_frame = std::make_unique<PendingFrame>(PendingFrame{image, timestamp_ns});

    cv::Mat left, right;
    uint64_t matched_ts = 0;
    MatchResult res;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_client_ = std::move(new_frame);
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
    pending_server_.reset();
    pending_client_.reset();
  }

private:
  // Must be called with mutex_ held
  MatchResult tryMatch(cv::Mat &left, cv::Mat &right, uint64_t &timestamp_ns) {
    if (!pending_server_ || !pending_client_) {
      return AWAITING_FRAME;
    }

    int64_t diff_ns = std::abs(static_cast<int64_t>(pending_server_->timestamp_ns - pending_client_->timestamp_ns));
    double diff_ms = diff_ns / 1000000.0;

    if (diff_ms <= sync_threshold_ms_) {
      // Synchronized pair matched successfully
      left = pending_server_->image;
      right = pending_client_->image;
      timestamp_ns = pending_server_->timestamp_ns; // Use server (master) timestamp as reference
      pending_server_.reset();
      pending_client_.reset();
      return MATCH_SUCCESS;
    } else {
      // Out of sync, discard the older frame
      if (pending_server_->timestamp_ns < pending_client_->timestamp_ns) {
        pending_server_.reset();
      } else {
        pending_client_.reset();
      }
      return OUT_OF_SYNC_DISCARD;
    }
  }

  double sync_threshold_ms_;
  MatchCallback match_callback_;
  DropCallback drop_callback_;
  std::mutex mutex_;
  std::unique_ptr<PendingFrame> pending_server_;
  std::unique_ptr<PendingFrame> pending_client_;
};

// Struct to hold a pair of synchronized frames with hardware timestamp for the publishing queue
struct StereoPair {
  cv::Mat left;
  cv::Mat right;
  uint64_t timestamp_ns;
};

// Main ROS2 Node Component managing configuration parameters, publishers and coordinating device pipelines
class IMX219StereoCamera : public BaseCameraNode {
public:
  // Set libcamera environment variables before any camera object is created
  static void configureLibcameraEnv() {
    setenv("LIBCAMERA_RPI_TUNING_FILE", "/usr/share/libcamera/ipa/rpi/pisp/imx219.json", 1);
    setenv("LIBCAMERA_IPA_CONFIG_PATH", "/usr/share/libcamera/ipa", 1);
  }

  explicit IMX219StereoCamera(const rclcpp::NodeOptions & options) 
  : BaseCameraNode("imx219_stereo_camera", options) {
    configureLibcameraEnv();

    // Declare and retrieve runtime parameters on single lines directly populating the config struct
    cfg_.server_idx         = this->declare_parameter<int>("server_idx", 0);
    cfg_.client_idx         = this->declare_parameter<int>("client_idx", 1);
    cfg_.width              = this->declare_parameter<int>("width", 640);
    cfg_.height             = this->declare_parameter<int>("height", 480);
    cfg_.fps                = this->declare_parameter<double>("fps", 30.0);
    cfg_.rotation           = this->declare_parameter<int>("rotation", 180);
    cfg_.publish_raw        = this->declare_parameter<bool>("publish_raw", false);
    cfg_.publish_compressed = this->declare_parameter<bool>("publish_compressed", true);
    cfg_.enable_benchmark   = this->declare_parameter<bool>("enable_benchmark", false);
    cfg_.benchmark_csv_path = this->declare_parameter<std::string>("benchmark_csv_path", "");

    // sync_threshold is derived from fps: half the frame period
    const double sync_threshold_ms = 0.5 * (1000.0 / cfg_.fps);

    if (cfg_.enable_benchmark) {
      benchmark_ = std::make_unique<CameraBenchmark>();
      if (!cfg_.benchmark_csv_path.empty()) {
        benchmark_->openCSV(cfg_.benchmark_csv_path);
        RCLCPP_INFO(this->get_logger(), "Logging benchmark metrics to CSV: %s", cfg_.benchmark_csv_path.c_str());
      }
    }

    // Populate base class parameters so other utilities can access them
    common_params_.width  = cfg_.width;
    common_params_.height = cfg_.height;
    common_params_.fps    = cfg_.fps;

    RCLCPP_INFO(this->get_logger(), "Initializing IMX219 Stereo Camera Component");
    RCLCPP_INFO(this->get_logger(), "Params: ServerIdx=%d, ClientIdx=%d, PublishSize=%dx%d, FPS=%.1f, SyncThreshold=%.1fms, Rotation=%d, PublishRaw=%d, PublishCompressed=%d, Benchmark=%d",
                cfg_.server_idx, cfg_.client_idx, cfg_.width, cfg_.height, cfg_.fps, sync_threshold_ms, cfg_.rotation, cfg_.publish_raw, cfg_.publish_compressed, cfg_.enable_benchmark);

    // Initialize publishers
    if (cfg_.publish_raw) {
      left_image_pub_ = image_transport::create_publisher(this, "/cameras/stereo_left/image_raw");
      right_image_pub_ = image_transport::create_publisher(this, "/cameras/stereo_right/image_raw");
    }
    if (cfg_.publish_compressed) {
      left_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/cameras/stereo_left/image_compressed", 10);
      right_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/cameras/stereo_right/image_compressed", 10);
    }

    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cameras/stereo_left/camera_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cameras/stereo_right/camera_info", 10);

    // Initialize Camera Info Managers
    left_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "stereo_left", imx219_hardware::LEFT_INFO_URL);
    right_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "stereo_right", imx219_hardware::RIGHT_INFO_URL);

    // Initialize FrameSynchronizer to queue synchronized frames
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
    worker_thread_ = std::thread(&IMX219StereoCamera::workerLoop, this);

    // Initialize and start cameras
    if (!startCameras()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start camera hardware");
      throw std::runtime_error("Failed to start camera hardware: not enough cameras or initialization failed");
    }
  }

  ~IMX219StereoCamera() override {
    stopCameras();

    // Signal and join the background worker thread
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
  bool startCameras() {
    // Create Camera Manager
    camera_manager_ = std::make_unique<CameraManager>();
    int ret = camera_manager_->start();
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start CameraManager: %s", strerror(-ret));
      return false;
    }

    std::vector<std::shared_ptr<Camera>> cameras = camera_manager_->cameras();
    if (cameras.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "Not enough cameras detected (found %zu, need at least 2)", cameras.size());
      camera_manager_->stop();
      return false;
    }

    std::shared_ptr<Camera> server_cam = nullptr;
    std::shared_ptr<Camera> client_cam = nullptr;

    // Distribute cameras based on client/server index parameters
    for (auto const &cam : cameras) {
      if (cfg_.server_idx >= 0 && cfg_.server_idx < static_cast<int>(cameras.size()) && cam == cameras[cfg_.server_idx]) {
        server_cam = cam;
      }
      if (cfg_.client_idx >= 0 && cfg_.client_idx < static_cast<int>(cameras.size()) && cam == cameras[cfg_.client_idx]) {
        client_cam = cam;
      }
    }

    if (!server_cam || !client_cam) {
      // Fallback allocation if parameters don't match
      server_cam = cameras[0];
      client_cam = cameras[1];
    }

    RCLCPP_INFO(this->get_logger(), "Server Camera ID: %s", server_cam->id().c_str());
    RCLCPP_INFO(this->get_logger(), "Client Camera ID: %s", client_cam->id().c_str());

    // Construct device wrappers, passing config values directly
    server_device_ = std::make_unique<IMX219Single>(
        this, server_cam, cfg_.width, cfg_.height, cfg_.fps, cfg_.rotation,
        [this](const cv::Mat &img, uint64_t ts) { synchronizer_->addServerFrame(img, ts); });

    client_device_ = std::make_unique<IMX219Single>(
        this, client_cam, cfg_.width, cfg_.height, cfg_.fps, cfg_.rotation,
        [this](const cv::Mat &img, uint64_t ts) { synchronizer_->addClientFrame(img, ts); });

    if (!server_device_->initialize() || !client_device_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera devices");
      return false;
    }

    // Start server in master mode (SyncModeServer) and client in slave mode (SyncModeClient)
    if (!server_device_->start(controls::rpi::SyncModeServer) ||
        !client_device_->start(controls::rpi::SyncModeClient)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start camera devices");
      return false;
    }

    // Queue requests to activate pipeline capture
    server_device_->queueRequests();
    client_device_->queueRequests();

    RCLCPP_INFO(this->get_logger(), "Cameras started and pipeline running");
    return true;
  }

  void stopCameras() {
    if (server_device_) {
      server_device_->stop();
      server_device_.reset();
    }
    if (client_device_) {
      client_device_->stop();
      client_device_.reset();
    }
    if (camera_manager_) {
      camera_manager_->stop();
      camera_manager_.reset();
    }
    if (synchronizer_) {
      synchronizer_->clear();
    }
  }

  void workerLoop() {
    while (worker_running_) {
      StereoPair pair;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return !publish_queue_.empty() || !worker_running_; });
        if (!worker_running_ && publish_queue_.empty()) {
          break;
        }
        pair = std::move(publish_queue_.front());
        publish_queue_.pop();
      }
      publishStereo(pair.left, pair.right);

      // Record benchmark metrics if enabled
      if (benchmark_) {
        benchmark_->recordPublish(pair.timestamp_ns);
        if (benchmark_->shouldReport(5.0)) {
          benchmark_->report(this->get_logger(), cfg_.fps);
        }
      }
    }
  }

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

  void publishStereo(const cv::Mat &left_raw, const cv::Mat &right_raw) {
    cv::Mat left_processed = left_raw;
    cv::Mat right_processed = right_raw;

    // Software rotation is only needed for 90 and 270 degrees.
    if (cfg_.rotation == 90) {
      cv::rotate(left_raw, left_processed, cv::ROTATE_90_CLOCKWISE);
      cv::rotate(right_raw, right_processed, cv::ROTATE_90_CLOCKWISE);
    } else if (cfg_.rotation == 270) {
      cv::rotate(left_raw, left_processed, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::rotate(right_raw, right_processed, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    // Hardware ISP handles resize directly.
    // Software resize is only a fallback in case raw size doesn't match target.
    cv::Mat left_resized = left_processed;
    cv::Mat right_resized = right_processed;
    if (left_processed.cols != cfg_.width || left_processed.rows != cfg_.height) {
      cv::resize(left_processed, left_resized, cv::Size(cfg_.width, cfg_.height));
    }
    if (right_processed.cols != cfg_.width || right_processed.rows != cfg_.height) {
      cv::resize(right_processed, right_resized, cv::Size(cfg_.width, cfg_.height));
    }

    // Debug: Save one image per second to workspace for monitoring
    static auto last_save_time = std::chrono::steady_clock::now();
    auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now_time - last_save_time).count() >= 1) {
      cv::imwrite("/home/duykhongcay/ros2_ws/debug_left.jpg", left_resized);
      cv::imwrite("/home/duykhongcay/ros2_ws/debug_right.jpg", right_resized);
      last_save_time = now_time;
    }

    // Prepare stamp and publish both images and info
    auto stamp = this->now();

    if (cfg_.publish_raw) {
      publishImageAndInfo(left_resized, left_image_pub_, left_info_pub_, left_info_manager_, imx219_hardware::LEFT_FRAME_ID, stamp);
      publishImageAndInfo(right_resized, right_image_pub_, right_info_pub_, right_info_manager_, imx219_hardware::RIGHT_FRAME_ID, stamp);
    } else {
      // Still publish CameraInfo when raw image publishing is disabled
      publishCameraInfo(left_resized, left_info_pub_, left_info_manager_, imx219_hardware::LEFT_FRAME_ID, stamp);
      publishCameraInfo(right_resized, right_info_pub_, right_info_manager_, imx219_hardware::RIGHT_FRAME_ID, stamp);
    }

    if (cfg_.publish_compressed) {
      std::vector<uchar> left_buf, right_buf;
      std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 80};
      
      cv::imencode(".jpg", left_resized, left_buf, encode_params);
      cv::imencode(".jpg", right_resized, right_buf, encode_params);

      auto left_compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      left_compressed_msg->header.stamp = stamp;
      left_compressed_msg->header.frame_id = imx219_hardware::LEFT_FRAME_ID;
      left_compressed_msg->format = "jpeg";
      left_compressed_msg->data = std::move(left_buf);

      auto right_compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      right_compressed_msg->header.stamp = stamp;
      right_compressed_msg->header.frame_id = imx219_hardware::RIGHT_FRAME_ID;
      right_compressed_msg->format = "jpeg";
      right_compressed_msg->data = std::move(right_buf);

      left_compressed_pub_->publish(*left_compressed_msg);
      right_compressed_pub_->publish(*right_compressed_msg);
    }
  }

  // Camera configuration parameter structure
  CameraConfig cfg_;

  // Performance benchmark helper
  std::unique_ptr<CameraBenchmark> benchmark_;

  // Libcamera pipeline objects
  std::unique_ptr<CameraManager> camera_manager_;
  std::unique_ptr<IMX219Single> server_device_;
  std::unique_ptr<IMX219Single> client_device_;

  // Synchronization manager
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

  // Background publishing worker thread and queue variables
  std::queue<StereoPair> publish_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread worker_thread_;
  bool worker_running_;
};

} // namespace lekiwi_cameras

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_cameras::IMX219StereoCamera)



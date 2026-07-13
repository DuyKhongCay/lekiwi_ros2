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
#include <map>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <functional>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdlib>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

using namespace std::chrono_literals;
using namespace libcamera;

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

// CameraDevice handles low-level camera management (Libcamera API, memory mapping, request queuing)
class CameraDevice {
public:
  using FrameCallback = std::function<void(const cv::Mat &image, uint64_t timestamp_ns)>;

  CameraDevice(rclcpp::Node *node, std::shared_ptr<Camera> camera, double fps, FrameCallback callback)
      : node_(node), camera_(camera), fps_(fps), callback_(callback), is_initialized_(false), is_started_(false) {}

  ~CameraDevice() {
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

    // Fixed hardware capture resolution to 1640x1232 (native field of view for IMX219)
    const Size hardware_size(1640, 1232);
    StreamConfiguration &stream_cfg = config->at(0);
    stream_cfg.size = hardware_size;
    stream_cfg.pixelFormat = formats::BGR888;
    stream_cfg.bufferCount = 4;

    config->validate();

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
    camera_->requestCompleted.connect(this, &CameraDevice::onRequestComplete);
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
      camera_->requestCompleted.disconnect(this, &CameraDevice::onRequestComplete);
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
      cv::Mat raw_img(1232, 1640, CV_8UC3, planes[0].mem, stream_cfg_.stride);
      
      // Pass the frame to callback, which handles thread safety and memory clone
      callback_(raw_img, ts_ns);
    }

    // Queue request back to camera pipeline
    request->reuse(Request::ReuseBuffers);
    camera_->queueRequest(request);
  }

  rclcpp::Node *node_;
  std::shared_ptr<Camera> camera_;
  double fps_;
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
  using MatchCallback = std::function<void(const cv::Mat &left_img, const cv::Mat &right_img)>;

  FrameSynchronizer(double sync_threshold_ms, MatchCallback match_callback)
      : sync_threshold_ms_(sync_threshold_ms), match_callback_(match_callback) {}

  // Receives Server frame, performs memory clone outside of the mutex lock to avoid performance bottleneck
  void addServerFrame(const cv::Mat &image, uint64_t timestamp_ns) {
    // Clone image data outside the lock to prevent thread blocking
    cv::Mat cloned_image = image.clone();
    auto new_frame = std::make_unique<PendingFrame>(PendingFrame{cloned_image, timestamp_ns});

    cv::Mat left, right;
    bool matched = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_server_ = std::move(new_frame);
      matched = tryMatch(left, right);
    }

    // Execute publish callback outside of the mutex lock to allow other threads to push frames concurrently
    if (matched) {
      match_callback_(left, right);
    }
  }

  // Receives Client frame, performs memory clone outside of the mutex lock to avoid performance bottleneck
  void addClientFrame(const cv::Mat &image, uint64_t timestamp_ns) {
    // Clone image data outside the lock to prevent thread blocking
    cv::Mat cloned_image = image.clone();
    auto new_frame = std::make_unique<PendingFrame>(PendingFrame{cloned_image, timestamp_ns});

    cv::Mat left, right;
    bool matched = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_client_ = std::move(new_frame);
      matched = tryMatch(left, right);
    }

    // Execute publish callback outside of the mutex lock to allow other threads to push frames concurrently
    if (matched) {
      match_callback_(left, right);
    }
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_server_.reset();
    pending_client_.reset();
  }

private:
  // Must be called with mutex_ held
  bool tryMatch(cv::Mat &left, cv::Mat &right) {
    if (!pending_server_ || !pending_client_) {
      return false; // Awaiting frame from the other camera
    }

    int64_t diff_ns = std::abs(static_cast<int64_t>(pending_server_->timestamp_ns - pending_client_->timestamp_ns));
    double diff_ms = diff_ns / 1000000.0;

    if (diff_ms <= sync_threshold_ms_) {
      // Synchronized pair matched successfully
      left = pending_server_->image;
      right = pending_client_->image;
      pending_server_.reset();
      pending_client_.reset();
      return true;
    } else {
      // Out of sync, discard the older frame
      if (pending_server_->timestamp_ns < pending_client_->timestamp_ns) {
        pending_server_.reset();
      } else {
        pending_client_.reset();
      }
      return false;
    }
  }

  double sync_threshold_ms_;
  MatchCallback match_callback_;
  std::mutex mutex_;
  std::unique_ptr<PendingFrame> pending_server_;
  std::unique_ptr<PendingFrame> pending_client_;
};

// Main ROS2 Node managing configuration parameters, publishers and coordinating device pipelines
class IMX219StereoCameraNode : public rclcpp::Node {
public:
  IMX219StereoCameraNode() : Node("imx219_stereo_camera_node") {
    // Declare and retrieve parameters
    this->declare_parameter<int>("server_idx", 0);
    this->declare_parameter<int>("client_idx", 1);
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<double>("sync_threshold_ms", 15.0);
    this->declare_parameter<std::string>("left_camera_info_url", "");
    this->declare_parameter<std::string>("right_camera_info_url", "");
    this->declare_parameter<std::string>("left_camera_frame_id", "stereo_left_optical");
    this->declare_parameter<std::string>("right_camera_frame_id", "stereo_right_optical");
    this->declare_parameter<int>("rotation", 180); // Rotate 180 degrees by default for physical alignment

    this->get_parameter("server_idx", server_idx_);
    this->get_parameter("client_idx", client_idx_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("fps", fps_);
    this->get_parameter("sync_threshold_ms", sync_threshold_ms_);
    this->get_parameter("left_camera_info_url", left_camera_info_url_);
    this->get_parameter("right_camera_info_url", right_camera_info_url_);
    this->get_parameter("left_camera_frame_id", left_camera_frame_id_);
    this->get_parameter("right_camera_frame_id", right_camera_frame_id_);
    this->get_parameter("rotation", rotation_);

    RCLCPP_INFO(this->get_logger(), "Initializing IMX219 Stereo Camera Node (C++)");
    RCLCPP_INFO(this->get_logger(), "Params: ServerIdx=%d, ClientIdx=%d, PublishSize=%dx%d, FPS=%.1f, SyncThreshold=%.1fms, Rotation=%d",
                server_idx_, client_idx_, width_, height_, fps_, sync_threshold_ms_, rotation_);

    // Initialize publishers
    left_image_pub_ = image_transport::create_publisher(this, "/stereo/left/image_raw");
    right_image_pub_ = image_transport::create_publisher(this, "/stereo/right/image_raw");

    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", 10);

    // Initialize Camera Info Managers
    left_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "stereo_left", left_camera_info_url_);
    right_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "stereo_right", right_camera_info_url_);

    // Initialize FrameSynchronizer
    synchronizer_ = std::make_unique<FrameSynchronizer>(
        sync_threshold_ms_,
        [this](const cv::Mat &left, const cv::Mat &right) {
          this->publishStereo(left, right);
        });

    // Initialize and start cameras
    if (!startCameras()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start camera hardware");
      rclcpp::shutdown();
    }
  }

  ~IMX219StereoCameraNode() override {
    stopCameras();
  }

private:
  bool startCameras() {
    // Create Camera Manager
    camera_manager_ = std::make_unique<CameraManager>();
    int ret = camera_manager_->start();
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start camera manager: %d", ret);
      return false;
    }

    std::vector<std::shared_ptr<Camera>> cameras = camera_manager_->cameras();
    if (cameras.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "Found %zu cameras. Need at least 2 cameras for stereo.", cameras.size());
      return false;
    }

    if (server_idx_ >= static_cast<int>(cameras.size()) || client_idx_ >= static_cast<int>(cameras.size())) {
      RCLCPP_ERROR(this->get_logger(), "Invalid server_idx (%d) or client_idx (%d). Max index is %zu",
                  server_idx_, client_idx_, cameras.size() - 1);
      return false;
    }

    std::shared_ptr<Camera> server_camera = cameras.at(server_idx_);
    std::shared_ptr<Camera> client_camera = cameras.at(client_idx_);

    // Create single CameraDevice instances to comply with DRY
    server_device_ = std::make_unique<CameraDevice>(
        this, server_camera, fps_,
        [this](const cv::Mat &image, uint64_t timestamp_ns) {
          synchronizer_->addServerFrame(image, timestamp_ns);
        });

    client_device_ = std::make_unique<CameraDevice>(
        this, client_camera, fps_,
        [this](const cv::Mat &image, uint64_t timestamp_ns) {
          synchronizer_->addClientFrame(image, timestamp_ns);
        });

    // Initialize camera devices
    if (!server_device_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize server camera device");
      return false;
    }
    if (!client_device_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize client camera device");
      server_device_->stop();
      return false;
    }

    // Start cameras (start client first so it waits for server broadcast sync signal)
    if (!client_device_->start(controls::rpi::SyncModeClient)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start client camera device");
      client_device_->stop();
      server_device_->stop();
      return false;
    }
    if (!server_device_->start(controls::rpi::SyncModeServer)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start server camera device");
      client_device_->stop();
      server_device_->stop();
      return false;
    }

    // Queue initial requests
    client_device_->queueRequests();
    server_device_->queueRequests();

    RCLCPP_INFO(this->get_logger(), "Cameras started and pipeline running");
    return true;
  }

  void stopCameras() {
    RCLCPP_INFO(this->get_logger(), "Shutting down cameras...");
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
    RCLCPP_INFO(this->get_logger(), "Cameras shutdown successfully");
  }

  void publishStereo(const cv::Mat &left_raw, const cv::Mat &right_raw) {
    cv::Mat left_processed = left_raw;
    cv::Mat right_processed = right_raw;

    // Apply rotation based on parameters
    if (rotation_ == 180) {
      cv::rotate(left_raw, left_processed, cv::ROTATE_180);
      cv::rotate(right_raw, right_processed, cv::ROTATE_180);
    } else if (rotation_ == 90) {
      cv::rotate(left_raw, left_processed, cv::ROTATE_90_CLOCKWISE);
      cv::rotate(right_raw, right_processed, cv::ROTATE_90_CLOCKWISE);
    } else if (rotation_ == 270) {
      cv::rotate(left_raw, left_processed, cv::ROTATE_90_COUNTERCLOCKWISE);
      cv::rotate(right_raw, right_processed, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    // Resize images to configured publishing/upload dimensions
    cv::Mat left_resized, right_resized;
    cv::resize(left_processed, left_resized, cv::Size(width_, height_));
    cv::resize(right_processed, right_resized, cv::Size(width_, height_));

    // Prepare ROS2 Image Messages
    auto stamp = this->now();

    std_msgs::msg::Header left_hdr;
    left_hdr.stamp = stamp;
    left_hdr.frame_id = left_camera_frame_id_;
    auto left_msg = cv_bridge::CvImage(left_hdr, "bgr8", left_resized).toImageMsg();

    std_msgs::msg::Header right_hdr;
    right_hdr.stamp = stamp;
    right_hdr.frame_id = right_camera_frame_id_;
    auto right_msg = cv_bridge::CvImage(right_hdr, "bgr8", right_resized).toImageMsg();

    // Publish Images
    left_image_pub_.publish(left_msg);
    right_image_pub_.publish(right_msg);

    // Prepare and Publish CameraInfo Messages
    if (left_info_manager_->isCalibrated()) {
      auto left_info = left_info_manager_->getCameraInfo();
      left_info.header.stamp = stamp;
      left_info.header.frame_id = left_camera_frame_id_;
      left_info_pub_->publish(left_info);
    } else {
      sensor_msgs::msg::CameraInfo left_info;
      left_info.header.stamp = stamp;
      left_info.header.frame_id = left_camera_frame_id_;
      left_info.width = width_;
      left_info.height = height_;
      left_info_pub_->publish(left_info);
    }

    if (right_info_manager_->isCalibrated()) {
      auto right_info = right_info_manager_->getCameraInfo();
      right_info.header.stamp = stamp;
      right_info.header.frame_id = right_camera_frame_id_;
      right_info_pub_->publish(right_info);
    } else {
      sensor_msgs::msg::CameraInfo right_info;
      right_info.header.stamp = stamp;
      right_info.header.frame_id = right_camera_frame_id_;
      right_info.width = width_;
      right_info.height = height_;
      right_info_pub_->publish(right_info);
    }
  }

  // Camera settings
  int server_idx_;
  int client_idx_;
  int width_;
  int height_;
  double fps_;
  double sync_threshold_ms_;
  std::string left_camera_info_url_;
  std::string right_camera_info_url_;
  std::string left_camera_frame_id_;
  std::string right_camera_frame_id_;
  int rotation_;

  // Libcamera pipeline objects
  std::unique_ptr<CameraManager> camera_manager_;
  std::unique_ptr<CameraDevice> server_device_;
  std::unique_ptr<CameraDevice> client_device_;

  // Synchronization manager
  std::unique_ptr<FrameSynchronizer> synchronizer_;

  // ROS2 Publishers
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  // Camera calibration configuration managers
  std::unique_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;
};

int main(int argc, char *argv[]) {
  // Set environment variable to explicitly point to the camera tuning file
  setenv("LIBCAMERA_RPI_TUNING_FILE", "/usr/share/libcamera/ipa/rpi/pisp/imx219.json", 1);
  // Set IPA configuration path to avoid conda environment mismatch
  setenv("LIBCAMERA_IPA_CONFIG_PATH", "/usr/share/libcamera/ipa", 1);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMX219StereoCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


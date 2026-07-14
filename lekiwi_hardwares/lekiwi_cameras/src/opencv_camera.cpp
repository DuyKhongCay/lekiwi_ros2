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
#include <thread>
#include <atomic>
#include <mutex>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

// OpenCV Headers
#include <opencv2/opencv.hpp>

// Base Class
#include "lekiwi_cameras/base_camera_node.hpp"

using namespace std::chrono_literals;

namespace lekiwi_cameras {

class OpenCVCamera : public BaseCameraNode {
public:
  explicit OpenCVCamera(const rclcpp::NodeOptions & options) 
  : BaseCameraNode("opencv_camera", options) {
    // Declare parameters with default values
    this->declare_parameter<std::string>("camera_name", "camera1");
    this->declare_parameter<std::string>("device_id", "0");
    this->declare_parameter<std::string>("device_path", "");
    this->declare_parameter<int>("capture_width", 640);
    this->declare_parameter<int>("capture_height", 480);
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
    this->declare_parameter<std::string>("camera_info_url", "");

    // Retrieve parameter values
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("device_id", device_id_);
    this->get_parameter("device_path", device_path_);
    this->get_parameter("capture_width", capture_width_);
    this->get_parameter("capture_height", capture_height_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("fps", fps_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("camera_info_url", camera_info_url_);

    // Populate base class parameters
    common_params_.width = width_;
    common_params_.height = height_;
    common_params_.fps = fps_;

    RCLCPP_INFO(this->get_logger(), "Initializing OpenCV Camera Component");
    RCLCPP_INFO(this->get_logger(), "Params: CameraName=%s, Capture=%dx%d, Publish=%dx%d, FPS=%.1f",
                camera_name_.c_str(), capture_width_, capture_height_, width_, height_, fps_);

    // Dynamic topic naming based on camera_name parameter
    std::string image_topic = "/cameras/" + camera_name_ + "/image_raw";
    std::string info_topic = "/cameras/" + camera_name_ + "/camera_info";

    // Initialize publishers
    image_pub_ = image_transport::create_publisher(this, image_topic);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);

    // Initialize Camera Info Manager
    info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);

    // Open video capture device
    if (!openCamera()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open camera device");
      rclcpp::shutdown();
      return;
    }

    // Start background capture thread to prevent blocking the ROS executor
    running_ = true;
    capture_thread_ = std::thread(&OpenCVCamera::captureLoop, this);
  }

  ~OpenCVCamera() override {
    // Stop capture thread and release camera resources
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
    RCLCPP_INFO(this->get_logger(), "Camera component shut down successfully");
  }

private:
  bool is_integer(const std::string & s) const {
    if (s.empty()) return false;
    for (char const & c : s) {
      if (!std::isdigit(c)) return false;
    }
    return true;
  }

  bool openCamera() {
    // Attempt to open device using path (e.g. /dev/video0 or pipeline string) if provided, 
    // otherwise auto-detect if device_id is an index or path
    if (!device_path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Opening camera using device path: %s", device_path_.c_str());
      cap_.open(device_path_, cv::CAP_ANY);
    } else if (is_integer(device_id_)) {
      int idx = std::stoi(device_id_);
      RCLCPP_INFO(this->get_logger(), "Opening camera using device ID index: %d", idx);
      cap_.open(idx, cv::CAP_ANY);
    } else {
      RCLCPP_INFO(this->get_logger(), "Opening camera using device ID path: %s", device_id_.c_str());
      cap_.open(device_id_, cv::CAP_ANY);
    }

    if (!cap_.isOpened()) {
      return false;
    }

    // Configure hardware capture properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, capture_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    // Log the actual hardware parameters set by OpenCV/driver backend
    double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(this->get_logger(), "Hardware configured actual: %dx%d (%.1f FPS)", 
                static_cast<int>(actual_width), static_cast<int>(actual_height), actual_fps);

    return true;
  }

  void captureLoop() {
    cv::Mat raw_frame;
    
    // Calculate target frame sleep time based on requested FPS to avoid CPU spin when driver doesn't limit rates
    auto frame_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / fps_));

    while (running_ && rclcpp::ok()) {
      auto start_time = std::chrono::steady_clock::now();

      if (!cap_.read(raw_frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
        std::this_thread::sleep_for(100ms);
        continue;
      }

      if (raw_frame.empty()) {
        continue;
      }

      // Process and publish the captured frame
      processAndPublish(raw_frame);

      // Throttling sleep if capture loop runs faster than target FPS
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed < frame_duration) {
        std::this_thread::sleep_for(frame_duration - elapsed);
      }
    }
  }

  void processAndPublish(cv::Mat &frame) {
    // 1. Auto-detect grayscale frame (1 channel) and convert it to 3 channels (BGR8)
    if (frame.channels() == 1) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Grayscale camera detected (1 channel). Auto-converting to BGR8 (3 channels).");
      cv::Mat color_frame;
      cv::cvtColor(frame, color_frame, cv::COLOR_GRAY2BGR);
      frame = color_frame;
    }

    // 2. Perform downsampling if capture resolution is larger than publish resolution
    if (frame.cols > width_ || frame.rows > height_) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Downsampling active: capturing at %dx%d, publishing at %dx%d", 
                       frame.cols, frame.rows, width_, height_);
      cv::Mat resized_frame;
      // Using cv::INTER_AREA for high-quality image downsampling to prevent aliasing artifacts
      cv::resize(frame, resized_frame, cv::Size(width_, height_), 0, 0, cv::INTER_AREA);
      frame = resized_frame;
    }

    // Prepare message header and publish
    auto stamp = this->now();
    publishImageAndInfo(frame, image_pub_, info_pub_, info_manager_, frame_id_, stamp);
  }

  // Node parameters
  std::string camera_name_;
  std::string device_id_;
  std::string device_path_;
  int capture_width_;
  int capture_height_;
  int width_;
  int height_;
  double fps_;
  std::string frame_id_;
  std::string camera_info_url_;

  // Thread control & capture
  std::atomic<bool> running_;
  std::thread capture_thread_;
  cv::VideoCapture cap_;

  // ROS2 Publishers & Managers
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
};

} // namespace lekiwi_cameras

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_cameras::OpenCVCamera)

#ifdef STANDALONE_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lekiwi_cameras::OpenCVCamera>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif

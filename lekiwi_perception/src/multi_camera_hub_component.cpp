// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "multi_camera_hub_component.hpp"

#include <gst/video/video.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "gst_hailo_meta.hpp"
#include "hailo/chess_vision_mapper.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

namespace lekiwi_perception
{
namespace
{

using Matrix3 = std::array<double, 9>;
using Matrix34 = std::array<double, 12>;

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

const char * mode_name(uint8_t mode)
{
  using CameraMode = lekiwi_interfaces::msg::CameraMode;
  switch (mode) {
    case CameraMode::STANDBY:
      return "STANDBY";
    case CameraMode::NAVIGATING:
      return "NAVIGATING";
    case CameraMode::CHESS_THINKING:
      return "CHESS_THINKING";
    case CameraMode::MANIPULATION_LEROBOT:
      return "MANIPULATION_LEROBOT";
    default:
      return "INVALID";
  }
}

rcl_interfaces::msg::ParameterDescriptor integer_descriptor(
  const std::string & description,
  int64_t minimum,
  int64_t maximum)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.integer_range.resize(1);
  descriptor.integer_range[0].from_value = minimum;
  descriptor.integer_range[0].to_value = maximum;
  descriptor.integer_range[0].step = 1;
  return descriptor;
}

Matrix3 multiply(const Matrix3 & left, const Matrix3 & right)
{
  Matrix3 result{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      for (std::size_t inner = 0; inner < 3U; ++inner) {
        result[row * 3U + column] += left[row * 3U + inner] * right[inner * 3U + column];
      }
    }
  }
  return result;
}

Matrix34 multiply(const Matrix3 & left, const Matrix34 & right)
{
  Matrix34 result{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 4U; ++column) {
      for (std::size_t inner = 0; inner < 3U; ++inner) {
        result[row * 4U + column] += left[row * 3U + inner] * right[inner * 4U + column];
      }
    }
  }
  return result;
}

Matrix3 pixel_rotation(int rotation, uint32_t width, uint32_t height)
{
  switch (rotation) {
    case 0:
      return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    case 90:
      return {0.0, -1.0, static_cast<double>(height - 1U),
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    case 180:
      return {-1.0, 0.0, static_cast<double>(width - 1U),
        0.0, -1.0, static_cast<double>(height - 1U), 0.0, 0.0, 1.0};
    case 270:
      return {0.0, 1.0, 0.0, -1.0, 0.0,
        static_cast<double>(width - 1U), 0.0, 0.0, 1.0};
    default:
      throw std::invalid_argument("Unsupported camera rotation");
  }
}

sensor_msgs::msg::CameraInfo transform_camera_info(
  sensor_msgs::msg::CameraInfo info,
  const std::string & frame_id,
  uint32_t capture_width,
  uint32_t capture_height,
  const GeomPlan & geometry)
{
  const uint32_t calibration_width = info.width == 0U ? capture_width : info.width;
  const uint32_t calibration_height = info.height == 0U ? capture_height : info.height;
  if (info.k[0] != 0.0) {
    const Matrix3 calibration_scale = {
      static_cast<double>(capture_width) / calibration_width, 0.0, 0.0,
      0.0, static_cast<double>(capture_height) / calibration_height, 0.0,
      0.0, 0.0, 1.0};
    const Matrix3 letterbox = {
      geometry.scale, 0.0, static_cast<double>(geometry.pre_rotation_pad_x),
      0.0, geometry.scale, static_cast<double>(geometry.pre_rotation_pad_y),
      0.0, 0.0, 1.0};
    const Matrix3 transform = multiply(
      pixel_rotation(
        geometry.rotation, geometry.pre_rotation_width, geometry.pre_rotation_height),
      multiply(letterbox, calibration_scale));
    info.k = multiply(transform, info.k);
    info.p = multiply(transform, info.p);
  }

  info.header.frame_id = frame_id;
  info.width = (geometry.rotation == 90 || geometry.rotation == 270) ?
    geometry.pre_rotation_height : geometry.pre_rotation_width;
  info.height = (geometry.rotation == 90 || geometry.rotation == 270) ?
    geometry.pre_rotation_width : geometry.pre_rotation_height;
  info.roi.x_offset = geometry.pad_x;
  info.roi.y_offset = geometry.pad_y;
  info.roi.width = geometry.active_width;
  info.roi.height = geometry.active_height;
  info.roi.do_rectify = false;
  return info;
}

}  // namespace

MultiCameraHubComponent::MultiCameraHubComponent(const rclcpp::NodeOptions & options)
: LifecycleNode("multi_camera_hub", options)
{
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }
  declare_parameters();
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  monitoring_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  RCLCPP_INFO(get_logger(), "MultiCameraHubComponent created with GStreamer %s",
      gst_version_string());
}

MultiCameraHubComponent::~MultiCameraHubComponent()
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string ignored;
  static_cast<void>(stop_all(ignored));
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
}

void MultiCameraHubComponent::declare_parameters()
{
  declare_parameter<bool>("use_test_sources", false);
  declare_parameter<int>(
    "trans_timeout_ms", 5000,
    integer_descriptor("Maximum pipeline transition and buffer-drain time", 100, 30000));
  declare_parameter<int>("transition_timeout_ms", -1);
  declare_parameter<int>(
    "status_period_ms", 1000,
    integer_descriptor("Camera hub status publication period", 100, 10000));
  declare_parameter<int>(
    "skew_warning_ms", 35,
    integer_descriptor("LeRobot inter-camera skew warning threshold", 1, 1000));
  declare_parameter<bool>("hailo.publish_debug_image", true);
  declare_parameter<std::string>("hailo.vdevice_group_id", "lekiwi_chess");

  const std::array<std::string, kStreamCount> prefixes = {
    "cameras.stereo_left", "cameras.stereo_right", "cameras.usb_wrist", "cameras.usb_side"};
  const std::array<std::string, kStreamCount> selectors = {
    "/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10",
    "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10",
    "/dev/wrist",
    "/dev/side"};
  const std::array<std::string, kStreamCount> frame_ids = {
    "stereo_left_optical", "stereo_right_optical",
    "wrist_camera_optical", "side_camera_optical"};
  const std::array<int, kStreamCount> capture_widths = {3280, 1640, 1280, 1280};
  const std::array<int, kStreamCount> capture_heights = {2464, 2464, 720, 720};
  const std::array<int, kStreamCount> output_widths = {640, 640, 640, 640};
  const std::array<int, kStreamCount> output_heights = {640, 480, 480, 480};
  const std::array<int, kStreamCount> frame_rates = {20, 30, 30, 30};
  const std::array<int, kStreamCount> rotations = {180, 0, 0, 0};

  for (std::size_t index = 0; index < kStreamCount; ++index) {
    const auto & prefix = prefixes[index];
    declare_parameter<std::string>(prefix + ".selector", selectors[index]);
    declare_parameter<std::string>(prefix + ".frame_id", frame_ids[index]);
    declare_parameter<std::string>(prefix + ".calibration_url", "");
    declare_parameter<int>(
      prefix + ".capture_width", capture_widths[index],
      integer_descriptor("Native capture width", 2, 8192));
    declare_parameter<int>(
      prefix + ".capture_height", capture_heights[index],
      integer_descriptor("Native capture height", 2, 8192));
    declare_parameter<int>(
      prefix + ".output_width", output_widths[index],
      integer_descriptor("Published image width", 2, 4096));
    declare_parameter<int>(
      prefix + ".output_height", output_heights[index],
      integer_descriptor("Published image height", 2, 4096));
    declare_parameter<int>(
      prefix + ".fps", frame_rates[index],
      integer_descriptor("Requested capture frame rate", 1, 120));
    declare_parameter<int>(
      prefix + ".rotation", rotations[index],
      integer_descriptor("Clockwise image rotation in degrees", 0, 270));
  }
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  std::string error;
  if (!load_configuration(error)) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.c_str());
    set_error(error);
    return CallbackReturn::FAILURE;
  }

  try {
    common_clock_ = gst_system_clock_obtain();
    if (common_clock_ == nullptr) {
      throw std::runtime_error("Failed to obtain the common GStreamer system clock");
    }
    calibrate_clock_bridge();
    for (std::size_t index = 0; index < kStreamCount; ++index) {
      const bool require_even = index == stream_index(StreamId::kStereoLeft);
      const auto & config = streams_[index];
      geometry_plans_[index] = make_geom_plan(config, require_even);
      sensor_msgs::msg::CameraInfo calibration;
      if (!config.calibration_url.empty()) {
        camera_info_manager::CameraInfoManager manager(this, stream_name(static_cast<StreamId>(index)),
          config.calibration_url);
        if (!manager.loadCameraInfo(config.calibration_url)) {
          throw std::runtime_error("Failed to load calibration URL: " + config.calibration_url);
        }
        calibration = manager.getCameraInfo();
      }
      camera_infos_[index] = transform_camera_info(
        std::move(calibration), config.frame_id, config.capture_width, config.capture_height,
        geometry_plans_[index]);
    }
    create_ros_entities();
  } catch (const std::exception & exception) {
    error = exception.what();
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.c_str());
    set_error(error);
    reset_ros_entities();
    if (common_clock_ != nullptr) {
      gst_object_unref(common_clock_);
      common_clock_ = nullptr;
    }
    return CallbackReturn::FAILURE;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    effective_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    prev_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    hailo_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED;
    lerobot_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED;
    last_error_.clear();
    set_mode_delay_ms_ = -1.0F;
  }
  reset_metrics();
  RCLCPP_INFO(get_logger(), "Camera hub configured without opening hardware");
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & publisher : image_pubs_) {
    if (publisher) {
      publisher->on_activate();
    }
  }
  for (auto & publisher : info_pubs_) {
    if (publisher) {
      publisher->on_activate();
    }
  }
  fen_pub_->on_activate();
  detections_pub_->on_activate();
  if (debug_image_pub_) {
    debug_image_pub_->on_activate();
    debug_info_pub_->on_activate();
  }
  status_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Camera hub active in STANDBY mode");
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  const bool stopped = stop_all(error);
  reset_metrics();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = effective_mode_;
    effective_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    if (!stopped) {
      last_error_ = error;
    }
  }
  for (auto & publisher : image_pubs_) {
    if (publisher) {
      publisher->on_deactivate();
    }
  }
  for (auto & publisher : info_pubs_) {
    if (publisher) {
      publisher->on_deactivate();
    }
  }
  fen_pub_->on_deactivate();
  detections_pub_->on_deactivate();
  if (debug_image_pub_) {
    debug_image_pub_->on_deactivate();
    debug_info_pub_->on_deactivate();
  }
  status_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Camera hub deactivated and camera pipelines released");
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  static_cast<void>(stop_all(error));
  reset_ros_entities();
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
  RCLCPP_INFO(get_logger(), "Camera hub cleaned up");
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  static_cast<void>(stop_all(error));
  reset_ros_entities();
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_error(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  static_cast<void>(stop_all(error));
  reset_ros_entities();
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
  return CallbackReturn::SUCCESS;
}

bool MultiCameraHubComponent::load_configuration(std::string & error)
{
  bool use_sim_time = false;
  get_parameter("use_sim_time", use_sim_time);
  if (use_sim_time) {
    error = "Physical camera clock mapping requires use_sim_time=false";
    return false;
  }

  use_test_sources_ = get_parameter("use_test_sources").as_bool();
  publish_debug_image_ = get_parameter("hailo.publish_debug_image").as_bool();
  hailo_config_.vdevice_group_id = get_parameter("hailo.vdevice_group_id").as_string();
  const int legacy_trans_timeout_ms = get_parameter("transition_timeout_ms").as_int();
  const int trans_timeout_ms = get_parameter("trans_timeout_ms").as_int();
  if (legacy_trans_timeout_ms >= 100) {
    RCLCPP_WARN(
      get_logger(),
      "Parameter transition_timeout_ms is deprecated; use trans_timeout_ms instead");
    transition_timeout_ = std::chrono::milliseconds(legacy_trans_timeout_ms);
  } else {
    transition_timeout_ = std::chrono::milliseconds(trans_timeout_ms);
  }
  status_period_ = std::chrono::milliseconds(get_parameter("status_period_ms").as_int());
  skew_warning_ms_ = static_cast<double>(get_parameter("skew_warning_ms").as_int());

  const auto package_share = ament_index_cpp::get_package_share_directory("lekiwi_perception");
  hailo_config_.board_hef_path = package_share + "/resources/models/yolov8n-seg.hef";
  hailo_config_.pcs_hef_path = package_share + "/resources/models/yolo11n.hef";
  for (const auto & path : {
      hailo_config_.board_hef_path, hailo_config_.pcs_hef_path})
  {
    if (!std::filesystem::exists(path)) {
      error = "Missing Hailo resource: " + path;
      return false;
    }
  }

  const std::array<std::string, kStreamCount> prefixes = {
    "cameras.stereo_left", "cameras.stereo_right", "cameras.usb_wrist", "cameras.usb_side"};
  for (std::size_t index = 0; index < kStreamCount; ++index) {
    const auto & prefix = prefixes[index];
    auto & config = streams_[index];
    config.selector = get_parameter(prefix + ".selector").as_string();
    config.frame_id = get_parameter(prefix + ".frame_id").as_string();
    config.calibration_url = get_parameter(prefix + ".calibration_url").as_string();
    config.capture_width = static_cast<uint32_t>(
      get_parameter(prefix + ".capture_width").as_int());
    config.capture_height = static_cast<uint32_t>(
      get_parameter(prefix + ".capture_height").as_int());
    config.output_width = static_cast<uint32_t>(
      get_parameter(prefix + ".output_width").as_int());
    config.output_height = static_cast<uint32_t>(
      get_parameter(prefix + ".output_height").as_int());
    config.fps = static_cast<uint32_t>(get_parameter(prefix + ".fps").as_int());
    config.rotation = static_cast<int>(get_parameter(prefix + ".rotation").as_int());

    if (!valid_rotation(config.rotation)) {
      error = prefix + ".rotation must be one of 0, 90, 180, or 270";
      return false;
    }
    if (!use_test_sources_ && config.selector.empty()) {
      error = prefix + ".selector cannot be empty in hardware mode";
      return false;
    }
    if (config.frame_id.empty()) {
      error = prefix + ".frame_id cannot be empty";
      return false;
    }
  }

  const auto & left = streams_[stream_index(StreamId::kStereoLeft)];
  if ((left.output_width % 2U) != 0U || (left.output_height % 2U) != 0U) {
    error = "Hailo input dimensions must be even";
    return false;
  }
  return true;
}

void MultiCameraHubComponent::create_ros_entities()
{
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  image_pubs_[stream_index(StreamId::kStereoRight)] =
    create_publisher<sensor_msgs::msg::Image>("cameras/stereo_right/image_raw", sensor_qos);
  image_pubs_[stream_index(StreamId::kUsbWrist)] =
    create_publisher<sensor_msgs::msg::Image>("cameras/usb_wrist/image_raw", sensor_qos);
  image_pubs_[stream_index(StreamId::kUsbSide)] =
    create_publisher<sensor_msgs::msg::Image>("cameras/usb_side/image_raw", sensor_qos);

  info_pubs_[stream_index(StreamId::kStereoRight)] =
    create_publisher<sensor_msgs::msg::CameraInfo>("cameras/stereo_right/camera_info", sensor_qos);
  info_pubs_[stream_index(StreamId::kUsbWrist)] =
    create_publisher<sensor_msgs::msg::CameraInfo>("cameras/usb_wrist/camera_info", sensor_qos);
  info_pubs_[stream_index(StreamId::kUsbSide)] =
    create_publisher<sensor_msgs::msg::CameraInfo>("cameras/usb_side/camera_info", sensor_qos);

  fen_pub_ = create_publisher<std_msgs::msg::String>("chess/fen", sensor_qos);
  detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "chess/detections_2d", sensor_qos);
  if (publish_debug_image_) {
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("chess/debug_image", sensor_qos);
    debug_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "chess/camera_info", sensor_qos);
  }

  status_pub_ = create_publisher<lekiwi_interfaces::msg::CamHubStatus>(
    "camera_hub/status", rclcpp::QoS(1).reliable().transient_local());
  mode_service_ = create_service<lekiwi_interfaces::srv::SetCamMode>(
    "camera_hub/set_mode",
    std::bind(
      &MultiCameraHubComponent::handle_set_mode, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    control_group_);
  status_timer_ = create_wall_timer(
    status_period_, std::bind(&MultiCameraHubComponent::publish_status, this), monitoring_group_);
  bus_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MultiCameraHubComponent::poll_pipe_errors, this), control_group_);

  stream_processor_ = std::make_unique<CamHubStreamProcessor>(
    CamHubStreamProcessor::Bindings{
      get_logger(), get_clock(), streams_, camera_infos_, image_pubs_, info_pubs_, fen_pub_,
      detections_pub_, debug_image_pub_, debug_info_pub_, gst_to_ros_offset_ns_, skew_warning_ms_});
}

void MultiCameraHubComponent::reset_ros_entities()
{
  stream_processor_.reset();
  if (status_timer_) {
    status_timer_->cancel();
  }
  if (bus_timer_) {
    bus_timer_->cancel();
  }
  status_timer_.reset();
  bus_timer_.reset();
  mode_service_.reset();
  fen_pub_.reset();
  detections_pub_.reset();
  debug_image_pub_.reset();
  debug_info_pub_.reset();
  status_pub_.reset();
  for (auto & publisher : image_pubs_) {
    publisher.reset();
  }
  for (auto & publisher : info_pubs_) {
    publisher.reset();
  }
}

void MultiCameraHubComponent::calibrate_clock_bridge()
{
  GstClockTime best_span = GST_CLOCK_TIME_NONE;
  int64_t best_offset = 0;
  for (int sample = 0; sample < 16; ++sample) {
    const GstClockTime before = gst_clock_get_time(common_clock_);
    const int64_t ros_now = get_clock()->now().nanoseconds();
    const GstClockTime after = gst_clock_get_time(common_clock_);
    const GstClockTime span = after - before;
    if (!GST_CLOCK_TIME_IS_VALID(best_span) || span < best_span) {
      const auto midpoint = before + (span / 2U);
      best_offset = ros_now - static_cast<int64_t>(midpoint);
      best_span = span;
    }
  }
  gst_to_ros_offset_ns_ = best_offset;
}

void MultiCameraHubComponent::reset_metrics()
{
  if (stream_processor_) {
    stream_processor_->reset_metrics();
  }
}

void MultiCameraHubComponent::handle_set_mode(
  const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
  std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  if (!lifecycle_active()) {
    response->success = false;
    response->message = "Camera hub lifecycle node is not active";
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    response->applied_mode.value = effective_mode_;
    return;
  }

  response->success = apply_mode(request->requested_mode.value, response->message);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    response->applied_mode.value = effective_mode_;
  }
  publish_status();
}

bool MultiCameraHubComponent::apply_mode(uint8_t requested_mode, std::string & message)
{
  using CameraMode = lekiwi_interfaces::msg::CameraMode;
  if (requested_mode > CameraMode::MANIPULATION_LEROBOT) {
    message = "Invalid camera mode value";
    return false;
  }

  uint8_t src_mode = CameraMode::STANDBY;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    src_mode = effective_mode_;
    if (src_mode == requested_mode && last_error_.empty()) {
      message = std::string("Mode already active: ") + mode_name(requested_mode);
      return true;
    }
  }

  const auto trans_started = std::chrono::steady_clock::now();
  std::string stop_error;
  bool src_stopped = true;
  if (src_mode == CameraMode::CHESS_THINKING) {
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    src_stopped = stop_hailo(stop_error);
  } else if (src_mode == CameraMode::MANIPULATION_LEROBOT) {
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    src_stopped = stop_lerobot(stop_error);
  }
  if (!src_stopped) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_error_ = stop_error;
    message = stop_error;
    return false;
  }

  reset_metrics();
  std::string start_error;
  bool started = true;
  if (requested_mode == CameraMode::CHESS_THINKING) {
    started = start_hailo(start_error);
  } else if (requested_mode == CameraMode::MANIPULATION_LEROBOT) {
    started = start_lerobot(start_error);
  }
  if (!started) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = effective_mode_;
    effective_mode_ = CameraMode::STANDBY;
    last_error_ = start_error;
    message = start_error;
    return false;
  }

  const auto trans_elapsed = std::chrono::steady_clock::now() - trans_started;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = src_mode;
    effective_mode_ = requested_mode;
    record_set_mode_delay(trans_elapsed);
    last_error_.clear();
  }
  message = std::string("Applied mode ") + mode_name(requested_mode);
  RCLCPP_INFO(get_logger(), "%s in %.1f ms", message.c_str(),
    std::chrono::duration<double, std::milli>(trans_elapsed).count());
  return true;
}

bool MultiCameraHubComponent::start_hailo(std::string & error)
{
  if (hailo_pipe_) {
    error = "hailo_perception_pipeline teardown is still pending";
    set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STARTING);
  const auto generation = generation_.load(std::memory_order_acquire);
  auto callback = [this, generation](
    StreamId stream, GstAppSink * sink, GstElement * pipeline)
    {
      if (generation == generation_.load(std::memory_order_acquire)) {
        if (stream_processor_) {
          stream_processor_->handle_sample(stream, sink, pipeline);
        }
      }
    };
  hailo_pipe_ = std::make_unique<GstPipeController>(
    "hailo_perception_pipeline",
    std::vector<GstPipeController::SinkSpec>{
      {"hailo_appsink", StreamId::kStereoLeft}},
    callback);
  const auto description = build_hailo_pipe(
    streams_[stream_index(StreamId::kStereoLeft)],
    geometry_plans_[stream_index(StreamId::kStereoLeft)],
    hailo_config_,
    use_test_sources_);
  RCLCPP_INFO(get_logger(), "Starting hailo_perception_pipeline: %s", description.c_str());
  if (!hailo_pipe_->start(description, common_clock_, transition_timeout_, error)) {
    set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_RUNNING);
  return true;
}

bool MultiCameraHubComponent::start_lerobot(std::string & error)
{
  if (lerobot_pipe_) {
    error = "lerobot_perception_pipeline teardown is still pending";
    set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STARTING);
  const auto generation = generation_.load(std::memory_order_acquire);
  auto callback = [this, generation](
    StreamId stream, GstAppSink * sink, GstElement * pipeline)
    {
      if (generation == generation_.load(std::memory_order_acquire)) {
        if (stream_processor_) {
          stream_processor_->handle_sample(stream, sink, pipeline);
        }
      }
    };
  lerobot_pipe_ = std::make_unique<GstPipeController>(
    "lerobot_perception_pipeline",
    std::vector<GstPipeController::SinkSpec>{
      {"stereo_right_sink", StreamId::kStereoRight},
      {"usb_wrist_sink", StreamId::kUsbWrist},
      {"usb_side_sink", StreamId::kUsbSide}},
    callback);
  const auto description = build_lerobot_pipe(
    streams_[stream_index(StreamId::kStereoRight)],
    geometry_plans_[stream_index(StreamId::kStereoRight)],
    streams_[stream_index(StreamId::kUsbWrist)],
    geometry_plans_[stream_index(StreamId::kUsbWrist)],
    streams_[stream_index(StreamId::kUsbSide)],
    geometry_plans_[stream_index(StreamId::kUsbSide)],
    use_test_sources_);
  RCLCPP_INFO(get_logger(), "Starting lerobot_perception_pipeline: %s", description.c_str());
  if (!lerobot_pipe_->start(description, common_clock_, transition_timeout_, error)) {
    set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_RUNNING);
  return true;
}

bool MultiCameraHubComponent::stop_hailo(std::string & error)
{
  if (!hailo_pipe_) {
    set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED);
    return true;
  }
  set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPING);
  const bool stopped = hailo_pipe_->stop(transition_timeout_, error);
  if (stopped) {
    hailo_pipe_.reset();
  }
  set_pipe_state(
    true, stopped ? lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED :
    lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
  return stopped;
}

bool MultiCameraHubComponent::stop_lerobot(std::string & error)
{
  if (!lerobot_pipe_) {
    set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED);
    return true;
  }
  set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPING);
  const bool stopped = lerobot_pipe_->stop(transition_timeout_, error);
  if (stopped) {
    lerobot_pipe_.reset();
  }
  set_pipe_state(
    false, stopped ? lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED :
    lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
  return stopped;
}

bool MultiCameraHubComponent::stop_all(std::string & error)
{
  std::string hailo_error;
  std::string lerobot_error;
  const bool hailo_stopped = stop_hailo(hailo_error);
  const bool lerobot_stopped = stop_lerobot(lerobot_error);
  if (!hailo_stopped || !lerobot_stopped) {
    error = !hailo_error.empty() ? hailo_error : lerobot_error;
    return false;
  }
  return true;
}

void MultiCameraHubComponent::poll_pipe_errors()
{
  if (!lifecycle_active()) {
    return;
  }
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  std::string hailo_diags;
  std::string lerobot_diags;
  const bool hailo_error = hailo_pipe_ && hailo_pipe_->poll_error(hailo_diags);
  const bool lerobot_error = lerobot_pipe_ && lerobot_pipe_->poll_error(lerobot_diags);
  if (!hailo_error && !lerobot_error) {
    if (!hailo_diags.empty()) {
      RCLCPP_WARN(get_logger(), "%s", hailo_diags.c_str());
    }
    if (!lerobot_diags.empty()) {
      RCLCPP_WARN(get_logger(), "%s", lerobot_diags.c_str());
    }
    return;
  }

  const std::string & error = hailo_error ? hailo_diags : lerobot_diags;
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string cleanup_error;
  static_cast<void>(stop_all(cleanup_error));
  reset_metrics();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = effective_mode_;
    effective_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    if (hailo_error) {
      hailo_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR;
    }
    if (lerobot_error) {
      lerobot_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR;
    }
  }
  set_error(error);
}

void MultiCameraHubComponent::publish_status()
{
  if (!status_pub_ || !lifecycle_active()) {
    return;
  }

  auto message = std::make_unique<lekiwi_interfaces::msg::CamHubStatus>();
  message->header.stamp = now();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    message->effective_mode.value = effective_mode_;
    message->prev_mode.value = prev_mode_;
    message->hailo_pipe_state = hailo_state_;
    message->lerobot_pipe_state = lerobot_state_;
    message->last_error = last_error_;
    message->set_mode_delay_ms = set_mode_delay_ms_;
  }
  const auto metrics = stream_processor_ ? stream_processor_->snapshot_metrics() :
    CamHubStreamProcessor::MetricsSnapshot{};
  message->hailo_fps = metrics.rates[stream_index(StreamId::kStereoLeft)];
  message->stereo_right_fps = metrics.rates[stream_index(StreamId::kStereoRight)];
  message->usb_wrist_fps = metrics.rates[stream_index(StreamId::kUsbWrist)];
  message->usb_side_fps = metrics.rates[stream_index(StreamId::kUsbSide)];
  message->hailo_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kStereoLeft)];
  message->stereo_right_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kStereoRight)];
  message->usb_wrist_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kUsbWrist)];
  message->usb_side_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kUsbSide)];
  message->lerobot_skew_p50_ms = metrics.lerobot_skew_p50_ms;
  message->lerobot_skew_p95_ms = metrics.lerobot_skew_p95_ms;
  message->lerobot_triplet_match_count = metrics.lerobot_triplet_match_count;
  message->lerobot_triplet_discarded_frame_count = metrics.lerobot_triplet_discarded_frame_count;
  status_pub_->publish(std::move(message));
}

void MultiCameraHubComponent::set_pipe_state(bool hailo, uint8_t state)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (hailo) {
    hailo_state_ = state;
  } else {
    lerobot_state_ = state;
  }
}

void MultiCameraHubComponent::record_set_mode_delay(std::chrono::steady_clock::duration elapsed)
{
  set_mode_delay_ms_ = static_cast<float>(
    std::chrono::duration<double, std::milli>(elapsed).count());
}

void MultiCameraHubComponent::set_error(const std::string & error)
{
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_error_ = error;
  }
  publish_status();
}

bool MultiCameraHubComponent::lifecycle_active() const
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

}  // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::MultiCameraHubComponent)

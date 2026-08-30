// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "camera_hub_config.hpp"

#include <filesystem>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace lekiwi_perception
{
namespace
{

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

}  // namespace

void declare_hub_parameters(rclcpp_lifecycle::LifecycleNode * node)
{
  node->declare_parameter<bool>("use_test_sources", false);
  node->declare_parameter<int>(
    "trans_timeout_ms", 5000,
    integer_descriptor("Maximum pipeline transition and buffer-drain time", 100, 30000));
  node->declare_parameter<int>("transition_timeout_ms", -1);
  node->declare_parameter<int>(
    "status_period_ms", 1000,
    integer_descriptor("Camera hub status publication period", 100, 10000));
  node->declare_parameter<int>(
    "skew_warning_ms", 35,
    integer_descriptor("LeRobot inter-camera skew warning threshold", 1, 1000));
  node->declare_parameter<bool>("hailo.publish_debug_image", true);
  node->declare_parameter<std::string>("hailo.vdevice_group_id", "lekiwi_chess");

  const std::array<std::string, MultiCameraHubConfig::kStreamCount> prefixes = {
    "cameras.stereo_left", "cameras.stereo_right", "cameras.usb_wrist", "cameras.usb_side"};
  const std::array<std::string, MultiCameraHubConfig::kStreamCount> selectors = {
    "/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10",
    "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10",
    "/dev/wrist",
    "/dev/side"};
  const std::array<std::string, MultiCameraHubConfig::kStreamCount> frame_ids = {
    "stereo_left_optical", "stereo_right_optical",
    "wrist_camera_optical", "side_camera_optical"};
  const std::array<int, MultiCameraHubConfig::kStreamCount> capture_widths = {3280, 1640, 1280, 1280};
  const std::array<int, MultiCameraHubConfig::kStreamCount> capture_heights = {2464, 2464, 720, 720};
  const std::array<int, MultiCameraHubConfig::kStreamCount> output_widths = {640, 640, 640, 640};
  const std::array<int, MultiCameraHubConfig::kStreamCount> output_heights = {640, 480, 480, 480};
  const std::array<int, MultiCameraHubConfig::kStreamCount> frame_rates = {20, 30, 30, 30};
  const std::array<int, MultiCameraHubConfig::kStreamCount> rotations = {180, 0, 0, 0};

  for (std::size_t index = 0; index < MultiCameraHubConfig::kStreamCount; ++index) {
    const auto & prefix = prefixes[index];
    node->declare_parameter<std::string>(prefix + ".selector", selectors[index]);
    node->declare_parameter<std::string>(prefix + ".frame_id", frame_ids[index]);
    node->declare_parameter<std::string>(prefix + ".calibration_url", "");
    node->declare_parameter<int>(
      prefix + ".capture_width", capture_widths[index],
      integer_descriptor("Native capture width", 2, 8192));
    node->declare_parameter<int>(
      prefix + ".capture_height", capture_heights[index],
      integer_descriptor("Native capture height", 2, 8192));
    node->declare_parameter<int>(
      prefix + ".output_width", output_widths[index],
      integer_descriptor("Published image width", 2, 4096));
    node->declare_parameter<int>(
      prefix + ".output_height", output_heights[index],
      integer_descriptor("Published image height", 2, 4096));
    node->declare_parameter<int>(
      prefix + ".fps", frame_rates[index],
      integer_descriptor("Requested capture frame rate", 1, 120));
    node->declare_parameter<int>(
      prefix + ".rotation", rotations[index],
      integer_descriptor("Clockwise image rotation in degrees", 0, 270));
  }
}

bool load_hub_configuration(
  rclcpp_lifecycle::LifecycleNode * node,
  MultiCameraHubConfig & config,
  std::string & error)
{
  bool use_sim_time = false;
  node->get_parameter("use_sim_time", use_sim_time);
  if (use_sim_time) {
    error = "Physical camera clock mapping requires use_sim_time=false";
    return false;
  }

  config.use_test_sources = node->get_parameter("use_test_sources").as_bool();
  config.publish_debug_image = node->get_parameter("hailo.publish_debug_image").as_bool();
  config.hailo_config.vdevice_group_id = node->get_parameter("hailo.vdevice_group_id").as_string();
  const int legacy_trans_timeout_ms = node->get_parameter("transition_timeout_ms").as_int();
  const int trans_timeout_ms = node->get_parameter("trans_timeout_ms").as_int();
  if (legacy_trans_timeout_ms >= 100) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter transition_timeout_ms is deprecated; use trans_timeout_ms instead");
    config.transition_timeout = std::chrono::milliseconds(legacy_trans_timeout_ms);
  } else {
    config.transition_timeout = std::chrono::milliseconds(trans_timeout_ms);
  }
  config.status_period = std::chrono::milliseconds(
    node->get_parameter("status_period_ms").as_int());
  config.skew_warning_ms = static_cast<double>(
    node->get_parameter("skew_warning_ms").as_int());

  const auto package_share = ament_index_cpp::get_package_share_directory("lekiwi_perception");
  config.hailo_config.board_hef_path = package_share + "/resources/models/yolov8n-seg.hef";
  config.hailo_config.pcs_hef_path = package_share + "/resources/models/yolo11n.hef";
  for (const auto & path : {
      config.hailo_config.board_hef_path, config.hailo_config.pcs_hef_path})
  {
    if (!std::filesystem::exists(path)) {
      error = "Missing Hailo resource: " + path;
      return false;
    }
  }

  const std::array<std::string, MultiCameraHubConfig::kStreamCount> prefixes = {
    "cameras.stereo_left", "cameras.stereo_right", "cameras.usb_wrist", "cameras.usb_side"};
  for (std::size_t index = 0; index < MultiCameraHubConfig::kStreamCount; ++index) {
    const auto & prefix = prefixes[index];
    auto & stream = config.streams[index];
    stream.selector = node->get_parameter(prefix + ".selector").as_string();
    stream.frame_id = node->get_parameter(prefix + ".frame_id").as_string();
    stream.calibration_url = node->get_parameter(prefix + ".calibration_url").as_string();
    stream.capture_width = static_cast<uint32_t>(
      node->get_parameter(prefix + ".capture_width").as_int());
    stream.capture_height = static_cast<uint32_t>(
      node->get_parameter(prefix + ".capture_height").as_int());
    stream.output_width = static_cast<uint32_t>(
      node->get_parameter(prefix + ".output_width").as_int());
    stream.output_height = static_cast<uint32_t>(
      node->get_parameter(prefix + ".output_height").as_int());
    stream.fps = static_cast<uint32_t>(node->get_parameter(prefix + ".fps").as_int());
    stream.rotation = static_cast<int>(node->get_parameter(prefix + ".rotation").as_int());

    if (!valid_rotation(stream.rotation)) {
      error = prefix + ".rotation must be one of 0, 90, 180, or 270";
      return false;
    }
    if (!config.use_test_sources && stream.selector.empty()) {
      error = prefix + ".selector cannot be empty in hardware mode";
      return false;
    }
    if (stream.frame_id.empty()) {
      error = prefix + ".frame_id cannot be empty";
      return false;
    }
  }

  const auto & left = config.streams[stream_index(StreamId::kStereoLeft)];
  if ((left.output_width % 2U) != 0U || (left.output_height % 2U) != 0U) {
    error = "Hailo input dimensions must be even";
    return false;
  }
  return true;
}

}  // namespace lekiwi_perception

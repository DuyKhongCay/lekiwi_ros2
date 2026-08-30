// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__CAMERA_HUB_CONFIG_HPP_
#define LEKIWI_PERCEPTION__CAMERA_HUB_CONFIG_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

#include "gstreamer/pipe_builder.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lekiwi_perception
{

struct MultiCameraHubConfig
{
  static constexpr std::size_t kStreamCount = 4U;

  std::array<CamStreamConfig, kStreamCount> streams{};
  HailoPipeConfig hailo_config{};
  bool use_test_sources{false};
  bool publish_debug_image{true};
  std::chrono::milliseconds transition_timeout{5000};
  std::chrono::milliseconds status_period{1000};
  double skew_warning_ms{35.0};
};

void declare_hub_parameters(rclcpp_lifecycle::LifecycleNode * node);

[[nodiscard]] bool load_hub_configuration(
  rclcpp_lifecycle::LifecycleNode * node,
  MultiCameraHubConfig & config,
  std::string & error);

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__CAMERA_HUB_CONFIG_HPP_

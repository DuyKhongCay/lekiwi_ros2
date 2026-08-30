// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__CAMERA_GEOMETRY_HPP_
#define LEKIWI_PERCEPTION__CAMERA_GEOMETRY_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "gstreamer/pipe_builder.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace lekiwi_perception
{

using Matrix3 = std::array<double, 9>;
using Matrix34 = std::array<double, 12>;

[[nodiscard]] Matrix3 multiply(const Matrix3 & left, const Matrix3 & right) noexcept;
[[nodiscard]] Matrix34 multiply(const Matrix3 & left, const Matrix34 & right) noexcept;
[[nodiscard]] Matrix3 pixel_rotation(int rotation, uint32_t width, uint32_t height);

[[nodiscard]] sensor_msgs::msg::CameraInfo transform_camera_info(
  sensor_msgs::msg::CameraInfo info,
  const std::string & frame_id,
  uint32_t capture_width,
  uint32_t capture_height,
  const GeomPlan & geometry);

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__CAMERA_GEOMETRY_HPP_

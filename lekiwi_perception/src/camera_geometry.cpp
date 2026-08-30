// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "camera_geometry.hpp"

#include <stdexcept>
#include <utility>

namespace lekiwi_perception
{

/**
 * \brief Multiplies two 3-by-3 matrices.
 *
 * \param left Left matrix operand.
 * \param right Right matrix operand.
 * \return Product of the matrix operands.
 */
Matrix3 multiply(const Matrix3 & left, const Matrix3 & right) noexcept
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

/**
 * \brief Multiplies a 3-by-3 matrix by a 3-by-4 matrix.
 *
 * \param left Left 3-by-3 matrix operand.
 * \param right Right 3-by-4 matrix operand.
 * \return Product of the matrix operands.
 */
Matrix34 multiply(const Matrix3 & left, const Matrix34 & right) noexcept
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

/**
 * \brief Creates a homogeneous transform for a clockwise pixel rotation.
 *
 * \param rotation Clockwise rotation in degrees: 0, 90, 180, or 270.
 * \param width Width of the unrotated image in pixels.
 * \param height Height of the unrotated image in pixels.
 * \return Pixel-coordinate transformation matrix.
 * \throws std::invalid_argument If \p rotation is unsupported.
 */
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

/**
 * \brief Transforms camera calibration and ROI metadata for a geometry plan.
 *
 * \param info Camera information copied and updated by this function.
 * \param frame_id Frame ID assigned to the returned camera information.
 * \param capture_width Width of the captured image in pixels.
 * \param capture_height Height of the captured image in pixels.
 * \param geometry Scale, padding, and rotation applied to the image.
 * \return Camera information matching the transformed image geometry.
 */
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

}  // namespace lekiwi_perception

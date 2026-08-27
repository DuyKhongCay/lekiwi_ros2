// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__PIPE_BUILDER_HPP_
#define LEKIWI_PERCEPTION__PIPE_BUILDER_HPP_

#include <cstdint>
#include <string>

namespace lekiwi_perception
{

enum class StreamId : uint8_t
{
  kStereoLeft = 0,
  kStereoRight = 1,
  kUsbWrist = 2,
  kUsbSide = 3,
};

struct CamStreamConfig
{
  std::string selector;
  std::string frame_id;
  std::string calibration_url;
  uint32_t capture_width{0};
  uint32_t capture_height{0};
  uint32_t output_width{0};
  uint32_t output_height{0};
  uint32_t fps{0};
  int rotation{0};
};

struct GeomPlan
{
  uint32_t pre_rotation_width{0};
  uint32_t pre_rotation_height{0};
  uint32_t active_width{0};
  uint32_t active_height{0};
  uint32_t pad_x{0};
  uint32_t pad_y{0};
  uint32_t pre_rotation_pad_x{0};
  uint32_t pre_rotation_pad_y{0};
  double scale{1.0};
  int rotation{0};
};

struct HailoPipeConfig
{
  std::string board_hef_path;
  std::string pcs_hef_path;
  std::string vdevice_group_id{"lekiwi_chess"};
};

[[nodiscard]] bool valid_rotation(int rotation) noexcept;
[[nodiscard]] std::string rot_method(int rotation);
[[nodiscard]] GeomPlan make_geom_plan(
  const CamStreamConfig & config,
  bool even_geometry);
[[nodiscard]] std::string build_hailo_pipe(
  const CamStreamConfig & left,
  const GeomPlan & geometry,
  const HailoPipeConfig & hailo,
  bool use_test_sources);
[[nodiscard]] std::string build_lerobot_pipe(
  const CamStreamConfig & right, const GeomPlan & right_geometry,
  const CamStreamConfig & wrist, const GeomPlan & wrist_geometry,
  const CamStreamConfig & side, const GeomPlan & side_geometry,
  bool use_test_sources);

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__PIPE_BUILDER_HPP_

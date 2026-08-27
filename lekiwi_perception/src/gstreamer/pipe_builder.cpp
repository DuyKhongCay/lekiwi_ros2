// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "gstreamer/pipe_builder.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lekiwi_perception
{
namespace
{

std::string quoted(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return '"' + escaped + '"';
}

std::string latest_frame_queue(const std::string & name)
{
  return "queue name=" + name +
         " leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0";
}

std::string hailo_queue(const std::string & name)
{
  return "queue name=" + name +
         " leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0";
}

std::string test_source(
  const std::string & name, uint32_t width, uint32_t height, uint32_t fps, int pattern)
{
  std::ostringstream stream;
  stream << "videotestsrc name=" << name << " is-live=true pattern=" << pattern
         << " ! video/x-raw,width=" << width
         << ",height=" << height
         << ",framerate=" << fps << "/1";
  return stream.str();
}

std::string csi_source(const CamStreamConfig & config, const std::string & name)
{
  std::ostringstream stream;
  stream << "libcamerasrc name=" << name;
  if (!config.selector.empty()) {
    stream << " camera-name=" << quoted(config.selector);
  }
  stream << " ! video/x-raw,format=NV12,width=" << config.capture_width
         << ",height=" << config.capture_height
         << ",framerate=" << config.fps << "/1";
  return stream.str();
}

std::string usb_source(
  const CamStreamConfig & config,
  const std::string & name,
  bool use_test_sources,
  int pattern)
{
  std::ostringstream stream;
  if (use_test_sources) {
    stream << test_source(name, config.capture_width, config.capture_height, config.fps, pattern)
           << " ! jpegenc ! " << latest_frame_queue(name + "_queue")
           << " ! jpegdec ! video/x-raw,format=I420";
  } else {
    stream << "v4l2src name=" << name << " device=" << quoted(config.selector)
           << " io-mode=mmap ! image/jpeg,width=" << config.capture_width
           << ",height=" << config.capture_height
           << ",framerate=" << config.fps
           << "/1 ! " << latest_frame_queue(name + "_queue")
           << " ! jpegdec ! video/x-raw,format=I420";
  }
  return stream.str();
}

std::string lerobot_transform(
  const CamStreamConfig & config,
  const GeomPlan & geometry,
  const std::string & sink_name,
  const std::string & decoded_format)
{
  std::ostringstream stream;
  stream << " ! videoscale add-borders=true n-threads=2"
         << " ! video/x-raw,format=" << decoded_format
         << ",width=" << geometry.pre_rotation_width
         << ",height=" << geometry.pre_rotation_height;
  if (config.rotation != 0) {
    stream << " ! videoflip method=" << rot_method(config.rotation);
  }
  stream << " ! videoconvert n-threads=3"
         << " ! video/x-raw,format=RGB,width=" << config.output_width
         << ",height=" << config.output_height
         << " ! appsink name=" << sink_name
         << " emit-signals=true max-buffers=1 drop=false sync=false enable-last-sample=false";
  return stream.str();
}

}  // namespace

bool valid_rotation(int rotation) noexcept
{
  return rotation == 0 || rotation == 90 || rotation == 180 || rotation == 270;
}

std::string rot_method(int rotation)
{
  switch (rotation) {
    case 0:
      return "identity";
    case 90:
      return "clockwise";
    case 180:
      return "rotate-180";
    case 270:
      return "counterclockwise";
    default:
      throw std::invalid_argument("Rotation must be one of 0, 90, 180, or 270 degrees");
  }
}

GeomPlan make_geom_plan(const CamStreamConfig & config, bool even_geometry)
{
  if (!valid_rotation(config.rotation) || config.capture_width == 0U ||
    config.capture_height == 0U || config.output_width == 0U || config.output_height == 0U)
  {
    throw std::invalid_argument("Camera geometry has invalid dimensions or rotation");
  }

  GeomPlan plan;
  plan.rotation = config.rotation;
  plan.pre_rotation_width =
    (config.rotation == 90 || config.rotation == 270) ? config.output_height : config.output_width;
  plan.pre_rotation_height =
    (config.rotation == 90 || config.rotation == 270) ? config.output_width : config.output_height;
  const uint32_t rotated_width =
    (config.rotation == 90 || config.rotation == 270) ? config.capture_height : config.capture_width;
  const uint32_t rotated_height =
    (config.rotation == 90 || config.rotation == 270) ? config.capture_width : config.capture_height;
  plan.scale = std::min(
    static_cast<double>(config.output_width) / rotated_width,
    static_cast<double>(config.output_height) / rotated_height);
  plan.active_width = std::min(
    config.output_width, static_cast<uint32_t>(std::lround(rotated_width * plan.scale)));
  plan.active_height = std::min(
    config.output_height, static_cast<uint32_t>(std::lround(rotated_height * plan.scale)));
  if (even_geometry) {
    plan.active_width = std::max(2U, plan.active_width & ~1U);
    plan.active_height = std::max(2U, plan.active_height & ~1U);
  }
  plan.pad_x = (config.output_width - plan.active_width) / 2U;
  plan.pad_y = (config.output_height - plan.active_height) / 2U;
  const bool quarter_turn = config.rotation == 90 || config.rotation == 270;
  plan.pre_rotation_pad_x = quarter_turn ? plan.pad_y : plan.pad_x;
  plan.pre_rotation_pad_y = quarter_turn ? plan.pad_x : plan.pad_y;
  return plan;
}

std::string build_hailo_pipe(
  const CamStreamConfig & left,
  const GeomPlan & geometry,
  const HailoPipeConfig & hailo,
  bool use_test_sources)
{
  std::ostringstream stream;
  if (use_test_sources) {
    stream << test_source("stereo_left_src", left.capture_width, left.capture_height, left.fps, 18)
           << " ! videoconvert ! video/x-raw,format=NV12";
  } else {
    stream << csi_source(left, "stereo_left_src");
  }

  stream << " ! videoscale add-borders=true n-threads=2"
         << " ! video/x-raw,format=NV12,width=" << geometry.pre_rotation_width
         << ",height=" << geometry.pre_rotation_height
         << ",pixel-aspect-ratio=1/1";
  if (left.rotation != 0) {
    stream << " ! videoflip method=" << rot_method(left.rotation);
  }

  stream << " ! videoconvert n-threads=3 ! video/x-raw,format=RGB"
         << " ! " << hailo_queue("board_queue")
         << " ! hailonet name=chessboard_net hef-path=" << quoted(hailo.board_hef_path)
         << " vdevice-group-id=" << quoted(hailo.vdevice_group_id)
         << " force-writable=true is-active=true"
         << " ! " << hailo_queue("filter_board_queue")
         << " ! hailofilter so-path=liblekiwi_chessboard_postprocess.so"
         << " function-name=filter_chessboard qos=false"
         << " ! " << hailo_queue("chess_queue")
         << " ! hailonet name=pieces_net hef-path=" << quoted(hailo.pcs_hef_path)
         << " vdevice-group-id=" << quoted(hailo.vdevice_group_id)
         << " force-writable=true is-active=true"
         << " ! " << hailo_queue("filter_chess_queue")
         << " ! hailofilter so-path=liblekiwi_pieces_postprocess.so"
         << " function-name=filter_letterbox"
         << " qos=false"
         << " ! " << hailo_queue("hailo_queue")
         << " ! appsink name=hailo_appsink"
         << " emit-signals=true max-buffers=1 drop=true sync=false enable-last-sample=false";
  return stream.str();
}

std::string build_lerobot_pipe(
  const CamStreamConfig & right, const GeomPlan & right_geometry,
  const CamStreamConfig & wrist, const GeomPlan & wrist_geometry,
  const CamStreamConfig & side, const GeomPlan & side_geometry,
  bool use_test_sources)
{
  std::ostringstream stream;
  if (use_test_sources) {
    stream << test_source(
      "stereo_right_src", right.capture_width, right.capture_height, right.fps, 0)
           << " ! videoconvert ! video/x-raw,format=NV12";
  } else {
    stream << csi_source(right, "stereo_right_src");
  }
  stream << " ! " << latest_frame_queue("stereo_right_queue")
         << lerobot_transform(right, right_geometry, "stereo_right_sink", "NV12") << ' ';

  stream << usb_source(wrist, "usb_wrist_src", use_test_sources, 1)
         << lerobot_transform(wrist, wrist_geometry, "usb_wrist_sink", "I420") << ' ';
  stream << usb_source(side, "usb_side_src", use_test_sources, 13)
         << lerobot_transform(side, side_geometry, "usb_side_sink", "I420");
  return stream.str();
}

}  // namespace lekiwi_perception

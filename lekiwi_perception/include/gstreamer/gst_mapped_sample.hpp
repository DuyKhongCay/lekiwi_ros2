// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__GSTREAMER__GST_MAPPED_SAMPLE_HPP_
#define LEKIWI_PERCEPTION__GSTREAMER__GST_MAPPED_SAMPLE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <cstdint>

namespace lekiwi_perception
{

/// RAII helper to map a GstSample buffer for read access and release it on destruction.
class GstMappedSample
{
public:
  explicit GstMappedSample(GstSample * sample)
  : sample_(sample)
  {
    buffer_ = sample_ != nullptr ? gst_sample_get_buffer(sample_) : nullptr;
    mapped_ = buffer_ != nullptr && gst_buffer_map(buffer_, &map_, GST_MAP_READ);
  }

  ~GstMappedSample()
  {
    if (mapped_) {
      gst_buffer_unmap(buffer_, &map_);
    }
    if (sample_ != nullptr) {
      gst_sample_unref(sample_);
    }
  }

  GstMappedSample(const GstMappedSample &) = delete;
  GstMappedSample & operator=(const GstMappedSample &) = delete;
  [[nodiscard]] bool valid() const noexcept {return mapped_;}
  [[nodiscard]] GstBuffer * buffer() const noexcept {return buffer_;}
  [[nodiscard]] const uint8_t * data() const noexcept {return map_.data;}
  [[nodiscard]] std::size_t size() const noexcept {return map_.size;}

private:
  GstSample * sample_{nullptr};
  GstBuffer * buffer_{nullptr};
  GstMapInfo map_{};
  bool mapped_{false};
};

/// Converts an RGB GStreamer buffer into an OpenCV BGR matrix for debug visualization.
inline bool make_debug_bgr(
  const GstVideoInfo & info,
  const uint8_t * data,
  std::size_t size,
  cv::Mat & output)
{
  const auto width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&info));
  const auto height = static_cast<uint32_t>(GST_VIDEO_INFO_HEIGHT(&info));
  if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
    return false;
  }
  const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
  const auto offset = static_cast<std::size_t>(GST_VIDEO_INFO_PLANE_OFFSET(&info, 0));
  if (stride < static_cast<int>(width * 3U) ||
    offset + static_cast<std::size_t>(stride) * height > size)
  {
    return false;
  }
  const cv::Mat rgb(
    static_cast<int>(height), static_cast<int>(width), CV_8UC3,
    const_cast<uint8_t *>(data + offset), static_cast<std::size_t>(stride));
  cv::cvtColor(rgb, output, cv::COLOR_RGB2BGR);
  return true;
}

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__GSTREAMER__GST_MAPPED_SAMPLE_HPP_

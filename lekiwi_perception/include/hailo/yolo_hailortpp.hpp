/**
 * @file yolo_hailortpp.hpp
 * @brief HailoRT NMS post-processing decoder for YOLO piece detection models.
 *
 * Decodes Hailo NPU output NMS tensors into structured `HailoDetection` objects.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <vector>

#if defined(__has_include) && __has_include(<sys/cdefs.h>)
#include <sys/cdefs.h>
#endif

#ifndef __BEGIN_DECLS
#ifdef __cplusplus
#define __BEGIN_DECLS \
  extern "C"          \
  {
#define __END_DECLS }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif
#endif

#include "hailo_objects.hpp"
#include "hailo_common.hpp"
#include "hailo/hailort.h"
#include "hailo_tensors.hpp"
#include "common/nms.hpp"
#include "common/structures.hpp"

/**
 * @brief Decoder class for parsing HailoRT hardware NMS tensor payloads.
 */
class YoloNmsPostprocess
{
public:
  /**
   * @brief Decodes an NMS tensor into a vector of HailoDetection objects.
   * @param[in] tensor Target NMS tensor pointer.
   * @return std::vector<HailoDetection> Extracted bounding boxes and labels.
   */
  [[nodiscard]] std::vector<HailoDetection> decode(const HailoTensorPtr &tensor) const
  {
    if (!tensor)
    {
      return {};
    }
    if (!tensor->format().is_nms)
    {
      throw std::invalid_argument(
          "Output tensor '" + tensor->name() + "' is not an NMS tensor (is_nms=false)");
    }

    std::vector<HailoDetection> detections;
    detections.reserve(kMaxBoxes);
    const auto nms_shape = tensor->nms_shape();
    std::size_t buffer_offset = 0U;
    auto *buffer = tensor->data();

    for (uint32_t class_id = 0U; class_id < nms_shape.number_of_classes; ++class_id)
    {
      float32_t bbox_count = 0.0F;
      std::memcpy(&bbox_count, buffer + buffer_offset, sizeof(bbox_count));
      buffer_offset += sizeof(bbox_count);

      if (bbox_count == 0.0F)
      {
        continue;
      }
      if (bbox_count > nms_shape.max_bboxes_per_class)
      {
        throw std::runtime_error("NMS buffer contains too many boxes for one class");
      }

      for (uint32_t bbox_index = 0U;
           bbox_index < static_cast<uint32_t>(bbox_count); ++bbox_index)
      {
        const auto *bbox = reinterpret_cast<const common::hailo_bbox_float32_t *>(
            buffer + buffer_offset);
        add_detection(*bbox, class_id + 1U, detections);
        buffer_offset += sizeof(common::hailo_bbox_float32_t);
      }
    }
    return detections;
  }

private:
  static constexpr float kDetectionThreshold = 0.3F;
  static constexpr std::size_t kMaxBoxes = 100U;
  inline static constexpr std::array<const char *, 12U> kLabels = {
      "B", "K", "N", "P", "Q", "R", "b", "k", "n", "p", "q", "r"};

  static void add_detection(
      const common::hailo_bbox_float32_t &bbox, uint32_t class_id,
      std::vector<HailoDetection> &detections)
  {
    if (bbox.score <= kDetectionThreshold)
    {
      return;
    }

    const float confidence = std::clamp(bbox.score, 0.0F, 1.0F);
    const float width = bbox.x_max - bbox.x_min;
    const float height = bbox.y_max - bbox.y_min;
    const std::string label = class_id <= kLabels.size() ? kLabels[class_id - 1U] : std::to_string(class_id);
    detections.emplace_back(
        HailoBBox(bbox.x_min, bbox.y_min, width, height), class_id, label, confidence);
  }
};

__BEGIN_DECLS

YoloNmsPostprocess *init(const std::string config_path, const std::string function_name);
void free_resources(void *params_void_ptr);
void filter(HailoROIPtr roi, void *params_void_ptr);
void filter_letterbox(HailoROIPtr roi, void *params_void_ptr);

__END_DECLS

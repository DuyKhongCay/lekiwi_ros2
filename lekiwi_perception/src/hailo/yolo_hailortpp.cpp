// Copyright 2026 LeKiwi Labs. All rights reserved.
#include "hailo/yolo_hailortpp.hpp"

extern "C" {

YoloNmsPostprocess * init(const std::string config_path, const std::string function_name)
{
  (void)config_path;
  (void)function_name;
  return new YoloNmsPostprocess();
}

void free_resources(void * params_void_ptr)
{
  delete reinterpret_cast<YoloNmsPostprocess *>(params_void_ptr);
}

void filter(HailoROIPtr roi, void * params_void_ptr)
{
  if (!roi || !roi->has_tensors()) {
    return;
  }

  const auto * postprocess = reinterpret_cast<YoloNmsPostprocess *>(params_void_ptr);
  if (!postprocess) {
    return;
  }

  for (const auto & tensor : roi->get_tensors()) {
    if (tensor->name().find("nms") != std::string::npos) {
      hailo_common::add_detections(roi, postprocess->decode(tensor));
    }
  }
}

void filter_letterbox(HailoROIPtr roi, void * params_void_ptr) {
  filter(roi, params_void_ptr);

  HailoBBox roi_bbox = hailo_common::create_flattened_bbox(roi->get_bbox(), roi->get_scaling_bbox());
  auto detections = hailo_common::get_hailo_detections(roi);
  for (auto & detection : detections) {
    auto det_bbox = detection->get_bbox();
    auto xmin = (det_bbox.xmin() * roi_bbox.width()) + roi_bbox.xmin();
    auto ymin = (det_bbox.ymin() * roi_bbox.height()) + roi_bbox.ymin();
    auto xmax = (det_bbox.xmax() * roi_bbox.width()) + roi_bbox.xmin();
    auto ymax = (det_bbox.ymax() * roi_bbox.height()) + roi_bbox.ymin();

    HailoBBox new_bbox(xmin, ymin, xmax - xmin, ymax - ymin);
    detection->set_bbox(new_bbox);
  }
  roi->clear_scaling_bbox();
}

}  // extern "C"


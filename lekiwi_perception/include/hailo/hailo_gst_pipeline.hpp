// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__HAILO__HAILO_GST_PIPELINE_HPP_
#define LEKIWI_PERCEPTION__HAILO__HAILO_GST_PIPELINE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "sensor_msgs/msg/image.hpp"

namespace lekiwi_perception
{

  struct HailoPipelineConfig
  {
    std::string board_hef_path;
    std::string pcs_hef_path;
    std::string vdevice_group_id{"lekiwi_chess"};
    uint32_t model_width{640};
    uint32_t model_height{640};
  };

  class HailoGstPipeline
  {
  public:
    using SampleCallback = std::function<void(GstSample *sample, GstElement *pipeline)>;

    explicit HailoGstPipeline(SampleCallback callback);
    ~HailoGstPipeline();

    HailoGstPipeline(const HailoGstPipeline &) = delete;
    HailoGstPipeline &operator=(const HailoGstPipeline &) = delete;

    [[nodiscard]] bool start(
        const HailoPipelineConfig &config,
        std::chrono::milliseconds timeout,
        std::string &error);

    [[nodiscard]] bool stop(
        std::chrono::milliseconds timeout,
        std::string &error);

    [[nodiscard]] bool push_image(
        const sensor_msgs::msg::Image &image,
        std::string &error);

    [[nodiscard]] bool poll_error(std::string &error);

    [[nodiscard]] bool is_running() const noexcept;

  private:
    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data);
    void drain_bus_messages(std::string &diags, bool &has_error);
    void release_pipeline();

    SampleCallback sample_callback_;
    GstElement *pipeline_{nullptr};
    GstBus *bus_{nullptr};
    GstAppSrc *appsrc_{nullptr};
    GstElement *appsink_{nullptr};
    gulong sample_signal_id_{0};

    std::mutex push_mutex_;
    std::mutex state_mutex_;
  };

} // namespace lekiwi_perception

#endif // LEKIWI_PERCEPTION__HAILO__HAILO_GST_PIPELINE_HPP_

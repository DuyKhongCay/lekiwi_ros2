/**
 * @file hailo_gst_pipeline.hpp
 * @brief C++ GStreamer wrapper for Hailo-8 NPU inference pipelines.
 *
 * Encapsulates GStreamer pipeline construction (appsrc -> hailotools -> appsink), image buffer feeding,
 * and asynchronous sample callback dispatch.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

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

  /**
   * @brief Thread-safe C++ wrapper managing GStreamer Hailo pipeline lifecycle.
   */
  class HailoGstPipeline
  {
  public:
    using SampleCallback = std::function<void(GstSample *sample, GstElement *pipeline)>;

    /**
     * @brief Constructs HailoGstPipeline with sample callback handler.
     * @param[in] callback Function invoked whenever a new inferenced frame sample arrives at appsink.
     */
    explicit HailoGstPipeline(SampleCallback callback);
    ~HailoGstPipeline();

    HailoGstPipeline(const HailoGstPipeline &) = delete;
    HailoGstPipeline &operator=(const HailoGstPipeline &) = delete;

    /**
     * @brief Builds GStreamer string description, links elements, and sets state to PLAYING.
     * @param[in] board_hef_path File path to board segmentation HEF model file.
     * @param[in] pcs_hef_path File path to piece detection HEF model file.
     * @param[in] vdevice_group_id Hailo Virtual Device group identifier.
     * @param[in] timeout Maximum duration to wait for GStreamer state transition.
     * @param[out] error Output string capturing error details on failure.
     * @return true On successful start, false on pipeline creation or state error.
     */
    [[nodiscard]] bool start(
        const std::string &board_hef_path,
        const std::string &pcs_hef_path,
        const std::string &vdevice_group_id,
        std::chrono::milliseconds timeout,
        std::string &error);

    /**
     * @brief Stops GStreamer pipeline and sets state to NULL.
     */
    [[nodiscard]] bool stop(
        std::chrono::milliseconds timeout,
        std::string &error);

    /**
     * @brief Pushes a ROS 2 Image frame into the GStreamer appsrc element.
     * @param[in] image Input ROS Image message.
     * @param[out] error Output string on buffer feed failure.
     * @return true If buffer was pushed into appsrc successfully, false otherwise.
     */
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

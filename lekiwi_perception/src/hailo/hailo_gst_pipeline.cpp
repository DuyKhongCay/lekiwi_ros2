/**
 * @file hailo_gst_pipeline.cpp
 * @brief Implementation of HailoGstPipeline GStreamer wrapper.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "hailo/hailo_gst_pipeline.hpp"

#include <cstring>
#include <sstream>
#include <utility>

#include "rclcpp/time.hpp"

namespace lekiwi_perception
{

  HailoGstPipeline::HailoGstPipeline(SampleCallback callback)
      : sample_callback_(std::move(callback))
  {
  }

  HailoGstPipeline::~HailoGstPipeline()
  {
    std::string ignored;
    static_cast<void>(stop(std::chrono::milliseconds(1000), ignored));
  }

  bool HailoGstPipeline::start(
      const std::string &board_hef_path,
      const std::string &pcs_hef_path,
      const std::string &vdevice_group_id,
      std::chrono::milliseconds timeout,
      std::string &error)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (pipeline_ != nullptr)
    {
      error = "Hailo pipeline is already running";
      return false;
    }

    std::ostringstream ss;
    ss << "appsrc name=hailo_appsrc is-live=true format=time do-timestamp=false block=false max-bytes=0 "
       << "caps=\"video/x-raw,format=RGB,width=640,height=640\" ! "
       << "queue name=board_queue leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! "
       << "hailonet name=chessboard_net hef-path=\"" << board_hef_path << "\" "
       << "vdevice-group-id=\"" << vdevice_group_id << "\" force-writable=true is-active=true ! "
       << "queue name=filter_board_queue leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! "
       << "hailofilter so-path=liblekiwi_chessboard_postprocess.so function-name=filter_chessboard qos=false ! "
       << "queue name=chess_queue leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! "
       << "hailonet name=pieces_net hef-path=\"" << pcs_hef_path << "\" "
       << "vdevice-group-id=\"" << vdevice_group_id << "\" force-writable=true is-active=true ! "
       << "queue name=filter_chess_queue leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! "
       << "hailofilter so-path=liblekiwi_pieces_postprocess.so function-name=filter_letterbox qos=false ! "
       << "queue name=hailo_queue leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! "
       << "appsink name=hailo_appsink emit-signals=true max-buffers=1 drop=true sync=false async=false enable-last-sample=false";

    const std::string description = ss.str();
    GError *parse_error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &parse_error);
    if (parse_error != nullptr || pipeline_ == nullptr)
    {
      error = "Pipeline parse failed: ";
      error += (parse_error != nullptr ? parse_error->message : "unknown GStreamer error");
      if (parse_error != nullptr)
      {
        g_error_free(parse_error);
      }
      release_pipeline();
      return false;
    }

    bus_ = gst_element_get_bus(pipeline_);

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "hailo_appsink");
    if (appsink_ == nullptr || !GST_IS_APP_SINK(appsink_))
    {
      error = "Pipeline is missing hailo_appsink element";
      release_pipeline();
      return false;
    }

    sample_signal_id_ = g_signal_connect(
        appsink_, "new-sample", G_CALLBACK(HailoGstPipeline::on_new_sample), this);

    GstElement *appsrc_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "hailo_appsrc");
    if (appsrc_elem == nullptr || !GST_IS_APP_SRC(appsrc_elem))
    {
      error = "Pipeline is missing hailo_appsrc element";
      if (appsrc_elem != nullptr)
      {
        gst_object_unref(appsrc_elem);
      }
      release_pipeline();
      return false;
    }
    appsrc_ = GST_APP_SRC(appsrc_elem);

    const auto state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    GstState current = GST_STATE_NULL;
    GstState pending = GST_STATE_VOID_PENDING;
    const auto wait_result = (state_result == GST_STATE_CHANGE_FAILURE)
                                 ? GST_STATE_CHANGE_FAILURE
                                 : gst_element_get_state(
                                       pipeline_, &current, &pending,
                                       static_cast<GstClockTime>(timeout.count()) * GST_MSECOND);

    if (wait_result == GST_STATE_CHANGE_FAILURE ||
        (wait_result == GST_STATE_CHANGE_ASYNC && current != GST_STATE_PLAYING))
    {
      bool has_err = false;
      std::string diags;
      drain_bus_messages(diags, has_err);
      error = (wait_result == GST_STATE_CHANGE_FAILURE)
                  ? "Pipeline failed while entering PLAYING"
                  : "Pipeline timed out while entering PLAYING";
      if (!diags.empty())
      {
        error += ": " + diags;
      }
      release_pipeline();
      return false;
    }

    return true;
  }

  bool HailoGstPipeline::stop(
      std::chrono::milliseconds timeout,
      std::string &error)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (pipeline_ == nullptr)
    {
      return true;
    }

    if (appsink_ != nullptr && sample_signal_id_ != 0U)
    {
      g_signal_handler_disconnect(appsink_, sample_signal_id_);
      sample_signal_id_ = 0U;
    }

    const auto state_result = gst_element_set_state(pipeline_, GST_STATE_NULL);
    GstState current = GST_STATE_VOID_PENDING;
    GstState pending = GST_STATE_VOID_PENDING;
    const auto wait_result = (state_result == GST_STATE_CHANGE_FAILURE)
                                 ? GST_STATE_CHANGE_FAILURE
                                 : gst_element_get_state(
                                       pipeline_, &current, &pending,
                                       static_cast<GstClockTime>(timeout.count()) * GST_MSECOND);

    if (wait_result == GST_STATE_CHANGE_FAILURE || current != GST_STATE_NULL)
    {
      error = "Pipeline did not reach NULL state (current=" +
              std::string(gst_element_state_get_name(current)) + ")";
      release_pipeline();
      return false;
    }

    release_pipeline();
    return true;
  }

  bool HailoGstPipeline::push_image(
      const sensor_msgs::msg::Image &image,
      std::string &error)
  {
    if (image.encoding != "rgb8" || image.step < image.width * 3U ||
        image.data.size() < static_cast<std::size_t>(image.step) * image.height)
    {
      error = "Image format must be tightly-packed rgb8";
      return false;
    }

    std::lock_guard<std::mutex> lock(push_mutex_);
    if (appsrc_ == nullptr)
    {
      error = "appsrc is not available (pipeline stopped)";
      return false;
    }

    const auto row_bytes = static_cast<std::size_t>(image.width) * 3U;
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, row_bytes * image.height, nullptr);
    if (buffer == nullptr)
    {
      error = "Could not allocate GstBuffer";
      return false;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
    {
      gst_buffer_unref(buffer);
      error = "Could not map GstBuffer";
      return false;
    }

    for (uint32_t row = 0; row < image.height; ++row)
    {
      std::memcpy(
          map.data + row * row_bytes,
          image.data.data() + static_cast<std::size_t>(row) * image.step,
          row_bytes);
    }
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer) = rclcpp::Time(image.header.stamp).nanoseconds();
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);

    const auto flow = gst_app_src_push_buffer(appsrc_, buffer);
    if (flow != GST_FLOW_OK)
    {
      error = "appsrc push failed with flow return " + std::to_string(flow);
      return false;
    }

    return true;
  }

  bool HailoGstPipeline::poll_error(std::string &error)
  {
    bool has_error = false;
    drain_bus_messages(error, has_error);
    return has_error;
  }

  bool HailoGstPipeline::is_running() const noexcept
  {
    return pipeline_ != nullptr;
  }

  GstFlowReturn HailoGstPipeline::on_new_sample(GstAppSink *sink, gpointer user_data)
  {
    auto *self = static_cast<HailoGstPipeline *>(user_data);
    if (self == nullptr)
    {
      return GST_FLOW_ERROR;
    }

    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr)
    {
      return GST_FLOW_OK;
    }

    if (self->sample_callback_)
    {
      self->sample_callback_(sample, self->pipeline_);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  void HailoGstPipeline::drain_bus_messages(std::string &diags, bool &has_error)
  {
    if (bus_ == nullptr)
    {
      return;
    }
    std::ostringstream ss;
    GstMessage *msg = nullptr;
    while ((msg = gst_bus_pop_filtered(
                bus_, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING))) != nullptr)
    {
      GError *err = nullptr;
      gchar *dbg = nullptr;
      const bool is_err = (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR);
      if (is_err)
      {
        gst_message_parse_error(msg, &err, &dbg);
        has_error = true;
      }
      else
      {
        gst_message_parse_warning(msg, &err, &dbg);
      }

      if (ss.tellp() > 0)
      {
        ss << " | ";
      }
      ss << (is_err ? "error" : "warning") << " from " << GST_OBJECT_NAME(msg->src)
         << ": " << (err != nullptr ? err->message : "unknown");
      if (dbg != nullptr && dbg[0] != '\0')
      {
        ss << " (" << dbg << ")";
      }

      if (err != nullptr)
      {
        g_error_free(err);
      }
      g_free(dbg);
      gst_message_unref(msg);
    }
    diags = ss.str();
  }

  void HailoGstPipeline::release_pipeline()
  {
    if (appsrc_ != nullptr)
    {
      gst_object_unref(appsrc_);
      appsrc_ = nullptr;
    }
    if (appsink_ != nullptr)
    {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    if (bus_ != nullptr)
    {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (pipeline_ != nullptr)
    {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

} // namespace lekiwi_perception

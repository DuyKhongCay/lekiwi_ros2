/**
 * @file camera_streamer_component.cpp
 * @brief Implementation of CameraStreamerComponent GStreamer lifecycle node.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "camera_streamer_component.hpp"

#include <gst/video/video.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cstring>
#include <sstream>
#include <utility>

namespace lekiwi_perception
{

  CameraStreamerComponent::CameraStreamerComponent(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("camera_streamer", options)
  {
    gst_init(nullptr, nullptr);

    declare_parameter<std::string>("gscam_config", "");
    declare_parameter<std::string>("camera_name", "camera");
    declare_parameter<std::string>("frame_id", "camera");
    declare_parameter<std::string>("camera_info_url", "");
    declare_parameter<std::string>("image_encoding", "rgb8");
    declare_parameter<bool>("sync_sink", false);
    declare_parameter<bool>("use_gst_timestamps", false);
    declare_parameter<bool>("use_sensor_data_qos", true);
    declare_parameter<bool>("autostart", true);
    declare_parameter<bool>("calib_mode", false);
    declare_parameter<std::vector<int64_t>>("active_modes", std::vector<int64_t>{});
    declare_parameter<std::string>("valve_name", "gate");
    declare_parameter<int64_t>("output_size", 0);
    declare_parameter<bool>("add_border", false);

    autostart_ = get_parameter("autostart").as_bool();
    if (autostart_)
    {
      lifecycle_helper_ = std::make_unique<utils::PerceptionLifecycleHelper>(
          this, camera_name_, camera_name_ + "_stream_status");
      lifecycle_helper_->setup_autostart(true);
    }
  }

  CameraStreamerComponent::~CameraStreamerComponent()
  {
    reset_pipeline();
  }

  CameraStreamerComponent::CallbackReturn CameraStreamerComponent::on_configure(
      const rclcpp_lifecycle::State & /*state*/)
  {
    try
    {
      gscam_config_ = get_parameter("gscam_config").as_string();
      camera_name_ = get_parameter("camera_name").as_string();
      frame_id_ = get_parameter("frame_id").as_string();
      camera_info_url_ = get_parameter("camera_info_url").as_string();
      image_encoding_ = get_parameter("image_encoding").as_string();
      sync_sink_ = get_parameter("sync_sink").as_bool();
      use_gst_timestamps_ = get_parameter("use_gst_timestamps").as_bool();
      use_sensor_data_qos_ = get_parameter("use_sensor_data_qos").as_bool();
      calib_mode_ = get_parameter("calib_mode").as_bool();
      active_modes_ = get_parameter("active_modes").as_integer_array();
      valve_name_ = get_parameter("valve_name").as_string();
      output_size_ = get_parameter("output_size").as_int();
      add_border_ = get_parameter("add_border").as_bool();

      if (gscam_config_.empty())
      {
        RCLCPP_ERROR(get_logger(), "Parameter 'gscam_config' is required but was empty");
        return CallbackReturn::FAILURE;
      }

      camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
          this, camera_name_, camera_info_url_);
      if (!camera_info_url_.empty())
      {
        camera_info_manager_->loadCameraInfo(camera_info_url_);
      }

      const bool only_mode_3 = !active_modes_.empty() &&
                               std::all_of(active_modes_.begin(), active_modes_.end(), [](int64_t m)
                                           { return m == 3; });
      if (only_mode_3)
      {
        publish_raw_ = false;
        publish_compressed_ = true;
      }
      else
      {
        publish_raw_ = true;
        publish_compressed_ = false;
      }

      rclcpp::QoS qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS(1);
      if (publish_raw_)
      {
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);
      }
      if (publish_compressed_)
      {
        compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_raw/compressed", qos);
      }
      info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos);

      if (!lifecycle_helper_)
      {
        lifecycle_helper_ = std::make_unique<utils::PerceptionLifecycleHelper>(
            this, camera_name_, camera_name_ + "_stream_status");
      }

      std::vector<uint8_t> active_u8;
      active_u8.reserve(active_modes_.size());
      for (int64_t m : active_modes_)
      {
        active_u8.push_back(static_cast<uint8_t>(m));
      }

      lifecycle_helper_->setup_camera_mode_sub(
          active_u8,
          [this](uint8_t /*new_mode*/)
          {
            this->update_valve_state();
          });

      std::string full_pipeline = gscam_config_;
      if (full_pipeline.find("appsink") == std::string::npos)
      {
        std::ostringstream ss;
        ss << " ! appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync="
           << (sync_sink_ ? "true" : "false") << " async=false enable-last-sample=false";
        full_pipeline += ss.str();
      }

      GError *error = nullptr;
      pipeline_ = gst_parse_launch(full_pipeline.c_str(), &error);
      if (error != nullptr || pipeline_ == nullptr)
      {
        RCLCPP_ERROR(
            get_logger(), "GStreamer parse error for camera '%s': %s",
            camera_name_.c_str(), error ? error->message : "unknown error");
        if (error != nullptr)
        {
          g_error_free(error);
        }
        reset_pipeline();
        return CallbackReturn::FAILURE;
      }

      bus_ = gst_element_get_bus(pipeline_);

      appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_sink");
      if (appsink_ == nullptr)
      {
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "gscam_appsink");
      }
      if (appsink_ == nullptr)
      {
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
      }
      if (appsink_ == nullptr || !GST_IS_APP_SINK(appsink_))
      {
        RCLCPP_ERROR(get_logger(), "Could not locate valid appsink element in pipeline");
        reset_pipeline();
        return CallbackReturn::FAILURE;
      }

      sample_signal_id_ = g_signal_connect(
          appsink_, "new-sample", G_CALLBACK(CameraStreamerComponent::on_new_sample), this);

      valve_ = gst_bin_get_by_name(GST_BIN(pipeline_), valve_name_.c_str());
      if (valve_ == nullptr)
      {
        valve_ = gst_bin_get_by_name(GST_BIN(pipeline_), "gate");
      }
      if (valve_ == nullptr)
      {
        valve_ = gst_bin_get_by_name(GST_BIN(pipeline_), "valve");
      }

      if (valve_ != nullptr)
      {
        RCLCPP_INFO(
            get_logger(), "Found GStreamer valve element '%s' for camera '%s' (calib_mode=%s)",
            GST_OBJECT_NAME(valve_), camera_name_.c_str(), calib_mode_ ? "true" : "false");
        g_object_set(G_OBJECT(valve_), "drop", calib_mode_ ? FALSE : TRUE, NULL);
      }
      else
      {
        RCLCPP_WARN(
            get_logger(), "No valve element found for camera '%s'; using software fallback gating",
            camera_name_.c_str());
      }

      lifecycle_helper_->diagnostics().updater().add(
          camera_name_ + "_stream_status", this,
          &CameraStreamerComponent::produce_diagnostics);

      RCLCPP_INFO(
          get_logger(), "CameraStreamerComponent configured successfully for camera '%s'",
          camera_name_.c_str());
      return CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception during on_configure: %s", e.what());
      reset_pipeline();
      return CallbackReturn::FAILURE;
    }
    catch (...)
    {
      RCLCPP_ERROR(get_logger(), "Unknown exception during on_configure");
      reset_pipeline();
      return CallbackReturn::FAILURE;
    }
  }

  CameraStreamerComponent::CallbackReturn CameraStreamerComponent::on_activate(
      const rclcpp_lifecycle::State & /*state*/)
  {
    try
    {
      if (image_pub_)
      {
        image_pub_->on_activate();
      }
      if (compressed_image_pub_)
      {
        compressed_image_pub_->on_activate();
      }
      if (info_pub_)
      {
        info_pub_->on_activate();
      }

      if (pipeline_ != nullptr)
      {
        const auto state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        GstState current = GST_STATE_NULL;
        GstState pending = GST_STATE_VOID_PENDING;
        const auto wait_result = (state_result == GST_STATE_CHANGE_FAILURE)
                                     ? GST_STATE_CHANGE_FAILURE
                                     : gst_element_get_state(
                                           pipeline_, &current, &pending,
                                           static_cast<GstClockTime>(5000) * GST_MSECOND);

        if (wait_result == GST_STATE_CHANGE_FAILURE ||
            (wait_result == GST_STATE_CHANGE_ASYNC && current != GST_STATE_PLAYING))
        {
          poll_bus_errors();
          RCLCPP_ERROR(
              get_logger(), "Failed to set GStreamer pipeline to PLAYING state (current=%s)",
              gst_element_state_get_name(current));
          return CallbackReturn::FAILURE;
        }
      }

      if (lifecycle_helper_)
      {
        lifecycle_helper_->perf_tracker().reset();
      }

      monitor_timer_ = create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&CameraStreamerComponent::monitor_tick, this));

      update_valve_state();

      RCLCPP_INFO(
          get_logger(), "CameraStreamerComponent activated for camera '%s'", camera_name_.c_str());
      return CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception during on_activate: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    catch (...)
    {
      RCLCPP_ERROR(get_logger(), "Unknown exception during on_activate");
      return CallbackReturn::FAILURE;
    }
  }

  CameraStreamerComponent::CallbackReturn CameraStreamerComponent::on_deactivate(
      const rclcpp_lifecycle::State & /*state*/)
  {
    try
    {
      if (monitor_timer_)
      {
        monitor_timer_->cancel();
        monitor_timer_.reset();
      }

      is_streaming_.store(false);
      if (valve_ != nullptr)
      {
        g_object_set(G_OBJECT(valve_), "drop", TRUE, NULL);
      }

      if (pipeline_ != nullptr)
      {
        gst_element_set_state(pipeline_, GST_STATE_PAUSED);
      }

      if (image_pub_)
      {
        image_pub_->on_deactivate();
      }
      if (compressed_image_pub_)
      {
        compressed_image_pub_->on_deactivate();
      }
      if (info_pub_)
      {
        info_pub_->on_deactivate();
      }

      RCLCPP_INFO(
          get_logger(), "CameraStreamerComponent deactivated for camera '%s'", camera_name_.c_str());
      return CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception during on_deactivate: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    catch (...)
    {
      RCLCPP_ERROR(get_logger(), "Unknown exception during on_deactivate");
      return CallbackReturn::FAILURE;
    }
  }

  CameraStreamerComponent::CallbackReturn CameraStreamerComponent::on_cleanup(
      const rclcpp_lifecycle::State & /*state*/)
  {
    reset_pipeline();
    RCLCPP_INFO(
        get_logger(), "CameraStreamerComponent cleaned up for camera '%s'", camera_name_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CameraStreamerComponent::CallbackReturn CameraStreamerComponent::on_shutdown(
      const rclcpp_lifecycle::State & /*state*/)
  {
    reset_pipeline();
    RCLCPP_INFO(
        get_logger(), "CameraStreamerComponent shut down for camera '%s'", camera_name_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CameraStreamerComponent::CallbackReturn CameraStreamerComponent::on_error(
      const rclcpp_lifecycle::State & /*state*/)
  {
    reset_pipeline();
    RCLCPP_WARN(
        get_logger(), "CameraStreamerComponent reset due to error for camera '%s'",
        camera_name_.c_str());
    return CallbackReturn::SUCCESS;
  }

  void CameraStreamerComponent::reset_pipeline()
  {
    is_streaming_.store(false);

    std::lock_guard<std::mutex> lock(gst_mutex_);

    if (monitor_timer_)
    {
      monitor_timer_->cancel();
      monitor_timer_.reset();
    }

    if (appsink_ != nullptr && sample_signal_id_ != 0U)
    {
      g_signal_handler_disconnect(appsink_, sample_signal_id_);
      sample_signal_id_ = 0U;
    }

    if (valve_ != nullptr)
    {
      gst_object_unref(valve_);
      valve_ = nullptr;
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

    if (lifecycle_helper_)
    {
      lifecycle_helper_->reset();
      lifecycle_helper_.reset();
    }

    image_pub_.reset();
    compressed_image_pub_.reset();
    info_pub_.reset();
    camera_info_manager_.reset();
  }

  void CameraStreamerComponent::monitor_tick()
  {
    poll_bus_errors();
    update_valve_state();
  }

  void CameraStreamerComponent::update_valve_state()
  {
    const uint8_t state_id = get_current_state().id();
    const bool is_active = (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
                            state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING);
    const bool mode_allowed = !lifecycle_helper_ || lifecycle_helper_->is_mode_allowed();
    const uint8_t mode = lifecycle_helper_ ? lifecycle_helper_->get_current_mode() : 0;

    const bool should_stream = calib_mode_ || (is_active && mode_allowed);
    const bool prev_streaming = is_streaming_.exchange(should_stream);

    if (valve_ != nullptr)
    {
      const gboolean drop_val = (calib_mode_ || should_stream) ? FALSE : TRUE;
      g_object_set(G_OBJECT(valve_), "drop", drop_val, NULL);
    }

    if (should_stream != prev_streaming)
    {
      RCLCPP_INFO(
          get_logger(),
          "[%s] Valve state changed: %s (calib_mode=%d, mode=%u, allowed=%d, active=%d)",
          camera_name_.c_str(),
          should_stream ? "OPEN (streaming)" : "DROPPING (idle)",
          calib_mode_ ? 1 : 0, mode, mode_allowed, is_active);
    }
  }

  void CameraStreamerComponent::poll_bus_errors()
  {
    if (bus_ == nullptr)
    {
      return;
    }

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
        std::string err_text = err ? err->message : "unknown error";
        {
          std::lock_guard<std::mutex> lock(error_mutex_);
          last_gst_error_ = err_text;
        }
        RCLCPP_ERROR(
            get_logger(), "[%s] GStreamer bus error from %s: %s%s%s",
            camera_name_.c_str(),
            GST_OBJECT_NAME(msg->src),
            err_text.c_str(),
            dbg ? " (" : "",
            dbg ? dbg : "");
        if (dbg)
        {
          RCLCPP_ERROR(get_logger(), "[%s] Debug info: %s", camera_name_.c_str(), dbg);
        }
      }
      else
      {
        gst_message_parse_warning(msg, &err, &dbg);
        std::string warn_text = err ? err->message : "unknown warning";
        {
          std::lock_guard<std::mutex> lock(error_mutex_);
          last_gst_error_ = "Warning: " + warn_text;
        }
        RCLCPP_WARN(
            get_logger(), "[%s] GStreamer bus warning from %s: %s%s%s",
            camera_name_.c_str(),
            GST_OBJECT_NAME(msg->src),
            warn_text.c_str(),
            dbg ? " (" : "",
            dbg ? dbg : "");
      }

      if (err != nullptr)
      {
        g_error_free(err);
      }
      g_free(dbg);
      gst_message_unref(msg);
    }
  }

  GstFlowReturn CameraStreamerComponent::on_new_sample(GstAppSink *sink, gpointer user_data)
  {
    auto *self = static_cast<CameraStreamerComponent *>(user_data);
    if (self == nullptr)
    {
      return GST_FLOW_ERROR;
    }

    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr)
    {
      return GST_FLOW_OK;
    }

    try
    {
      self->process_sample(sample);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(self->get_logger(), "[%s] Exception in process_sample: %s",
                   self->camera_name_.c_str(), e.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(self->get_logger(), "[%s] Unknown exception in process_sample",
                   self->camera_name_.c_str());
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  void CameraStreamerComponent::process_sample(GstSample *sample)
  {
    if (!is_streaming_.load())
    {
      return;
    }

    const bool raw_active = (image_pub_ && image_pub_->is_activated());
    const bool comp_active = (compressed_image_pub_ && compressed_image_pub_->is_activated());
    if (!raw_active && !comp_active)
    {
      return;
    }

    if (sample == nullptr)
    {
      return;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (buffer == nullptr)
    {
      return;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
      return;
    }

    GstCaps *caps = gst_sample_get_caps(sample);
    if (caps == nullptr)
    {
      gst_buffer_unmap(buffer, &map);
      return;
    }

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;

    const auto now_time = now();
    double gst_pipeline_lat_ms = 0.0;
    int64_t pipeline_lat_ns = 0;
    bool has_valid_pipeline_lat = false;

    if (pipeline_ != nullptr && (GST_BUFFER_PTS_IS_VALID(buffer) || GST_BUFFER_DTS_IS_VALID(buffer)))
    {
      GstClock *clock = gst_element_get_clock(pipeline_);
      if (clock != nullptr)
      {
        const GstClockTime now_gst = gst_clock_get_time(clock);
        const GstClockTime base_time = gst_element_get_base_time(pipeline_);
        const GstClockTime pts = GST_BUFFER_PTS_IS_VALID(buffer) ? GST_BUFFER_PTS(buffer) : GST_BUFFER_DTS(buffer);

        if (GST_CLOCK_TIME_IS_VALID(now_gst) && GST_CLOCK_TIME_IS_VALID(base_time) && GST_CLOCK_TIME_IS_VALID(pts))
        {
          const GstClockTime buffer_capture_time = base_time + pts;
          if (now_gst >= buffer_capture_time)
          {
            const GstClockTime diff_ns = now_gst - buffer_capture_time;
            pipeline_lat_ns = static_cast<int64_t>(diff_ns);
            gst_pipeline_lat_ms = static_cast<double>(diff_ns) * 1e-6;
            has_valid_pipeline_lat = true;
          }
        }
        gst_object_unref(clock);
      }
    }

    if (use_gst_timestamps_)
    {
      if (has_valid_pipeline_lat)
      {
        header.stamp = now_time - rclcpp::Duration::from_nanoseconds(pipeline_lat_ns);
      }
      else
      {
        header.stamp = now_time;
      }
    }
    else
    {
      header.stamp = now_time;
    }

    GstStructure *structure = gst_caps_get_structure(caps, 0);
    const gchar *media_type = structure ? gst_structure_get_name(structure) : "";
    const bool is_jpeg = (g_strcmp0(media_type, "image/jpeg") == 0);

    uint32_t frame_width = 0;
    uint32_t frame_height = 0;

    if (is_jpeg)
    {
      int width = 0;
      int height = 0;
      if (structure != nullptr)
      {
        gst_structure_get_int(structure, "width", &width);
        gst_structure_get_int(structure, "height", &height);
      }
      frame_width = static_cast<uint32_t>(width);
      frame_height = static_cast<uint32_t>(height);

      if (comp_active)
      {
        auto comp_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        comp_msg->header = header;
        comp_msg->format = "jpeg";
        comp_msg->data.assign(map.data, map.data + map.size);
        compressed_image_pub_->publish(std::move(comp_msg));
      }
      else if (raw_active)
      {
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header = header;
        img_msg->width = frame_width;
        img_msg->height = frame_height;
        img_msg->encoding = "jpeg";
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<uint32_t>(map.size);
        img_msg->data.assign(map.data, map.data + map.size);
        image_pub_->publish(std::move(img_msg));
      }
    }
    else
    {
      GstVideoInfo video_info;
      gst_video_info_init(&video_info);
      const bool has_video_info = gst_video_info_from_caps(&video_info, caps);

      auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
      img_msg->header = header;

      if (has_video_info)
      {
        img_msg->width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&video_info));
        img_msg->height = static_cast<uint32_t>(GST_VIDEO_INFO_HEIGHT(&video_info));
        img_msg->encoding = image_encoding_;
        img_msg->is_bigendian = false;

        uint32_t bpp = 3;
        if (image_encoding_ == "mono8" || image_encoding_ == "8UC1")
        {
          bpp = 1;
        }
        else if (image_encoding_ == "rgba8" || image_encoding_ == "bgra8")
        {
          bpp = 4;
        }
        img_msg->step = img_msg->width * bpp;
        img_msg->data.resize(static_cast<std::size_t>(img_msg->step) * img_msg->height);

        const gsize stride = GST_VIDEO_INFO_PLANE_STRIDE(&video_info, 0);
        if (stride == 0 || stride == img_msg->step)
        {
          const std::size_t copy_size = std::min(map.size, img_msg->data.size());
          std::memcpy(img_msg->data.data(), map.data, copy_size);
        }
        else
        {
          for (uint32_t r = 0; r < img_msg->height; ++r)
          {
            if ((r + 1) * stride <= map.size && (r + 1) * img_msg->step <= img_msg->data.size())
            {
              std::memcpy(
                  img_msg->data.data() + static_cast<std::size_t>(r) * img_msg->step,
                  map.data + static_cast<std::size_t>(r) * stride,
                  img_msg->step);
            }
          }
        }
      }
      else
      {
        int width = 0;
        int height = 0;
        if (structure != nullptr)
        {
          gst_structure_get_int(structure, "width", &width);
          gst_structure_get_int(structure, "height", &height);
        }
        img_msg->width = static_cast<uint32_t>(width);
        img_msg->height = static_cast<uint32_t>(height);
        img_msg->encoding = image_encoding_;
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<uint32_t>(map.size);
        img_msg->data.assign(map.data, map.data + map.size);
      }

      frame_width = img_msg->width;
      frame_height = img_msg->height;

      if (raw_active)
      {
        image_pub_->publish(std::move(img_msg));
      }
    }

    gst_buffer_unmap(buffer, &map);

    const uint32_t target_w = (output_size_ > 0) ? static_cast<uint32_t>(output_size_) : frame_width;
    const uint32_t target_h = (output_size_ > 0) ? static_cast<uint32_t>(output_size_) : frame_height;

    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    if (camera_info_manager_)
    {
      const auto raw_info = camera_info_manager_->getCameraInfo();
      *info_msg = scale_camera_info(raw_info, target_w, target_h);
    }
    else
    {
      info_msg->width = target_w;
      info_msg->height = target_h;
    }
    info_msg->header = header;

    if (!is_streaming_.load())
    {
      return;
    }

    if (info_pub_ && info_pub_->is_activated())
    {
      info_pub_->publish(std::move(info_msg));
    }

    if (lifecycle_helper_ && is_streaming_.load())
    {
      const double lat_record = (use_gst_timestamps_ || !has_valid_pipeline_lat) ? 0.0 : gst_pipeline_lat_ms;
      lifecycle_helper_->perf_tracker().record_frame(lat_record);
    }
  }

  bool CameraStreamerComponent::is_streaming() const noexcept
  {
    return is_streaming_.load();
  }

  uint8_t CameraStreamerComponent::current_camera_mode() const noexcept
  {
    return lifecycle_helper_ ? lifecycle_helper_->get_current_mode() : 0;
  }

  bool CameraStreamerComponent::is_valve_open() const
  {
    if (valve_ == nullptr)
    {
      return is_streaming_.load();
    }
    gboolean drop = TRUE;
    g_object_get(G_OBJECT(valve_), "drop", &drop, NULL);
    return (drop == FALSE);
  }

  void CameraStreamerComponent::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (!lifecycle_helper_)
    {
      return;
    }

    const bool streaming = is_streaming_.load();
    std::string err_msg;
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      err_msg = last_gst_error_;
    }

    lifecycle_helper_->update_diagnostics(
        stat, streaming, err_msg, 5.0F,
        [&](diagnostic_updater::DiagnosticStatusWrapper &s)
        {
          s.add("Camera Name", camera_name_);
          s.add("Frame ID", frame_id_);
          s.add("Stream State", streaming ? "Streaming" : (is_valve_open() ? "Valve Open (Idle)" : "Gated (Dropping)"));
          s.add("Use GST Timestamps", use_gst_timestamps_ ? "true" : "false");
          s.add("Publish Raw", publish_raw_ ? "true" : "false");
          s.add("Publish Compressed", publish_compressed_ ? "true" : "false");
        });
  }

  sensor_msgs::msg::CameraInfo CameraStreamerComponent::scale_camera_info(
      const sensor_msgs::msg::CameraInfo &orig_info,
      uint32_t target_w, uint32_t target_h) const
  {
    return utils::CameraInfoScaler::scale(orig_info, target_w, target_h, add_border_);
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::CameraStreamerComponent)

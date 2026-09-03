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

    autostart_ = get_parameter("autostart").as_bool();
    if (autostart_)
    {
      autostart_timer_ = create_wall_timer(
          std::chrono::milliseconds(1),
          [this]()
          {
            if (autostart_timer_)
            {
              autostart_timer_->cancel();
              autostart_timer_.reset();
            }
            if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
            {
              this->configure();
            }
            if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
            {
              this->activate();
            }
          });
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

      rclcpp::QoS qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS(1);
      image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);
      info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos);

      rclcpp::QoS mode_qos(1);
      mode_qos.reliable();
      mode_qos.transient_local();
      mode_sub_ = create_subscription<lekiwi_interfaces::msg::CameraMode>(
          "/camera_mode", mode_qos,
          std::bind(&CameraStreamerComponent::on_camera_mode, this, std::placeholders::_1));

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

      updater_ = std::make_shared<diagnostic_updater::Updater>(this);
      updater_->setHardwareID(camera_name_);
      updater_->add(
          camera_name_ + "_stream_status", this,
          &CameraStreamerComponent::produce_diagnostics);

      last_fps_time_ = std::chrono::steady_clock::now();
      last_fps_frame_count_ = 0;

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

      frame_counter_.store(0);
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
    std::lock_guard<std::mutex> lock(gst_mutex_);

    if (autostart_timer_)
    {
      autostart_timer_->cancel();
      autostart_timer_.reset();
    }
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

    mode_sub_.reset();
    image_pub_.reset();
    info_pub_.reset();
    camera_info_manager_.reset();
    is_streaming_.store(false);
  }

  void CameraStreamerComponent::on_camera_mode(
      const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg)
  {
    if (!msg)
    {
      return;
    }
    current_camera_mode_.store(msg->value);
    update_valve_state();
  }

  void CameraStreamerComponent::monitor_tick()
  {
    poll_bus_errors();
    update_valve_state();
  }

  void CameraStreamerComponent::update_valve_state()
  {
    const bool is_active = (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const uint8_t mode = current_camera_mode_.load();

    const bool mode_allowed = active_modes_.empty() ||
                              (std::find(active_modes_.begin(), active_modes_.end(), static_cast<int64_t>(mode)) != active_modes_.end());

    size_t sub_count = 0;
    if (image_pub_)
    {
      sub_count += image_pub_->get_subscription_count() + image_pub_->get_intra_process_subscription_count();
    }
    if (info_pub_)
    {
      sub_count += info_pub_->get_subscription_count() + info_pub_->get_intra_process_subscription_count();
    }
    const bool has_subscribers = (sub_count > 0);

    const bool should_stream = calib_mode_ || (is_active && mode_allowed && has_subscribers);
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
          "[%s] Valve state changed: %s (calib_mode=%d, mode=%u, allowed=%d, sub_count=%zu, active=%d)",
          camera_name_.c_str(),
          should_stream ? "OPEN (streaming)" : "DROPPING (idle)",
          calib_mode_ ? 1 : 0, mode, mode_allowed, sub_count, is_active);
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
        last_gst_error_ = err_text;
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
        last_gst_error_ = "Warning: " + warn_text;
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
    if (!is_streaming_.load() || !image_pub_ || !image_pub_->is_activated())
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

    GstVideoInfo video_info;
    gst_video_info_init(&video_info);
    const bool has_video_info = gst_video_info_from_caps(&video_info, caps);

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    if (use_gst_timestamps_ && GST_BUFFER_PTS_IS_VALID(buffer) && GST_BUFFER_PTS(buffer) > 0U)
    {
      header.stamp = rclcpp::Time(static_cast<int64_t>(GST_BUFFER_PTS(buffer)), RCL_SYSTEM_TIME);
    }
    else
    {
      header.stamp = now();
    }

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
      // Non-raw or JPEG caps fallback
      GstStructure *structure = gst_caps_get_structure(caps, 0);
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

    gst_buffer_unmap(buffer, &map);

    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    if (camera_info_manager_)
    {
      *info_msg = camera_info_manager_->getCameraInfo();
    }
    info_msg->header = header;
    if (info_msg->width == 0 && info_msg->height == 0)
    {
      info_msg->width = img_msg->width;
      info_msg->height = img_msg->height;
    }

    image_pub_->publish(std::move(img_msg));
    if (info_pub_ && info_pub_->is_activated())
    {
      info_pub_->publish(std::move(info_msg));
    }

    if (GST_BUFFER_PTS_IS_VALID(buffer) && GST_BUFFER_PTS(buffer) > 0U)
    {
      const auto now_ns = now().nanoseconds();
      const auto pts_ns = static_cast<int64_t>(GST_BUFFER_PTS(buffer));
      if (now_ns > pts_ns)
      {
        const double lat_ms = static_cast<double>(now_ns - pts_ns) * 1e-6;
        current_latency_ms_.store(lat_ms);
      }
    }

    frame_counter_.fetch_add(1, std::memory_order_relaxed);
  }

  bool CameraStreamerComponent::is_streaming() const noexcept
  {
    return is_streaming_.load();
  }

  uint8_t CameraStreamerComponent::current_camera_mode() const noexcept
  {
    return current_camera_mode_.load();
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
    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_sec = std::chrono::duration<float>(now_tp - last_fps_time_).count();
    const uint64_t current_count = frame_counter_.load(std::memory_order_relaxed);

    if (elapsed_sec >= 0.5F)
    {
      const uint64_t delta_frames = current_count - last_fps_frame_count_;
      current_fps_.store(static_cast<float>(delta_frames) / elapsed_sec);
      last_fps_time_ = now_tp;
      last_fps_frame_count_ = current_count;
    }

    const float fps = current_fps_.load();
    const double latency_ms = current_latency_ms_.load();
    const bool streaming = is_streaming_.load();
    const bool valve_open = is_valve_open();
    const bool is_active = (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // 1. Overall Status
    if (!last_gst_error_.empty() && last_gst_error_.rfind("Warning:", 0) != 0)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "GStreamer pipeline error: %s", last_gst_error_.c_str());
    }
    else if (!is_active)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Camera streamer node inactive");
    }
    else if (streaming && fps < 5.0F && current_count > 10)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "Low camera frame rate (%.1f FPS)", fps);
    }
    else if (streaming)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Streaming active (%.1f FPS, %.1f ms latency)", fps, latency_ms);
    }
    else
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Gated / Standby (No active stream requested)");
    }

    // 2. Metrics
    stat.add("Camera Name", camera_name_);
    stat.add("Frame ID", frame_id_);
    stat.add("Stream State", streaming ? "Streaming" : (valve_open ? "Valve Open (Idle)" : "Gated (Dropping)"));
    stat.addf("Framerate (FPS)", "%.1f", fps);
    stat.addf("Gst to ROS Latency (ms)", "%.2f", latency_ms);
    stat.add("Total Frames Published", current_count);

    size_t sub_count = 0;
    if (image_pub_)
    {
      sub_count += image_pub_->get_subscription_count() + image_pub_->get_intra_process_subscription_count();
    }
    stat.add("Image Subscribers Count", sub_count);

    if (!last_gst_error_.empty())
    {
      stat.add("Last Gst Status", last_gst_error_);
    }
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::CameraStreamerComponent)

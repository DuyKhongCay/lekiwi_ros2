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
    compressed_image_pub_.reset();
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
    const uint8_t state_id = get_current_state().id();
    const bool is_active = (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
                            state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING);
    const uint8_t mode = current_camera_mode_.load();

    const bool mode_allowed = active_modes_.empty() ||
                              (std::find(active_modes_.begin(), active_modes_.end(), static_cast<int64_t>(mode)) != active_modes_.end());

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
      // In use_gst_timestamps mode, latency is embedded into header.stamp
      current_latency_ms_.store(0.0);
    }
    else
    {
      header.stamp = now_time;
      // In decoupled mode, measure and store GStreamer pipeline latency separately
      current_latency_ms_.store(has_valid_pipeline_lat ? gst_pipeline_lat_ms : 0.0);
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
        // Fallback: publish JPEG payload into Image message data if only raw_active
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

    if (info_pub_ && info_pub_->is_activated())
    {
      info_pub_->publish(std::move(info_msg));
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
      if (use_gst_timestamps_)
      {
        stat.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "Streaming active (%.1f FPS, use_gst_timestamps=true)", fps);
      }
      else
      {
        stat.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "Streaming active (%.1f FPS, %.1f ms latency)", fps, latency_ms);
      }
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
    stat.add("Use GST Timestamps", use_gst_timestamps_ ? "true" : "false");
    if (use_gst_timestamps_)
    {
      stat.add("Gst to ROS Latency", "N/A (use_gst_timestamps=true)");
    }
    else
    {
      stat.addf("Gst to ROS Latency (ms)", "%.2f", latency_ms);
    }
    stat.add("Total Frames Published", current_count);
    stat.add("Publish Raw", publish_raw_ ? "true" : "false");
    stat.add("Publish Compressed", publish_compressed_ ? "true" : "false");

    if (!last_gst_error_.empty())
    {
      stat.add("Last Gst Status", last_gst_error_);
    }
  }

  sensor_msgs::msg::CameraInfo CameraStreamerComponent::scale_camera_info(
      const sensor_msgs::msg::CameraInfo &orig_info,
      uint32_t target_w, uint32_t target_h) const
  {
    // Rescales camera intrinsic matrix K and projection matrix P based on output resolution.
    auto scaled_info = orig_info;
    const uint32_t orig_w = orig_info.width;
    const uint32_t orig_h = orig_info.height;

    scaled_info.width = target_w;
    scaled_info.height = target_h;

    if (orig_w == 0 || orig_h == 0 || target_w == 0 || target_h == 0)
    {
      return scaled_info;
    }

    if (orig_w == target_w && orig_h == target_h)
    {
      return scaled_info;
    }

    const double orig_ar = static_cast<double>(orig_w) / static_cast<double>(orig_h);
    const double target_ar = static_cast<double>(target_w) / static_cast<double>(target_h);

    double sx = 1.0;
    double sy = 1.0;
    double offset_x = 0.0;
    double offset_y = 0.0;

    if (add_border_)
    {
      // Aspect-ratio-preserving scale with letterbox padding (e.g. videoscale add-borders=true)
      if (orig_ar >= target_ar)
      {
        const double s = static_cast<double>(target_w) / static_cast<double>(orig_w);
        sx = s;
        sy = s;
        offset_x = 0.0;
        const double active_h = static_cast<double>(orig_h) * s;
        offset_y = (static_cast<double>(target_h) - active_h) / 2.0;
      }
      else
      {
        const double s = static_cast<double>(target_h) / static_cast<double>(orig_h);
        sx = s;
        sy = s;
        const double active_w = static_cast<double>(orig_w) * s;
        offset_x = (static_cast<double>(target_w) - active_w) / 2.0;
        offset_y = 0.0;
      }

      // Scale Camera Matrix K (3x3 row-major)
      scaled_info.k[0] = orig_info.k[0] * sx;
      scaled_info.k[2] = orig_info.k[2] * sx + offset_x;
      scaled_info.k[4] = orig_info.k[4] * sy;
      scaled_info.k[5] = orig_info.k[5] * sy + offset_y;

      // Scale Projection Matrix P (3x4 row-major)
      scaled_info.p[0] = orig_info.p[0] * sx;
      scaled_info.p[2] = orig_info.p[2] * sx + offset_x;
      scaled_info.p[3] = orig_info.p[3] * sx;
      scaled_info.p[5] = orig_info.p[5] * sy;
      scaled_info.p[6] = orig_info.p[6] * sy + offset_y;
      scaled_info.p[7] = orig_info.p[7] * sy;
    }
    else
    {
      // Aspect-ratio-preserving scale with center cropping (e.g. videocrop left/right or top/bottom)
      double crop_x = 0.0;
      double crop_y = 0.0;
      double s = 1.0;

      if (orig_ar > target_ar)
      {
        const double cropped_w = static_cast<double>(orig_h) * target_ar;
        crop_x = (static_cast<double>(orig_w) - cropped_w) / 2.0;
        s = static_cast<double>(target_w) / cropped_w;
      }
      else if (orig_ar < target_ar)
      {
        const double cropped_h = static_cast<double>(orig_w) / target_ar;
        crop_y = (static_cast<double>(orig_h) - cropped_h) / 2.0;
        s = static_cast<double>(target_h) / cropped_h;
      }
      else
      {
        s = static_cast<double>(target_w) / static_cast<double>(orig_w);
      }

      sx = s;
      sy = s;

      // Scale Camera Matrix K (3x3 row-major)
      scaled_info.k[0] = orig_info.k[0] * sx;
      scaled_info.k[2] = (orig_info.k[2] - crop_x) * sx;
      scaled_info.k[4] = orig_info.k[4] * sy;
      scaled_info.k[5] = (orig_info.k[5] - crop_y) * sy;

      // Scale Projection Matrix P (3x4 row-major)
      scaled_info.p[0] = orig_info.p[0] * sx;
      scaled_info.p[2] = (orig_info.p[2] - crop_x) * sx;
      scaled_info.p[3] = orig_info.p[3] * sx;
      scaled_info.p[5] = orig_info.p[5] * sy;
      scaled_info.p[6] = (orig_info.p[6] - crop_y) * sy;
      scaled_info.p[7] = orig_info.p[7] * sy;
    }

    return scaled_info;
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::CameraStreamerComponent)

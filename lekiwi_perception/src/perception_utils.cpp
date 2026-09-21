/**
 * @file perception_utils.cpp
 * @brief Implementation of reusable utilities for lekiwi_perception.
 *
 * @author LeKiwi Engineering
 * @copyright Apache-2.0
 */

#include "perception_utils.hpp"

namespace lekiwi_perception::utils
{

  // ===========================================================================
  // FramePerformanceTracker Implementation
  // ===========================================================================

  FramePerformanceTracker::FramePerformanceTracker(float window_duration_sec)
      : window_duration_sec_(window_duration_sec),
        last_fps_time_(std::chrono::steady_clock::now())
  {
  }

  void FramePerformanceTracker::record_frame(double latency_ms) noexcept
  {
    frame_counter_.fetch_add(1, std::memory_order_relaxed);
    if (latency_ms >= 0.0)
    {
      current_latency_ms_.store(latency_ms, std::memory_order_relaxed);
    }
    tick_window();
  }

  void FramePerformanceTracker::tick_window() noexcept
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_sec = std::chrono::duration<float>(now_tp - last_fps_time_).count();
    const uint64_t current_count = frame_counter_.load(std::memory_order_relaxed);

    if (elapsed_sec >= window_duration_sec_)
    {
      const uint64_t delta_frames = current_count - last_fps_frame_count_;
      current_fps_.store(static_cast<float>(delta_frames) / elapsed_sec, std::memory_order_relaxed);
      last_fps_time_ = now_tp;
      last_fps_frame_count_ = current_count;
    }
  }

  float FramePerformanceTracker::get_fps() const noexcept
  {
    return current_fps_.load(std::memory_order_relaxed);
  }

  double FramePerformanceTracker::get_latency_ms() const noexcept
  {
    return current_latency_ms_.load(std::memory_order_relaxed);
  }

  uint64_t FramePerformanceTracker::get_total_frames() const noexcept
  {
    return frame_counter_.load(std::memory_order_relaxed);
  }

  void FramePerformanceTracker::reset() noexcept
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    frame_counter_.store(0, std::memory_order_relaxed);
    current_fps_.store(0.0F, std::memory_order_relaxed);
    current_latency_ms_.store(0.0, std::memory_order_relaxed);
    last_fps_time_ = std::chrono::steady_clock::now();
    last_fps_frame_count_ = 0;
  }

  // ===========================================================================
  // CameraInfoScaler Implementation
  // ===========================================================================

  sensor_msgs::msg::CameraInfo CameraInfoScaler::scale(
      const sensor_msgs::msg::CameraInfo &orig_info,
      uint32_t target_w,
      uint32_t target_h,
      bool add_border)
  {
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

    if (add_border)
    {
      // Aspect-ratio-preserving scale with letterbox padding
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
      // Aspect-ratio-preserving scale with center cropping
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

  // ===========================================================================
  // PerceptionDiagnosticsHelper Implementation
  // ===========================================================================

  void PerceptionDiagnosticsHelper::populate_status(
      diagnostic_updater::DiagnosticStatusWrapper &stat,
      bool is_active,
      bool is_busy_or_streaming,
      const std::string &error_msg,
      float min_fps_warning_threshold,
      CustomFieldsFn custom_fields_fn)
  {
    perf_tracker_.tick_window();
    const float fps = perf_tracker_.get_fps();
    const double latency_ms = perf_tracker_.get_latency_ms();
    const uint64_t current_count = perf_tracker_.get_total_frames();

    // 1. Evaluate Overall Summary Status
    if (!error_msg.empty() && error_msg.rfind("Warning:", 0) != 0)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "%s Pipeline error: %s", hardware_id_.c_str(), error_msg.c_str());
    }
    else if (!is_active)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "%s component inactive / unconfigured", hardware_id_.c_str());
    }
    else if (is_busy_or_streaming && fps < min_fps_warning_threshold && current_count > 10)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "Low processing frame rate (%.1f FPS < %.1f FPS)", fps, min_fps_warning_threshold);
    }
    else if (is_busy_or_streaming)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Active (%.1f FPS, %.1f ms latency)", fps, latency_ms);
    }
    else
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Standby / Gated (Idle)");
    }

    // 2. Standardized Telemetry Metrics
    stat.add("Hardware ID", hardware_id_);
    stat.addf("Framerate (FPS)", "%.1f", fps);
    stat.addf("Latency (ms)", "%.2f", latency_ms);
    stat.add("Total Frames Processed", current_count);

    if (!error_msg.empty())
    {
      stat.add("Last Error", error_msg);
    }

    // 3. User / Component Specific Fields
    if (custom_fields_fn)
    {
      custom_fields_fn(stat);
    }
  }

  FramePerformanceTracker &PerceptionDiagnosticsHelper::perf_tracker() noexcept
  {
    return perf_tracker_;
  }

  diagnostic_updater::Updater &PerceptionDiagnosticsHelper::updater() noexcept
  {
    return *updater_;
  }

  // ===========================================================================
  // PerceptionLifecycleHelper Implementation
  // ===========================================================================

  PerceptionLifecycleHelper::PerceptionLifecycleHelper(
      rclcpp_lifecycle::LifecycleNode *node,
      const std::string &hardware_id,
      const std::string &task_name)
      : node_(node),
        diag_helper_(std::make_unique<PerceptionDiagnosticsHelper>(node, hardware_id, task_name))
  {
  }

  void PerceptionLifecycleHelper::setup_camera_mode_sub(
      const std::vector<uint8_t> &allowed_modes,
      ModeChangedCallback on_mode_changed,
      const std::string &topic_name)
  {
    set_allowed_modes(allowed_modes);
    mode_changed_cb_ = std::move(on_mode_changed);

    if (!node_)
    {
      return;
    }

    rclcpp::QoS mode_qos(1);
    mode_qos.reliable();
    mode_qos.transient_local();

    mode_sub_ = node_->create_subscription<lekiwi_interfaces::msg::CameraMode>(
        topic_name, mode_qos,
        std::bind(&PerceptionLifecycleHelper::on_camera_mode_msg, this, std::placeholders::_1));
  }

  void PerceptionLifecycleHelper::set_allowed_modes(const std::vector<uint8_t> &allowed_modes)
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    allowed_modes_ = allowed_modes;
  }

  bool PerceptionLifecycleHelper::is_mode_allowed() const noexcept
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    const uint8_t cur = current_mode_.load(std::memory_order_relaxed);
    if (allowed_modes_.empty())
    {
      return true;
    }
    return std::find(allowed_modes_.begin(), allowed_modes_.end(), cur) != allowed_modes_.end();
  }

  uint8_t PerceptionLifecycleHelper::get_current_mode() const noexcept
  {
    return current_mode_.load(std::memory_order_relaxed);
  }

  void PerceptionLifecycleHelper::set_current_mode(uint8_t mode) noexcept
  {
    const uint8_t old_mode = current_mode_.exchange(mode);
    if (old_mode != mode && mode_changed_cb_)
    {
      mode_changed_cb_(mode);
    }
  }

  void PerceptionLifecycleHelper::on_camera_mode_msg(
      const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg)
  {
    if (msg)
    {
      set_current_mode(msg->value);
    }
  }

  void PerceptionLifecycleHelper::setup_autostart(bool autostart)
  {
    if (!autostart || !node_)
    {
      return;
    }

    autostart_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]()
        {
          if (autostart_timer_)
          {
            autostart_timer_->cancel();
            autostart_timer_.reset();
          }
          if (!node_)
          {
            return;
          }
          if (node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
          {
            node_->configure();
          }
          if (node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
          {
            node_->activate();
          }
        });
  }

  void PerceptionLifecycleHelper::reset() noexcept
  {
    if (autostart_timer_)
    {
      autostart_timer_->cancel();
      autostart_timer_.reset();
    }
    mode_sub_.reset();
    perf_tracker().reset();
  }

  FramePerformanceTracker &PerceptionLifecycleHelper::perf_tracker() noexcept
  {
    return diag_helper_->perf_tracker();
  }

  PerceptionDiagnosticsHelper &PerceptionLifecycleHelper::diagnostics() noexcept
  {
    return *diag_helper_;
  }

  void PerceptionLifecycleHelper::update_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat,
      bool is_busy_or_streaming,
      const std::string &error_msg,
      float min_fps_warning_threshold,
      PerceptionDiagnosticsHelper::CustomFieldsFn custom_fields_fn)
  {
    const bool is_active = (node_ && node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    diag_helper_->populate_status(
        stat, is_active, is_busy_or_streaming, error_msg,
        min_fps_warning_threshold, custom_fields_fn);
  }

} // namespace lekiwi_perception::utils

/**
 * @file perception_utils.hpp
 * @brief Consolidated reusable utilities for lekiwi_perception components.
 *
 * Implements:
 * - FramePerformanceTracker: Thread-safe rolling FPS, latency, and frame counting.
 * - CameraInfoScaler: Geometric scaling of CameraInfo K/P matrices (crop and letterbox).
 * - PerceptionDiagnosticsHelper: Standardized ROS 2 /diagnostics reporting.
 * - PerceptionLifecycleHelper: Composition-over-inheritance lifecycle, camera mode gating,
 *   autostart, and diagnostics manager for LifecycleNodes.
 *
 * @author LeKiwi Engineering
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__PERCEPTION_UTILS_HPP_
#define LEKIWI_PERCEPTION__PERCEPTION_UTILS_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <lekiwi_interfaces/msg/perception_context.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace lekiwi_perception::utils
{

  /**
   * @brief Thread-safe performance tracker calculating rolling FPS and tracking latency.
   */
  class FramePerformanceTracker
  {
  public:
    explicit FramePerformanceTracker(float window_duration_sec = 0.5F);

    /**
     * @brief Records a processed frame with optional latency in milliseconds.
     */
    void record_frame(double latency_ms = 0.0) noexcept;

    /**
     * @brief Updates rolling FPS window calculation.
     */
    void tick_window() noexcept;

    [[nodiscard]] float get_fps() const noexcept;
    [[nodiscard]] double get_latency_ms() const noexcept;
    [[nodiscard]] uint64_t get_total_frames() const noexcept;
    void reset() noexcept;

  private:
    float window_duration_sec_{0.5F};
    std::chrono::steady_clock::time_point last_fps_time_;
    uint64_t last_fps_frame_count_{0};

    std::atomic<uint64_t> frame_counter_{0};
    std::atomic<float> current_fps_{0.0F};
    std::atomic<double> current_latency_ms_{0.0};
    std::mutex time_mutex_;
  };

  /**
   * @brief Helper class for scaling camera intrinsic and projection matrices.
   */
  class CameraInfoScaler
  {
  public:
    /**
     * @brief Scales camera intrinsic matrix K and projection matrix P.
     * @param[in] orig_info Original CameraInfo message.
     * @param[in] target_w Desired output width.
     * @param[in] target_h Desired output height.
     * @param[in] add_border If true, applies letterbox padding. If false, applies center crop.
     * @return Scaled CameraInfo message.
     */
    [[nodiscard]] static sensor_msgs::msg::CameraInfo scale(
        const sensor_msgs::msg::CameraInfo &orig_info,
        uint32_t target_w,
        uint32_t target_h,
        bool add_border);
  };

  /**
   * @brief Standardized diagnostic status builder and telemetry helper for perception tasks.
   */
  class PerceptionDiagnosticsHelper
  {
  public:
    using CustomFieldsFn = std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)>;

    template <typename NodeT>
    PerceptionDiagnosticsHelper(
        NodeT *node,
        const std::string &hardware_id,
        const std::string &task_name = "")
        : hardware_id_(hardware_id),
          task_name_(task_name),
          updater_(std::make_shared<diagnostic_updater::Updater>(node))
    {
      updater_->setHardwareID(hardware_id_);
    }

    /**
     * @brief Populates diagnostic status wrapper with standardized metrics and state evaluation.
     */
    void populate_status(
        diagnostic_updater::DiagnosticStatusWrapper &stat,
        bool is_active,
        bool is_busy_or_streaming,
        const std::string &error_msg,
        float min_fps_warning_threshold = 1.0F,
        CustomFieldsFn custom_fields_fn = nullptr);

    [[nodiscard]] FramePerformanceTracker &perf_tracker() noexcept;
    [[nodiscard]] diagnostic_updater::Updater &updater() noexcept;

  private:
    std::string hardware_id_;
    std::string task_name_;
    std::shared_ptr<diagnostic_updater::Updater> updater_;
    FramePerformanceTracker perf_tracker_;
  };

  /**
   * @brief Composition-over-inheritance lifecycle helper.
   *
   * Manages:
   * 1. Relative "~/perception_context" topic subscription with latched QoS.
   * 2. Allowed perception contexts list and `is_context_allowed()` gating.
   * 3. Optional 1ms autostart timer.
   * 4. Integrated diagnostics updater and performance tracking.
   */
  class PerceptionLifecycleHelper
  {
  public:
    using ContextChangedCallback = std::function<void(uint8_t new_context)>;

    PerceptionLifecycleHelper(
        rclcpp_lifecycle::LifecycleNode *node,
        const std::string &hardware_id,
        const std::string &task_name);

    /**
     * @brief Configures subscription to /perception_context.
     * @param[in] allowed_contexts Set of allowed contexts. If empty, all contexts are allowed.
     * @param[in] on_context_changed Optional callback fired when context value changes.
     * @param[in] topic_name Topic name for perception context.
     */
    void setup_perception_context_sub(
        const std::vector<uint8_t> &allowed_contexts = {},
        ContextChangedCallback on_context_changed = nullptr,
        const std::string &topic_name = "/perception_context");

    /**
     * @brief Sets allowed contexts dynamically.
     */
    void set_allowed_contexts(const std::vector<uint8_t> &allowed_contexts);

    /**
     * @brief Checks if the current perception context is within the allowed contexts list.
     */
    [[nodiscard]] bool is_context_allowed() const noexcept;

    [[nodiscard]] uint8_t get_current_context() const noexcept;
    void set_current_context(uint8_t context) noexcept;

    /**
     * @brief Configures one-shot 1ms autostart timer to configure and activate node.
     */
    void setup_autostart(bool autostart);

    /**
     * @brief Resets autostart timer and internal state.
     */
    void reset() noexcept;

    [[nodiscard]] FramePerformanceTracker &perf_tracker() noexcept;
    [[nodiscard]] PerceptionDiagnosticsHelper &diagnostics() noexcept;

    /**
     * @brief Produces lifecycle-aware diagnostics status.
     */
    void update_diagnostics(
        diagnostic_updater::DiagnosticStatusWrapper &stat,
        bool is_busy_or_streaming,
        const std::string &error_msg,
        float min_fps_warning_threshold = 1.0F,
        PerceptionDiagnosticsHelper::CustomFieldsFn custom_fields_fn = nullptr);

  private:
    void on_perception_context_msg(const lekiwi_interfaces::msg::PerceptionContext::ConstSharedPtr &msg);

    rclcpp_lifecycle::LifecycleNode *node_{nullptr};
    std::atomic<uint8_t> current_context_{lekiwi_interfaces::msg::PerceptionContext::IDLE_STANDBY};
    std::vector<uint8_t> allowed_contexts_;
    mutable std::mutex context_mutex_;

    ContextChangedCallback context_changed_cb_;
    rclcpp::Subscription<lekiwi_interfaces::msg::PerceptionContext>::SharedPtr context_sub_;
    rclcpp::TimerBase::SharedPtr autostart_timer_;

    std::unique_ptr<PerceptionDiagnosticsHelper> diag_helper_;
  };

} // namespace lekiwi_perception::utils

#endif // LEKIWI_PERCEPTION__PERCEPTION_UTILS_HPP_

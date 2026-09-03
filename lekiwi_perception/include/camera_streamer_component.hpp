/**
 * @file camera_streamer_component.hpp
 * @brief GStreamer-based camera streaming lifecycle component for LeKiwi perception.
 *
 * Wraps GStreamer pipeline execution (compatible with gscam configurations) inside a ROS 2
 * lifecycle component. Publishes `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`
 * using zero-copy intra-process comms where available.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__CAMERA_STREAMER_COMPONENT_HPP_
#define LEKIWI_PERCEPTION__CAMERA_STREAMER_COMPONENT_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include "lekiwi_interfaces/msg/camera_mode.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace lekiwi_perception
{

  /**
   * @brief ROS 2 Lifecycle Component streaming video frames via GStreamer.
   *
   * Manages GStreamer pipeline lifecycle (`PLAYING`, `PAUSED`, `NULL`), mode-based GStreamer valve control,
   * camera info calibration loading, and health diagnostics publishing.
   */
  class CameraStreamerComponent : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    /**
     * @brief Constructs CameraStreamerComponent with node options.
     * @param[in] options Node options passed by component container.
     */
    explicit CameraStreamerComponent(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CameraStreamerComponent() override;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief Configures parameters, loads calibration URL, and creates GStreamer pipeline.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Activates image publishers and sets GStreamer pipeline state to PLAYING.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Deactivates image publishers and pauses GStreamer pipeline.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Cleans up GStreamer pipeline and bus elements.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Handles node shutdown transition.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Handles error state transition and resets pipeline.
     */
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    // Public getters for testing / status introspection
    [[nodiscard]] bool is_streaming() const noexcept;
    [[nodiscard]] uint8_t current_camera_mode() const noexcept;
    [[nodiscard]] bool is_valve_open() const;

  private:
    void on_camera_mode(const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg);
    void update_valve_state();
    void monitor_tick();
    void poll_bus_errors();
    void reset_pipeline();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data);
    void process_sample(GstSample *sample);

    // GStreamer elements and bus
    GstElement *pipeline_{nullptr};
    GstBus *bus_{nullptr};
    GstElement *appsink_{nullptr};
    GstElement *valve_{nullptr};
    gulong sample_signal_id_{0};

    // ROS 2 publishers and subscriptions
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Subscription<lekiwi_interfaces::msg::CameraMode>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr autostart_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    // Diagnostic updater
    std::shared_ptr<diagnostic_updater::Updater> updater_;

    // Parameters (compatible with gscam)
    std::string gscam_config_;
    std::string camera_name_{"camera"};
    std::string frame_id_{"camera"};
    std::string camera_info_url_;
    std::string image_encoding_{"rgb8"};
    bool sync_sink_{false};
    bool use_gst_timestamps_{false};
    bool use_sensor_data_qos_{true};
    bool autostart_{true};
    bool calib_mode_{false};
    std::vector<int64_t> active_modes_;
    std::string valve_name_{"gate"};

    // State & Thread safety
    std::mutex gst_mutex_;
    std::atomic<uint8_t> current_camera_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
    std::atomic<bool> is_streaming_{false};
    std::atomic<uint64_t> frame_counter_{0};

    // Telemetry & Diagnostic stats
    std::atomic<double> current_latency_ms_{0.0};
    std::atomic<float> current_fps_{0.0F};
    std::chrono::steady_clock::time_point last_fps_time_;
    uint64_t last_fps_frame_count_{0};
    std::string last_gst_error_;
  };

} // namespace lekiwi_perception

#endif // LEKIWI_PERCEPTION__CAMERA_STREAMER_COMPONENT_HPP_

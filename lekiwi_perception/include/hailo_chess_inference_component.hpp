/**
 * @file hailo_chess_inference_component.hpp
 * @brief Perception node running Hailo chessboard and piece detection pipeline.
 *
 * Implements a lifecycle node managing camera frame inference through Hailo NPU,
 * publishes piece detections, spatial board states (Full FEN), and optional debug images.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_
#define LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <lekiwi_interfaces/srv/set_cam_mode.hpp>
#include <lekiwi_interfaces/msg/camera_mode.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hailo/chess_game_state_tracker.hpp"
#include "hailo/chess_vision_mapper.hpp"
#include "hailo/hailo_gst_pipeline.hpp"

namespace lekiwi_perception
{

  /**
   * @brief Lifecycle-managed component running neural network inference on camera frames.
   */
  class HailoChessInferenceComponent : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit HailoChessInferenceComponent(const rclcpp::NodeOptions &options);
    ~HailoChessInferenceComponent() override;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief Loads model HEF paths, initializes ROS 2 publishers, services, and Hailo pipeline.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Starts Hailo GStreamer pipeline and activates publishers.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Pauses Hailo pipeline and deactivates publishers.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Cleans up Hailo pipeline instance.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Handles shutdown transition.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Resets pipeline state on lifecycle error.
     */
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

  private:
    void handle_sample(GstSample *sample, GstElement *pipeline);
    void handle_image_input(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void handle_tag_centers(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr &msg);
    void handle_set_mode(
        const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
        std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response);
    void poll_bus_errors();
    void reset_state();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    /// Parameters
    std::string camera_topic_{"/cameras/stereo_left/image_raw"};
    std::string fen_topic_{"/chess/fen"};
    std::string detections_topic_{"/chess/detections_2d"};
    std::string tag_centers_topic_{"/chess/tag_centers"};
    std::string board_hef_path_;
    std::string pcs_hef_path_;
    std::string vdevice_group_id_{"lekiwi_chess"};
    std::string frame_id_{"stereo_left_optical"};
    double confidence_threshold_{0.35};
    std::chrono::milliseconds transition_timeout_{5000};

    std::unique_ptr<HailoGstPipeline> hailo_pipeline_;
    std::unique_ptr<hailo::ChessGameStateTracker> game_tracker_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fen_pub_;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;

    rclcpp::Service<lekiwi_interfaces::srv::SetCamMode>::SharedPtr mode_srv_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr tag_centers_sub_;
    std::mutex tags_mutex_;
    std::vector<hailo::Tag2D> latest_tags_;
    std::atomic<int> a1_corner_idx_{0};
    rclcpp::TimerBase::SharedPtr bus_timer_;

    // Diagnostic Updater
    std::shared_ptr<diagnostic_updater::Updater> updater_;

    std::mutex state_mutex_;
    std::atomic<uint8_t> current_camera_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
    std::string pipeline_state_{"STOPPED"};
    std::string last_error_;
    std::string last_logged_fen_;

    std::atomic<uint64_t> frame_counter_{0};
    std::chrono::steady_clock::time_point last_fps_time_;
    uint64_t last_fps_frame_count_{0};
    std::atomic<float> current_fps_{0.0F};
    std::atomic<double> current_latency_ms_{0.0};
  };

} // namespace lekiwi_perception

#endif // LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_

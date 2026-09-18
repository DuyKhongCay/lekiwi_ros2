/**
 * @file hailo_chess_inference_component.hpp
 * @brief Perception node running Hailo chessboard and piece detection pipeline.
 *
 * Implements a lifecycle node managing camera frame inference through Hailo NPU,
 * publishes piece detections, spatial board states (Full FEN), and optional debug images.
 * Integrates PerceptionLifecycleHelper for camera mode gating, autostart, and diagnostics.
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

#include "hailo/chess_constants.hpp"
#include "hailo/chess_vision_mapper.hpp"
#include "hailo/hailo_gst_pipeline.hpp"
#include "perception_utils.hpp"

namespace lekiwi_perception
{

  /**
   * @brief Pipeline state enum class eliminating string data races.
   */
  enum class PipelineState : uint8_t
  {
    STOPPED,
    STARTING,
    RUNNING,
    STOPPING,
    ERROR
  };

  inline const char *to_string(PipelineState s)
  {
    switch (s)
    {
    case PipelineState::STOPPED:
      return "STOPPED";
    case PipelineState::STARTING:
      return "STARTING";
    case PipelineState::RUNNING:
      return "RUNNING";
    case PipelineState::STOPPING:
      return "STOPPING";
    case PipelineState::ERROR:
      return "ERROR";
    }
    return "UNKNOWN";
  }

  /**
   * @brief Lifecycle-managed component running neural network inference on camera frames.
   */
  class HailoChessInferenceComponent : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit HailoChessInferenceComponent(const rclcpp::NodeOptions &options);
    ~HailoChessInferenceComponent() override;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    [[nodiscard]] uint8_t current_camera_mode() const noexcept
    {
      return lifecycle_helper_ ? lifecycle_helper_->get_current_mode() : 0;
    }

    void handle_set_mode(
        const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
        std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response);

  private:
    void handle_sample(GstSample *sample, GstElement *pipeline);
    void handle_image_input(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void handle_tag_centers(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr &msg);
    void poll_bus_errors();
    void reset_state();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    /// Parameters
    std::string camera_topic_{"/cameras/stereo_left/image_raw"};
    std::string fen_topic_{"/chess/raw_fen"};
    std::string detections_topic_{"/chess/detections_2d"};
    std::string tag_centers_topic_{"/chess/tag_centers"};
    std::string grid_points_topic_{"/chess/grid_points"};
    std::string board_hef_path_;
    std::string pcs_hef_path_;
    std::string vdevice_group_id_{"lekiwi_chess"};
    std::string frame_id_{"stereo_left_optical"};
    double confidence_threshold_{0.35};
    bool debug_{true};
    std::chrono::milliseconds transition_timeout_{5000};
    std::map<int, int> tag_offsets_;

    std::unique_ptr<HailoGstPipeline> hailo_pipeline_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fen_pub_;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr grid_points_pub_;

    rclcpp::Service<lekiwi_interfaces::srv::SetCamMode>::SharedPtr mode_srv_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr tag_centers_sub_;
    std::mutex tags_mutex_;
    std::vector<hailo::Tag2D> latest_tags_;
    std::atomic<int> a1_corner_idx_{0};
    rclcpp::TimerBase::SharedPtr bus_timer_;

    // Composition helper for Lifecycle, Mode Gating, Autostart & Diagnostics
    std::unique_ptr<utils::PerceptionLifecycleHelper> lifecycle_helper_;

    std::atomic<PipelineState> pipeline_state_{PipelineState::STOPPED};
    std::mutex error_mutex_;
    std::string last_error_;
    std::string last_logged_fen_;

    /// Structure representing an unmapped or conflicting chess piece detection
    struct UnmappedPieceInfo
    {
      std::string label;
      std::string square;
      float confidence{0.0F};
      bool is_duplicate{false};
    };

    // YOLO Debug Telemetry (populated when debug_ is true)
    std::mutex debug_metrics_mutex_;
    int debug_yolo_detections_{0};
    int debug_yolo_mapped_{0};
    float debug_yolo_avg_conf_{0.0F};
    float debug_yolo_min_conf_{0.0F};
    std::string debug_yolo_placement_;
    std::vector<UnmappedPieceInfo> debug_unmapped_pieces_;
  };

} // namespace lekiwi_perception

#endif // LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_

// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__MULTI_CAMERA_HUB_COMPONENT_HPP_
#define LEKIWI_PERCEPTION__MULTI_CAMERA_HUB_COMPONENT_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "cam_hub_stream_processor.hpp"
#include "camera_geometry.hpp"
#include "camera_hub_config.hpp"
#include "gstreamer/gst_pipe.hpp"
#include "gstreamer/pipe_builder.hpp"
#include "lekiwi_interfaces/msg/cam_hub_status.hpp"
#include "lekiwi_interfaces/msg/camera_mode.hpp"
#include "lekiwi_interfaces/srv/set_cam_mode.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace lekiwi_perception
{

class MultiCameraHubComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MultiCameraHubComponent(const rclcpp::NodeOptions & options);
  ~MultiCameraHubComponent() override;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  static constexpr std::size_t kStreamCount = 4U;

  void create_ros_entities();
  void reset_ros_entities();
  void calibrate_clock_bridge();
  void reset_metrics();

  void handle_set_mode(
    const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
    std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response);
  [[nodiscard]] bool apply_mode(uint8_t requested_mode, std::string & message);
  [[nodiscard]] bool start_hailo(std::string & error);
  [[nodiscard]] bool start_lerobot(std::string & error);
  [[nodiscard]] bool stop_hailo(std::string & error);
  [[nodiscard]] bool stop_lerobot(std::string & error);
  [[nodiscard]] bool stop_all(std::string & error);

  void poll_pipe_errors();
  void publish_status();
  void set_pipe_state(bool hailo, uint8_t state);
  void record_set_mode_delay(std::chrono::steady_clock::duration elapsed);
  void set_error(const std::string & error);
  [[nodiscard]] bool lifecycle_active() const;

  MultiCameraHubConfig hub_config_;
  std::array<GeomPlan, kStreamCount> geometry_plans_;
  std::array<sensor_msgs::msg::CameraInfo, kStreamCount> camera_infos_;

  GstClock * common_clock_{nullptr};
  int64_t gst_to_ros_offset_ns_{0};
  std::unique_ptr<GstPipeController> hailo_pipe_;
  std::unique_ptr<GstPipeController> lerobot_pipe_;
  std::unique_ptr<CamHubStreamProcessor> stream_processor_;

  std::array<
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr,
    kStreamCount> image_pubs_;
  std::array<
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr,
    kStreamCount> info_pubs_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fen_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr
    detections_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr debug_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<lekiwi_interfaces::msg::CamHubStatus>::SharedPtr status_pub_;
  rclcpp::Service<lekiwi_interfaces::srv::SetCamMode>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr bus_timer_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr monitoring_group_;

  mutable std::mutex trans_mutex_;
  mutable std::mutex state_mutex_;
  std::atomic<uint64_t> generation_{0};
  uint8_t effective_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
  uint8_t prev_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
  uint8_t hailo_state_{lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED};
  uint8_t lerobot_state_{lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED};
  std::string last_error_;
  float set_mode_delay_ms_{-1.0F};
};

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__MULTI_CAMERA_HUB_COMPONENT_HPP_

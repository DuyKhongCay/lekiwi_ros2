// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_
#define LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_

#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "hailo/hailo_gst_pipeline.hpp"
#include "hailo/chess_vision_mapper.hpp"
#include "lekiwi_interfaces/msg/camera_mode.hpp"
#include "lekiwi_interfaces/msg/hailo_inference_status.hpp"
#include "lekiwi_interfaces/srv/set_cam_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace lekiwi_perception
{

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

  private:
    void handle_sample(GstSample *sample, GstElement *pipeline);
    void handle_image_input(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void handle_set_mode(
        const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
        std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response);
    void publish_status();
    void poll_bus_errors();
    void reset_state();

    HailoPipelineConfig pipeline_config_;
    std::string frame_id_{"stereo_left_optical"};
    bool publish_debug_image_{true};
    std::chrono::milliseconds transition_timeout_{5000};
    std::chrono::milliseconds status_period_{1000};

    std::unique_ptr<HailoGstPipeline> hailo_pipeline_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fen_pub_;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp_lifecycle::LifecyclePublisher<lekiwi_interfaces::msg::HailoInferenceStatus>::SharedPtr status_pub_;

    rclcpp::Service<lekiwi_interfaces::srv::SetCamMode>::SharedPtr mode_srv_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr bus_timer_;

    std::mutex state_mutex_;
    std::atomic<uint8_t> current_camera_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
    uint8_t pipeline_state_{lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_STOPPED};
    std::string last_error_;
    std::string last_logged_fen_;

    std::atomic<uint64_t> frame_counter_{0};
    std::chrono::steady_clock::time_point last_fps_time_;
    uint64_t last_fps_frame_count_{0};
    float current_fps_{0.0F};
  };

} // namespace lekiwi_perception

#endif // LEKIWI_PERCEPTION__HAILO_CHESS_INFERENCE_COMPONENT_HPP_

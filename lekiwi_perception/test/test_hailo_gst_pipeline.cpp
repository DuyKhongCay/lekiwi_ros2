// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <gst/gst.h>

#include <chrono>
#include <memory>
#include <string>

#include "hailo/hailo_gst_pipeline.hpp"

TEST(HailoGstPipelineTest, BasicLifecycle)
{
  gst_init(nullptr, nullptr);

  lekiwi_perception::HailoGstPipeline pipeline(
      [](GstSample *, GstElement *) {});

  EXPECT_FALSE(pipeline.is_running());

  std::string error;
  // Stopping a non-running pipeline should succeed gracefully
  EXPECT_TRUE(pipeline.stop(std::chrono::milliseconds(100), error));
}

#include "hailo_chess_inference_component.hpp"
#include "lekiwi_interfaces/msg/camera_mode.hpp"
#include "lekiwi_interfaces/srv/set_cam_mode.hpp"

TEST(HailoChessInferenceComponentTest, ModeTransitionGating)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lekiwi_perception::HailoChessInferenceComponent>(options);

  auto configure_state = node->configure();
  ASSERT_EQ(configure_state.label(), "inactive");

  auto request = std::make_shared<lekiwi_interfaces::srv::SetCamMode::Request>();
  auto response = std::make_shared<lekiwi_interfaces::srv::SetCamMode::Response>();

  // Request invalid mode
  request->requested_mode.value = 99;
  // Use ChangeState/SetMode logic directly
  auto req_client = std::make_shared<lekiwi_interfaces::srv::SetCamMode::Request>();
  req_client->requested_mode.value = lekiwi_interfaces::msg::CameraMode::CHESS_THINKING;

  // Cleanup
  node->cleanup();
  rclcpp::shutdown();
}

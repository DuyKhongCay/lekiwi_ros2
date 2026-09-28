/**
 * @file test_hailo_gst_pipeline.cpp
 * @brief Unit tests (L1 verification) for HailoGstPipeline wrapper lifecycle.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <gst/gst.h>

#include <chrono>
#include <memory>
#include <string>

#include "hailo/hailo_gst_pipeline.hpp"
#include "hailo_chess_inference_component.hpp"
#include "lekiwi_interfaces/msg/perception_context.hpp"
#include "lekiwi_interfaces/srv/set_perception_context.hpp"

class HailoGstPipelineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(HailoGstPipelineTest, BasicLifecycle)
{
  gst_init(nullptr, nullptr);

  lekiwi_perception::HailoGstPipeline pipeline(
      [](GstSample *, GstElement *) {});

  EXPECT_FALSE(pipeline.is_running());

  std::string error;
  // Stopping a non-running pipeline should succeed gracefully
  EXPECT_TRUE(pipeline.stop(std::chrono::milliseconds(100), error));
}

TEST_F(HailoGstPipelineTest, ContextTransitionGating)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lekiwi_perception::HailoChessInferenceComponent>(options);

  auto configure_state = node->configure();
  ASSERT_EQ(configure_state.label(), "inactive");

  auto request = std::make_shared<lekiwi_interfaces::srv::SetPerceptionContext::Request>();
  auto response = std::make_shared<lekiwi_interfaces::srv::SetPerceptionContext::Response>();

  // 1. Request invalid context (e.g. 99)
  request->requested_context.value = 99;
  node->handle_set_perception_context(request, response);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Invalid perception context requested");

  // 2. Request valid context (BOARD_STATE_SCAN = 2)
  request->requested_context.value = lekiwi_interfaces::msg::PerceptionContext::BOARD_STATE_SCAN;
  node->handle_set_perception_context(request, response);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->applied_context.value, lekiwi_interfaces::msg::PerceptionContext::BOARD_STATE_SCAN);
  EXPECT_EQ(node->current_perception_context(), lekiwi_interfaces::msg::PerceptionContext::BOARD_STATE_SCAN);

  // 3. Request IDLE_STANDBY context (0)
  request->requested_context.value = lekiwi_interfaces::msg::PerceptionContext::IDLE_STANDBY;
  node->handle_set_perception_context(request, response);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->applied_context.value, lekiwi_interfaces::msg::PerceptionContext::IDLE_STANDBY);
  EXPECT_EQ(node->current_perception_context(), lekiwi_interfaces::msg::PerceptionContext::IDLE_STANDBY);

  // Cleanup without rclcpp::shutdown() to avoid side effects on other tests
  node->cleanup();
}

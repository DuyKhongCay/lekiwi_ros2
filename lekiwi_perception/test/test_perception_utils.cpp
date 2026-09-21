/**
 * @file test_perception_utils.cpp
 * @brief Unit tests for FramePerformanceTracker, CameraInfoScaler,
 *        PerceptionDiagnosticsHelper, and PerceptionLifecycleHelper.
 *
 * @author LeKiwi Engineering
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "perception_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace lekiwi_perception::utils;

class PerceptionUtilsTest : public ::testing::Test
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

// =============================================================================
// FramePerformanceTracker Tests
// =============================================================================

TEST_F(PerceptionUtilsTest, FramePerformanceTrackerBasic)
{
  FramePerformanceTracker tracker(0.1F); // 100ms window for fast testing

  EXPECT_EQ(tracker.get_total_frames(), 0U);
  EXPECT_FLOAT_EQ(tracker.get_fps(), 0.0F);
  EXPECT_DOUBLE_EQ(tracker.get_latency_ms(), 0.0);

  tracker.record_frame(12.5);
  EXPECT_EQ(tracker.get_total_frames(), 1U);
  EXPECT_DOUBLE_EQ(tracker.get_latency_ms(), 12.5);

  // Record 10 frames over > 100ms
  for (int i = 0; i < 9; ++i)
  {
    tracker.record_frame(10.0 + i);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  tracker.record_frame(15.0);

  EXPECT_EQ(tracker.get_total_frames(), 11U);
  EXPECT_GT(tracker.get_fps(), 0.0F);

  tracker.reset();
  EXPECT_EQ(tracker.get_total_frames(), 0U);
  EXPECT_FLOAT_EQ(tracker.get_fps(), 0.0F);
  EXPECT_DOUBLE_EQ(tracker.get_latency_ms(), 0.0);
}

// =============================================================================
// CameraInfoScaler Tests
// =============================================================================

TEST_F(PerceptionUtilsTest, CameraInfoScalerLetterbox)
{
  sensor_msgs::msg::CameraInfo orig;
  orig.width = 1280;
  orig.height = 720; // 16:9
  orig.k = {1000.0, 0.0, 640.0,
            0.0, 1000.0, 360.0,
            0.0, 0.0, 1.0};
  orig.p = {1000.0, 0.0, 640.0, 0.0,
            0.0, 1000.0, 360.0, 0.0,
            0.0, 0.0, 1.0, 0.0};

  // Target 640x640 (1:1) with add_border = true (letterbox)
  const auto scaled = CameraInfoScaler::scale(orig, 640, 640, true);

  EXPECT_EQ(scaled.width, 640U);
  EXPECT_EQ(scaled.height, 640U);

  // Scale factor sx = sy = 640 / 1280 = 0.5
  EXPECT_DOUBLE_EQ(scaled.k[0], 500.0);
  EXPECT_DOUBLE_EQ(scaled.k[4], 500.0);

  // Cx = 640 * 0.5 + 0 = 320.0
  EXPECT_DOUBLE_EQ(scaled.k[2], 320.0);

  // Active height = 720 * 0.5 = 360, offset_y = (640 - 360) / 2 = 140
  // Cy = 360 * 0.5 + 140 = 320.0
  EXPECT_DOUBLE_EQ(scaled.k[5], 320.0);
}

TEST_F(PerceptionUtilsTest, CameraInfoScalerCenterCrop)
{
  sensor_msgs::msg::CameraInfo orig;
  orig.width = 1280;
  orig.height = 720;
  orig.k = {1000.0, 0.0, 640.0,
            0.0, 1000.0, 360.0,
            0.0, 0.0, 1.0};
  orig.p = {1000.0, 0.0, 640.0, 0.0,
            0.0, 1000.0, 360.0, 0.0,
            0.0, 0.0, 1.0, 0.0};

  // Target 360x360 (1:1) with add_border = false (center crop)
  const auto scaled = CameraInfoScaler::scale(orig, 360, 360, false);

  EXPECT_EQ(scaled.width, 360U);
  EXPECT_EQ(scaled.height, 360U);
  EXPECT_GT(scaled.k[0], 0.0);
  EXPECT_GT(scaled.k[4], 0.0);
}

// =============================================================================
// PerceptionDiagnosticsHelper & PerceptionLifecycleHelper Tests
// =============================================================================

TEST_F(PerceptionUtilsTest, PerceptionLifecycleHelperModeGating)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_lifecycle_node");
  PerceptionLifecycleHelper helper(node.get(), "test_hw", "test_task");

  // Allow mode 0 (STANDBY) and mode 2 (CHESS_THINKING)
  uint8_t notified_mode = 255;
  helper.setup_camera_mode_sub({0, 2}, [&](uint8_t m)
                               { notified_mode = m; }, "/test_camera_mode");

  EXPECT_TRUE(helper.is_mode_allowed()); // Standby is 0 by default
  EXPECT_EQ(helper.get_current_mode(), 0);

  // Set mode to 1 (NAVIGATING) -> not allowed
  helper.set_current_mode(1);
  EXPECT_EQ(notified_mode, 1);
  EXPECT_EQ(helper.get_current_mode(), 1);
  EXPECT_FALSE(helper.is_mode_allowed());

  // Set mode to 2 (CHESS_THINKING) -> allowed
  helper.set_current_mode(2);
  EXPECT_EQ(notified_mode, 2);
  EXPECT_TRUE(helper.is_mode_allowed());
}

TEST_F(PerceptionUtilsTest, PerceptionDiagnosticsStatusEvaluation)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_diag_node");
  PerceptionDiagnosticsHelper helper(node.get(), "test_camera", "stream_status");

  diagnostic_updater::DiagnosticStatusWrapper stat;

  // 1. Inactive node -> WARN
  helper.populate_status(stat, false, false, "");
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  // 2. Active + Error message -> ERROR
  helper.populate_status(stat, true, true, "Pipeline link failed");
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  // 3. Active + Idle (not streaming) -> OK
  helper.populate_status(stat, true, false, "");
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  // 4. Custom fields verification
  bool custom_called = false;
  helper.populate_status(stat, true, false, "", 1.0F, [&](diagnostic_updater::DiagnosticStatusWrapper &s)
                         {
    s.add("CustomKey", "CustomVal");
    custom_called = true; });
  EXPECT_TRUE(custom_called);
}

TEST_F(PerceptionUtilsTest, PerceptionLifecycleHelperConcurrency)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_concurrency_node");
  PerceptionLifecycleHelper helper(node.get(), "test_hw", "test_task");

  constexpr int kIterations = 5000;

  // Thread 1: Continuously toggles allowed modes
  std::thread writer_modes([&]()
                           {
    for (int i = 0; i < kIterations; ++i)
    {
      if (i % 2 == 0)
      {
        helper.set_allowed_modes({0, 1});
      }
      else
      {
        helper.set_allowed_modes({1, 2, 3});
      }
    } });

  // Thread 2: Continuously changes current camera mode
  std::thread writer_state([&]()
                           {
    for (int i = 0; i < kIterations; ++i)
    {
      helper.set_current_mode(static_cast<uint8_t>(i % 4));
    } });

  // Thread 3: Continuously reads is_mode_allowed()
  std::thread reader([&]()
                     {
    for (int i = 0; i < kIterations; ++i)
    {
      (void)helper.is_mode_allowed();
    } });

  writer_modes.join();
  writer_state.join();
  reader.join();

  EXPECT_TRUE(true);
}

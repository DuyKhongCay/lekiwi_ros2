/**
 * @file test_camera_streamer_component.cpp
 * @brief Unit & lifecycle integration tests (L1/L2) for CameraStreamerComponent.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <gst/gst.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "camera_streamer_component.hpp"
#include "lekiwi_interfaces/msg/camera_mode.hpp"
#include "rclcpp/rclcpp.hpp"

class CameraStreamerComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    // cleanup
  }
};

TEST_F(CameraStreamerComponentTest, BasicLifecycleAndGating)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "test_camera"},
                               {"frame_id", "test_camera_optical"},
                               {"gscam_config", "videotestsrc is-live=true ! valve name=gate drop=true ! video/x-raw,format=RGB,width=320,height=240,framerate=15/1"},
                               {"active_modes", std::vector<int64_t>{1, 2}},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);

  // Initial state: Unconfigured
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(node->is_streaming());

  // Transition: configure
  auto state = node->configure();
  ASSERT_EQ(state.label(), "inactive");
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  // Transition: activate
  state = node->activate();
  ASSERT_EQ(state.label(), "active");

  // In STANDBY mode (0), even when active, camera is not allowed in active_modes [1, 2]
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  // Create a subscriber to satisfy subscriber gating
  auto helper_node = std::make_shared<rclcpp::Node>("test_subscriber_node");
  size_t msg_count = 0;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", rclcpp::SensorDataQoS(),
      [&msg_count](const sensor_msgs::msg::Image::ConstSharedPtr)
      {
        msg_count++;
      });

  // Switch mode to NAVIGATING (1)
  auto mode_msg = std::make_shared<lekiwi_interfaces::msg::CameraMode>();
  mode_msg->value = lekiwi_interfaces::msg::CameraMode::NAVIGATING;

  // Let executor process subscriber graph connection and mode
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper_node);

  // Directly trigger mode callback
  auto mode_pub = helper_node->create_publisher<lekiwi_interfaces::msg::CameraMode>(
      "/camera_mode", rclcpp::QoS(1).reliable().transient_local());
  mode_pub->publish(*mode_msg);

  // Spin briefly to process mode message and timer
  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_camera_mode(), lekiwi_interfaces::msg::CameraMode::NAVIGATING);
  EXPECT_TRUE(node->is_streaming());
  EXPECT_TRUE(node->is_valve_open());

  // Switch mode to MANIPULATION_LEROBOT (3) -> should gate/drop
  mode_msg->value = lekiwi_interfaces::msg::CameraMode::MANIPULATION_LEROBOT;
  mode_pub->publish(*mode_msg);

  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_camera_mode(), lekiwi_interfaces::msg::CameraMode::MANIPULATION_LEROBOT);
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  // Transition: deactivate
  state = node->deactivate();
  EXPECT_EQ(state.label(), "inactive");
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  // Transition: cleanup
  state = node->cleanup();
  EXPECT_EQ(state.label(), "unconfigured");

  // Transition: shutdown
  state = node->shutdown();
  EXPECT_EQ(state.label(), "finalized");
}

TEST_F(CameraStreamerComponentTest, InvalidConfigHandling)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "invalid_camera"},
                               {"gscam_config", "invalid_element_that_does_not_exist ! sink"},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);
  auto state = node->configure();
  // Configure should fail gracefully and not crash
  EXPECT_EQ(state.label(), "unconfigured");
}

TEST_F(CameraStreamerComponentTest, CameraInfoScalingWithOutputSize)
{
  // Validates intrinsic and projection matrix scaling when output_size is specified.
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "test_scaling_camera"},
                               {"gscam_config", "videotestsrc is-live=true ! valve name=gate drop=true ! video/x-raw,format=RGB,width=640,height=640,framerate=10/1"},
                               {"output_size", static_cast<int64_t>(640)},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);

  sensor_msgs::msg::CameraInfo orig_info;
  orig_info.width = 3280;
  orig_info.height = 2464;
  // K matrix: fx=2274.0, fy=2287.0, cx=1654.0, cy=1291.0
  orig_info.k = {2274.0, 0.0, 1654.0, 0.0, 2287.0, 1291.0, 0.0, 0.0, 1.0};
  // P matrix: fx=2298.0, fy=2303.0, cx=1650.0, cy=1277.0, Tx=10.0, Ty=20.0
  orig_info.p = {2298.0, 0.0, 1650.0, 10.0, 0.0, 2303.0, 1277.0, 20.0, 0.0, 0.0, 1.0, 0.0};

  const auto scaled = node->scale_camera_info(orig_info, 640, 640);

  EXPECT_EQ(scaled.width, 640U);
  EXPECT_EQ(scaled.height, 640U);

  const double expected_sx = 640.0 / 3280.0;
  const double expected_sy = 640.0 / 2464.0;

  EXPECT_NEAR(scaled.k[0], 2274.0 * expected_sx, 1e-4);
  EXPECT_NEAR(scaled.k[2], 1654.0 * expected_sx, 1e-4);
  EXPECT_NEAR(scaled.k[4], 2287.0 * expected_sy, 1e-4);
  EXPECT_NEAR(scaled.k[5], 1291.0 * expected_sy, 1e-4);

  EXPECT_NEAR(scaled.p[0], 2298.0 * expected_sx, 1e-4);
  EXPECT_NEAR(scaled.p[2], 1650.0 * expected_sx, 1e-4);
  EXPECT_NEAR(scaled.p[3], 10.0 * expected_sx, 1e-4);
  EXPECT_NEAR(scaled.p[5], 2303.0 * expected_sy, 1e-4);
  EXPECT_NEAR(scaled.p[6], 1277.0 * expected_sy, 1e-4);
  EXPECT_NEAR(scaled.p[7], 20.0 * expected_sy, 1e-4);
}

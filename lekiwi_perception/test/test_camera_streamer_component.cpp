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
                               {"active_modes", std::vector<int64_t>{0, 2}},
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

  // In STANDBY mode (0), active_modes [0, 2] allows streaming immediately without subscribers
  EXPECT_TRUE(node->is_streaming());
  EXPECT_TRUE(node->is_valve_open());

  auto helper_node = std::make_shared<rclcpp::Node>("test_helper_node");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper_node);

  auto mode_pub = helper_node->create_publisher<lekiwi_interfaces::msg::CameraMode>(
      "/camera_mode", rclcpp::QoS(1).reliable().transient_local());

  // Switch mode to NAVIGATING (1) -> should drop/gate since 1 is not in [0, 2]
  auto mode_msg = std::make_shared<lekiwi_interfaces::msg::CameraMode>();
  mode_msg->value = lekiwi_interfaces::msg::CameraMode::NAVIGATING;
  mode_pub->publish(*mode_msg);

  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_camera_mode(), lekiwi_interfaces::msg::CameraMode::NAVIGATING);
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  // Switch mode to CHESS_THINKING (2) -> should open since 2 is in [0, 2]
  mode_msg->value = lekiwi_interfaces::msg::CameraMode::CHESS_THINKING;
  mode_pub->publish(*mode_msg);

  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_camera_mode(), lekiwi_interfaces::msg::CameraMode::CHESS_THINKING);
  EXPECT_TRUE(node->is_streaming());
  EXPECT_TRUE(node->is_valve_open());

  // Switch mode to MANIPULATION_LEROBOT (3) -> should drop/gate
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

TEST_F(CameraStreamerComponentTest, Mode3CompressedPublisher)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "test_jpeg_camera"},
                               {"frame_id", "test_jpeg_optical"},
                               {"gscam_config", "videotestsrc is-live=true ! valve name=gate drop=true ! video/x-raw,format=I420,width=320,height=240,framerate=15/1 ! jpegenc quality=80 ! image/jpeg"},
                               {"active_modes", std::vector<int64_t>{3}},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);
  auto state = node->configure();
  ASSERT_EQ(state.label(), "inactive");
  state = node->activate();
  ASSERT_EQ(state.label(), "active");

  // In STANDBY mode (0), active_modes [3] will drop
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  auto helper_node = std::make_shared<rclcpp::Node>("test_jpeg_helper");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper_node);

  size_t compressed_count = 0;
  auto comp_sub = helper_node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", rclcpp::SensorDataQoS(),
      [&compressed_count](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
      {
        if (msg && msg->format == "jpeg" && !msg->data.empty())
        {
          compressed_count++;
        }
      });

  auto mode_pub = helper_node->create_publisher<lekiwi_interfaces::msg::CameraMode>(
      "/camera_mode", rclcpp::QoS(1).reliable().transient_local());

  auto mode_msg = std::make_shared<lekiwi_interfaces::msg::CameraMode>();
  mode_msg->value = lekiwi_interfaces::msg::CameraMode::MANIPULATION_LEROBOT;
  mode_pub->publish(*mode_msg);

  for (int i = 0; i < 20; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  EXPECT_EQ(node->current_camera_mode(), lekiwi_interfaces::msg::CameraMode::MANIPULATION_LEROBOT);
  EXPECT_TRUE(node->is_streaming());
  EXPECT_TRUE(node->is_valve_open());
  EXPECT_GT(compressed_count, 0U);

  node->deactivate();
  node->cleanup();
  node->shutdown();
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

TEST_F(CameraStreamerComponentTest, CameraInfoScalingWithOutputSizeAddBorder)
{
  // Validates intrinsic and projection matrix scaling when add_border is true (letterbox padding).
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "test_scaling_camera"},
                               {"gscam_config", "videotestsrc is-live=true ! valve name=gate drop=true ! video/x-raw,format=RGB,width=640,height=640,framerate=10/1"},
                               {"output_size", static_cast<int64_t>(640)},
                               {"add_border", true},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);
  node->configure();

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

  const double expected_s = 640.0 / 3280.0;
  const double active_h = 2464.0 * expected_s;
  const double expected_pad_y = (640.0 - active_h) / 2.0;

  EXPECT_NEAR(scaled.k[0], 2274.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.k[2], 1654.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.k[4], 2287.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.k[5], 1291.0 * expected_s + expected_pad_y, 1e-4);

  EXPECT_NEAR(scaled.p[0], 2298.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[2], 1650.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[3], 10.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[5], 2303.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[6], 1277.0 * expected_s + expected_pad_y, 1e-4);
  EXPECT_NEAR(scaled.p[7], 20.0 * expected_s, 1e-4);
}

TEST_F(CameraStreamerComponentTest, CameraInfoScalingWithOutputSizeCrop)
{
  // Validates intrinsic and projection matrix scaling when add_border is false (center crop).
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "test_scaling_camera_crop"},
                               {"gscam_config", "videotestsrc is-live=true ! valve name=gate drop=true ! video/x-raw,format=RGB,width=384,height=384,framerate=10/1"},
                               {"output_size", static_cast<int64_t>(384)},
                               {"add_border", false},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);
  node->configure();

  sensor_msgs::msg::CameraInfo orig_info;
  orig_info.width = 1640;
  orig_info.height = 1232;
  // K matrix: fx=1137.0, fy=1143.0, cx=827.0, cy=645.0
  orig_info.k = {1137.0, 0.0, 827.0, 0.0, 1143.0, 645.0, 0.0, 0.0, 1.0};
  orig_info.p = {1149.0, 0.0, 825.0, 5.0, 0.0, 1151.0, 638.0, 10.0, 0.0, 0.0, 1.0, 0.0};

  const auto scaled = node->scale_camera_info(orig_info, 384, 384);

  EXPECT_EQ(scaled.width, 384U);
  EXPECT_EQ(scaled.height, 384U);

  const double crop_x = (1640.0 - 1232.0) / 2.0; // 204.0
  const double expected_s = 384.0 / 1232.0;

  EXPECT_NEAR(scaled.k[0], 1137.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.k[2], (827.0 - crop_x) * expected_s, 1e-4);
  EXPECT_NEAR(scaled.k[4], 1143.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.k[5], 645.0 * expected_s, 1e-4);

  EXPECT_NEAR(scaled.p[0], 1149.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[2], (825.0 - crop_x) * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[3], 5.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[5], 1151.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[6], 638.0 * expected_s, 1e-4);
  EXPECT_NEAR(scaled.p[7], 10.0 * expected_s, 1e-4);
}

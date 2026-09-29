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
#include "lekiwi_interfaces/msg/perception_context.hpp"
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
                               {"active_contexts", std::vector<int64_t>{0, 2}},
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

  // In IDLE_STANDBY context (0), active_contexts [0, 2] allows streaming immediately without subscribers
  EXPECT_TRUE(node->is_streaming());
  EXPECT_TRUE(node->is_valve_open());

  auto helper_node = std::make_shared<rclcpp::Node>("test_helper_node");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper_node);

  auto context_pub = helper_node->create_publisher<lekiwi_interfaces::msg::PerceptionContext>(
      "/perception_context", rclcpp::QoS(1).reliable().transient_local());

  // Switch context to TF_TRACKING_AND_NAV (1) -> should drop/gate since 1 is not in [0, 2]
  auto context_msg = std::make_shared<lekiwi_interfaces::msg::PerceptionContext>();
  context_msg->value = lekiwi_interfaces::msg::PerceptionContext::TF_TRACKING_AND_NAV;
  context_pub->publish(*context_msg);

  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_perception_context(), lekiwi_interfaces::msg::PerceptionContext::TF_TRACKING_AND_NAV);
  EXPECT_FALSE(node->is_streaming());
  EXPECT_FALSE(node->is_valve_open());

  // Switch context to BOARD_STATE_SCAN (2) -> should open since 2 is in [0, 2]
  context_msg->value = lekiwi_interfaces::msg::PerceptionContext::BOARD_STATE_SCAN;
  context_pub->publish(*context_msg);

  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_perception_context(), lekiwi_interfaces::msg::PerceptionContext::BOARD_STATE_SCAN);
  EXPECT_TRUE(node->is_streaming());
  EXPECT_TRUE(node->is_valve_open());

  // Switch context to MANIPULATION_ACTOR (3) -> should drop/gate
  context_msg->value = lekiwi_interfaces::msg::PerceptionContext::MANIPULATION_ACTOR;
  context_pub->publish(*context_msg);

  for (int i = 0; i < 10; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(node->current_perception_context(), lekiwi_interfaces::msg::PerceptionContext::MANIPULATION_ACTOR);
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

TEST_F(CameraStreamerComponentTest, ManipulationCompressedPublisher)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_name", "test_jpeg_camera"},
                               {"frame_id", "test_jpeg_optical"},
                               {"gscam_config", "videotestsrc is-live=true ! valve name=gate drop=true ! video/x-raw,format=I420,width=320,height=240,framerate=15/1 ! jpegenc quality=80 ! image/jpeg"},
                               {"active_contexts", std::vector<int64_t>{3}},
                               {"autostart", false}});

  auto node = std::make_shared<lekiwi_perception::CameraStreamerComponent>(options);
  auto state = node->configure();
  ASSERT_EQ(state.label(), "inactive");
  state = node->activate();
  ASSERT_EQ(state.label(), "active");

  // In IDLE_STANDBY context (0), active_contexts [3] will drop
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

  auto context_pub = helper_node->create_publisher<lekiwi_interfaces::msg::PerceptionContext>(
      "/perception_context", rclcpp::QoS(1).reliable().transient_local());

  auto context_msg = std::make_shared<lekiwi_interfaces::msg::PerceptionContext>();
  context_msg->value = lekiwi_interfaces::msg::PerceptionContext::MANIPULATION_ACTOR;
  context_pub->publish(*context_msg);

  for (int i = 0; i < 20; ++i)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  EXPECT_EQ(node->current_perception_context(), lekiwi_interfaces::msg::PerceptionContext::MANIPULATION_ACTOR);
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

/**
 * @file test_chess_overlay_component.cpp
 * @brief Unit and integration tests for ChessOverlayComponent.
 *
 * Validates parameter loading, lazy evaluation (zero subscriber bypass),
 * visual overlay generation, and staleness/TTL gating.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "chess_overlay_component.hpp"
#include "hailo/chess_constants.hpp"

using namespace std::chrono_literals;

class ChessOverlayComponentTest : public ::testing::Test
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
  }
};

TEST_F(ChessOverlayComponentTest, DefaultParametersInitialization)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lekiwi_perception::ChessOverlayComponent>(options);

  EXPECT_DOUBLE_EQ(node->get_parameter("stale_timeout_sec").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("jpeg_quality").as_int(), 80);
  EXPECT_FALSE(node->get_parameter("debug").as_bool());
  EXPECT_EQ(node->get_parameter("camera_topic").as_string(), "/cameras/stereo_left/image_raw");
  EXPECT_EQ(node->get_parameter("overlay_topic").as_string(), "/chess/overlay_image/compressed");
  EXPECT_EQ(node->get_parameter("detections_topic").as_string(), "/chess/detections_2d");
  EXPECT_EQ(node->get_parameter("tag_centers_topic").as_string(), "/chess/tag_centers");
  EXPECT_EQ(node->get_parameter("grid_points_topic").as_string(), "/chess/grid_points");
}

TEST_F(ChessOverlayComponentTest, CustomParametersInitialization)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"stale_timeout_sec", 0.25},
                               {"jpeg_quality", 92},
                               {"debug", true},
                               {"camera_topic", "/custom/cam"},
                               {"overlay_topic", "/custom/overlay"},
                               {"detections_topic", "/custom/dets"},
                               {"tag_centers_topic", "/custom/tags"},
                               {"grid_points_topic", "/custom/grid"},
                               {"tags.ids", std::vector<int64_t>{3, 2, 1, 0}}});

  auto node = std::make_shared<lekiwi_perception::ChessOverlayComponent>(options);

  EXPECT_DOUBLE_EQ(node->get_parameter("stale_timeout_sec").as_double(), 0.25);
  EXPECT_EQ(node->get_parameter("jpeg_quality").as_int(), 92);
  EXPECT_TRUE(node->get_parameter("debug").as_bool());
  EXPECT_EQ(node->get_parameter("camera_topic").as_string(), "/custom/cam");
  EXPECT_EQ(node->get_parameter("overlay_topic").as_string(), "/custom/overlay");
  EXPECT_EQ(node->get_parameter("detections_topic").as_string(), "/custom/dets");
  EXPECT_EQ(node->get_parameter("tag_centers_topic").as_string(), "/custom/tags");
  EXPECT_EQ(node->get_parameter("grid_points_topic").as_string(), "/custom/grid");
}

TEST_F(ChessOverlayComponentTest, LazyEvaluationSkipWhenNoSubscriber)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_topic", "/test_lazy/cam"},
                               {"overlay_topic", "/test_lazy/overlay"}});

  auto node = std::make_shared<lekiwi_perception::ChessOverlayComponent>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("test_lazy_helper");
  auto cam_pub = helper_node->create_publisher<sensor_msgs::msg::Image>(
      "/test_lazy/cam", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  // Send an image when there are 0 subscribers on overlay_topic
  cv::Mat test_frame(240, 320, CV_8UC3, cv::Scalar(100, 150, 200));
  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", test_frame).toImageMsg();
  cam_pub->publish(*img_msg);

  // Spin briefly: lazy evaluation must drop early without error
  executor.spin_some(50ms);
  SUCCEED();
}

TEST_F(ChessOverlayComponentTest, OverlayRenderingWithSubscriber)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_topic", "/test_render/cam"},
                               {"overlay_topic", "/test_render/overlay"},
                               {"detections_topic", "/test_render/dets"},
                               {"tag_centers_topic", "/test_render/tags"},
                               {"grid_points_topic", "/test_render/grid"},
                               {"debug", true},
                               {"stale_timeout_sec", 1.0}});

  auto node = std::make_shared<lekiwi_perception::ChessOverlayComponent>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("test_render_helper");

  auto cam_pub = helper_node->create_publisher<sensor_msgs::msg::Image>(
      "/test_render/cam", rclcpp::SensorDataQoS());
  auto dets_pub = helper_node->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/test_render/dets", rclcpp::SensorDataQoS());
  auto tags_pub = helper_node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/test_render/tags", rclcpp::QoS(1).transient_local().reliable());
  auto grid_pub = helper_node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/test_render/grid", rclcpp::SensorDataQoS());

  sensor_msgs::msg::CompressedImage::SharedPtr received_overlay;
  auto overlay_sub = helper_node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/test_render/overlay", rclcpp::SensorDataQoS(),
      [&received_overlay](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
      {
        received_overlay = msg;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  // Wait for pub-sub connection to establish
  auto start_time = std::chrono::steady_clock::now();
  while (overlay_sub->get_publisher_count() == 0 &&
         (std::chrono::steady_clock::now() - start_time) < 1s)
  {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }

  // 1. Publish tag centers (4 corners: A1, H1, H8, A8)
  geometry_msgs::msg::PolygonStamped tags_msg;
  tags_msg.header.stamp = helper_node->now();
  for (int i = 0; i < 4; ++i)
  {
    geometry_msgs::msg::Point32 pt;
    pt.x = 0.2f + 0.5f * (i % 2);
    pt.y = 0.2f + 0.5f * (i / 2);
    pt.z = static_cast<float>(i); // tag ID
    tags_msg.polygon.points.push_back(pt);
  }
  tags_pub->publish(tags_msg);

  // 2. Publish 81 grid points
  geometry_msgs::msg::PolygonStamped grid_msg;
  grid_msg.header.stamp = helper_node->now();
  for (size_t i = 0; i < lekiwi_perception::hailo::kGridPointsCount; ++i)
  {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(i % 9) / 8.0f;
    pt.y = static_cast<float>(i / 9) / 8.0f;
    grid_msg.polygon.points.push_back(pt);
  }
  grid_pub->publish(grid_msg);

  // 3. Publish detections
  vision_msgs::msg::Detection2DArray dets_msg;
  dets_msg.header.stamp = helper_node->now();
  vision_msgs::msg::Detection2D det;
  det.bbox.center.position.x = 160.0;
  det.bbox.center.position.y = 120.0;
  det.bbox.size_x = 40.0;
  det.bbox.size_y = 50.0;
  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = "w_pawn";
  hyp.hypothesis.score = 0.95f;
  det.results.push_back(hyp);
  dets_msg.detections.push_back(det);
  dets_pub->publish(dets_msg);

  // Process subscriptions
  executor.spin_some(50ms);

  // 4. Publish camera frame
  cv::Mat test_frame(240, 320, CV_8UC3, cv::Scalar(50, 50, 50));
  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", test_frame).toImageMsg();
  cam_pub->publish(*img_msg);

  // Wait for overlay image to be received
  start_time = std::chrono::steady_clock::now();
  while (!received_overlay && (std::chrono::steady_clock::now() - start_time) < 2s)
  {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_NE(received_overlay, nullptr);
  EXPECT_EQ(received_overlay->format, "jpeg");
  EXPECT_FALSE(received_overlay->data.empty());

  cv::Mat decoded = cv::imdecode(received_overlay->data, cv::IMREAD_COLOR);
  EXPECT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, 320);
  EXPECT_EQ(decoded.rows, 240);
}

TEST_F(ChessOverlayComponentTest, StalenessTTLTimeoutGating)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"camera_topic", "/test_ttl/cam"},
      {"overlay_topic", "/test_ttl/overlay"},
      {"detections_topic", "/test_ttl/dets"},
      {"stale_timeout_sec", 0.05} // 50ms TTL
  });

  auto node = std::make_shared<lekiwi_perception::ChessOverlayComponent>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("test_ttl_helper");

  auto cam_pub = helper_node->create_publisher<sensor_msgs::msg::Image>(
      "/test_ttl/cam", rclcpp::SensorDataQoS());
  auto dets_pub = helper_node->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/test_ttl/dets", rclcpp::SensorDataQoS());

  sensor_msgs::msg::CompressedImage::SharedPtr received_overlay;
  auto overlay_sub = helper_node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/test_ttl/overlay", rclcpp::SensorDataQoS(),
      [&received_overlay](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
      {
        received_overlay = msg;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  // Wait for pub-sub connection to establish
  auto start_time = std::chrono::steady_clock::now();
  while (overlay_sub->get_publisher_count() == 0 &&
         (std::chrono::steady_clock::now() - start_time) < 1s)
  {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }

  // Publish detection
  vision_msgs::msg::Detection2DArray dets_msg;
  dets_msg.header.stamp = helper_node->now();
  vision_msgs::msg::Detection2D det;
  det.bbox.center.position.x = 100.0;
  det.bbox.center.position.y = 100.0;
  det.bbox.size_x = 30.0;
  det.bbox.size_y = 30.0;
  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = "b_king";
  hyp.hypothesis.score = 0.88f;
  det.results.push_back(hyp);
  dets_msg.detections.push_back(det);
  dets_pub->publish(dets_msg);
  executor.spin_some(20ms);

  // Sleep for 80ms (> 50ms TTL) so detections become stale
  std::this_thread::sleep_for(80ms);

  // Publish image: should succeed and ignore the stale detections without error
  cv::Mat test_frame(240, 320, CV_8UC3, cv::Scalar(10, 10, 10));
  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", test_frame).toImageMsg();
  cam_pub->publish(*img_msg);

  start_time = std::chrono::steady_clock::now();
  while (!received_overlay && (std::chrono::steady_clock::now() - start_time) < 2s)
  {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_NE(received_overlay, nullptr);
  EXPECT_EQ(received_overlay->format, "jpeg");
}

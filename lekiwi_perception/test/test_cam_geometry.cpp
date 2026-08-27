// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "camera_info_manager/camera_info_manager.hpp"
#include "gstreamer/pipe_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

lekiwi_perception::CamStreamConfig make_config(int rotation)
{
  lekiwi_perception::CamStreamConfig config;
  config.capture_width = 1280;
  config.capture_height = 720;
  config.output_width = 640;
  config.output_height = 480;
  config.rotation = rotation;
  return config;
}

}  // namespace

TEST(CameraGeometry, LetterboxesWithoutCropping)
{
  const auto plan = lekiwi_perception::make_geom_plan(make_config(0), false);
  EXPECT_EQ(plan.active_width, 640U);
  EXPECT_EQ(plan.active_height, 360U);
  EXPECT_EQ(plan.pad_x, 0U);
  EXPECT_EQ(plan.pad_y, 60U);
  EXPECT_EQ(plan.pre_rotation_pad_x, 0U);
  EXPECT_EQ(plan.pre_rotation_pad_y, 60U);
}

TEST(CameraGeometry, SupportsAllQuarterTurns)
{
  for (const int rotation : {0, 90, 180, 270}) {
    const auto config = make_config(rotation);
    const auto plan = lekiwi_perception::make_geom_plan(config, false);
    const uint32_t output_width = (rotation == 90 || rotation == 270) ?
      plan.pre_rotation_height : plan.pre_rotation_width;
    const uint32_t output_height = (rotation == 90 || rotation == 270) ?
      plan.pre_rotation_width : plan.pre_rotation_height;
    EXPECT_EQ(output_width, config.output_width) << rotation;
    EXPECT_EQ(output_height, config.output_height) << rotation;
    EXPECT_LE(plan.pad_x + plan.active_width, output_width) << rotation;
    EXPECT_LE(plan.pad_y + plan.active_height, output_height) << rotation;
  }
}

TEST(CameraGeometry, LoadsCalibrationFixture)
{
  const bool owns_rclcpp = !rclcpp::ok();
  if (owns_rclcpp) {
    rclcpp::init(0, nullptr);
  }
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("camera_info_manager_test");
    const std::string url = "file://" +
      std::string(LEKIWI_PERCEPTION_TEST_RESOURCE_DIR) + "/stereo_right.yaml";
    camera_info_manager::CameraInfoManager manager(node.get(), "stereo_right", url);
    ASSERT_TRUE(manager.loadCameraInfo(url));
    const auto calibration = manager.getCameraInfo();
    EXPECT_EQ(calibration.width, 640U);
    EXPECT_EQ(calibration.height, 480U);
    EXPECT_GT(calibration.k[0], 0.0);
  }
  if (owns_rclcpp) {
    rclcpp::shutdown();
  }
}

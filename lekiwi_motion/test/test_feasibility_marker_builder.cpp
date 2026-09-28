// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "lekiwi_motion/feasibility_marker_builder.hpp"

using namespace lekiwi_motion::visualization;
using FeasibilityResponse = lekiwi_interfaces::srv::CheckMoveFeasibility::Response;

TEST(TestFeasibilityMarkerBuilder, RejectPlanBuildsHudWithError)
{
  FeasibilityMarkerBuilder builder("map", "chessboard_frame");

  FeasibilityResponse resp;
  resp.feasible = false;
  resp.message = "Target square out of reach";

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  stamp.nanosec = 0;
  auto markers = builder.build(resp, "e2e4", stamp);

  ASSERT_GE(markers.markers.size(), 2u);

  // Marker 0 must be DELETEALL
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);

  // Marker 1 is HUD Status
  const auto &hud = markers.markers[1];
  EXPECT_EQ(hud.ns, "feasibility/hud_status");
  EXPECT_EQ(hud.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_FLOAT_EQ(hud.color.r, 1.0f); // Red alert
  EXPECT_NE(hud.text.find("[NOT FEASIBLE]"), std::string::npos);
  EXPECT_NE(hud.text.find("Target square out of reach"), std::string::npos);

  // When rejected, no footprint or target sphere markers are created
  for (size_t i = 2; i < markers.markers.size(); ++i)
  {
    EXPECT_NE(markers.markers[i].ns, "feasibility/target_points");
    EXPECT_NE(markers.markers[i].ns, "feasibility/base_footprints");
  }
}

TEST(TestFeasibilityMarkerBuilder, SingleBaseFeasiblePlan)
{
  FeasibilityMarkerBuilder builder("map", "chessboard_frame");

  FeasibilityResponse resp;
  resp.feasible = true;
  resp.plan_type = FeasibilityResponse::PLAN_SINGLE_BASE;
  resp.message = "Single standoff found";

  resp.pick_point.x = 0.10;
  resp.pick_point.y = 0.10;
  resp.pick_point.z = 0.025;

  resp.place_point.x = 0.10;
  resp.place_point.y = 0.20;
  resp.place_point.z = 0.025;

  resp.pick_base_pose.header.frame_id = "map";
  resp.pick_base_pose.pose.position.x = 0.50;
  resp.pick_base_pose.pose.position.y = 0.50;
  resp.pick_base_pose.pose.orientation.w = 1.0;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 200;
  stamp.nanosec = 0;
  auto markers = builder.build(resp, "e2e4", stamp);

  // First must be DELETEALL
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);

  // HUD should be green
  EXPECT_EQ(markers.markers[1].ns, "feasibility/hud_status");
  EXPECT_FLOAT_EQ(markers.markers[1].color.g, 1.0f);
  EXPECT_NE(markers.markers[1].text.find("[FEASIBLE]"), std::string::npos);

  // Find target points
  bool found_pick = false;
  bool found_place = false;
  bool found_base_disk = false;
  bool found_reach_ring = false;

  for (const auto &m : markers.markers)
  {
    if (m.ns == "feasibility/target_points" && m.id == 10 && m.type == visualization_msgs::msg::Marker::SPHERE)
    {
      found_pick = true;
      EXPECT_EQ(m.header.frame_id, "chessboard_frame");
      EXPECT_DOUBLE_EQ(m.pose.position.x, 0.10);
    }
    if (m.ns == "feasibility/target_points" && m.id == 11 && m.type == visualization_msgs::msg::Marker::SPHERE)
    {
      found_place = true;
      EXPECT_EQ(m.header.frame_id, "chessboard_frame");
      EXPECT_DOUBLE_EQ(m.pose.position.y, 0.20);
    }
    if (m.ns == "feasibility/base_footprints" && m.id == 20 && m.type == visualization_msgs::msg::Marker::CYLINDER)
    {
      found_base_disk = true;
      EXPECT_EQ(m.header.frame_id, "map");
      EXPECT_DOUBLE_EQ(m.scale.x, 0.30); // 30cm chassis diameter
    }
    if (m.ns == "feasibility/reach_envelope" && m.id == 30)
    {
      found_reach_ring = true;
      EXPECT_EQ(m.header.frame_id, "map");
      EXPECT_FALSE(m.points.empty());
    }
  }

  EXPECT_TRUE(found_pick);
  EXPECT_TRUE(found_place);
  EXPECT_TRUE(found_base_disk);
  EXPECT_TRUE(found_reach_ring);
}

TEST(TestFeasibilityMarkerBuilder, CaptureTripleBasePlan)
{
  FeasibilityMarkerBuilder builder("map", "chessboard_frame");

  FeasibilityResponse resp;
  resp.feasible = true;
  resp.plan_type = FeasibilityResponse::PLAN_CAPTURE_TRIPLE_BASE;
  resp.message = "3 Standoff bases required";

  resp.clear_point.x = 0.15;
  resp.clear_point.y = 0.15;
  resp.clear_point.z = 0.025;

  resp.pick_point.x = 0.10;
  resp.pick_point.y = 0.10;
  resp.pick_point.z = 0.025;

  resp.place_point.x = 0.20;
  resp.place_point.y = 0.20;
  resp.place_point.z = 0.025;

  resp.clear_base_pose.header.frame_id = "map";
  resp.clear_base_pose.pose.position.x = 0.40;
  resp.pick_base_pose.header.frame_id = "map";
  resp.pick_base_pose.pose.position.x = 0.50;
  resp.place_base_pose.header.frame_id = "map";
  resp.place_base_pose.pose.position.x = 0.60;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 300;
  stamp.nanosec = 0;
  auto markers = builder.build(resp, "e4d5", stamp);

  bool found_clear_target = false;
  bool found_clear_base = false;
  bool found_pick_base = false;
  bool found_place_base = false;
  bool found_nav_trajectory = false;

  for (const auto &m : markers.markers)
  {
    if (m.ns == "feasibility/target_points" && m.id == 12)
    {
      found_clear_target = true;
      EXPECT_FLOAT_EQ(m.color.r, 1.0f); // Red
    }
    if (m.ns == "feasibility/base_footprints" && m.id == 23)
    {
      found_clear_base = true;
    }
    if (m.ns == "feasibility/base_footprints" && m.id == 24)
    {
      found_pick_base = true;
    }
    if (m.ns == "feasibility/base_footprints" && m.id == 25)
    {
      found_place_base = true;
    }
    if (m.ns == "feasibility/nav_trajectory" && m.id == 50)
    {
      found_nav_trajectory = true;
      EXPECT_EQ(m.points.size(), 3u); // 3 sequential waypoints
    }
  }

  EXPECT_TRUE(found_clear_target);
  EXPECT_TRUE(found_clear_base);
  EXPECT_TRUE(found_pick_base);
  EXPECT_TRUE(found_place_base);
  EXPECT_TRUE(found_nav_trajectory);
}

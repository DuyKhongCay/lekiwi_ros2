// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file feasibility_marker_builder.cpp
 * @brief Implementation of ROS 2 MarkerArray builder for motion feasibility visualization.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/kinematics/feasibility_marker_builder.hpp"

#include <cmath>
#include <utility>

namespace lekiwi_motion::visualization
{

  FeasibilityMarkerBuilder::FeasibilityMarkerBuilder(
      std::string map_frame,
      std::string board_frame,
      FeasibilityVisualConfig config)
      : map_frame_(std::move(map_frame)),
        board_frame_(std::move(board_frame)),
        config_(config)
  {
  }

  visualization_msgs::msg::MarkerArray FeasibilityMarkerBuilder::build(
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
      const std::string &move_uci,
      const builtin_interfaces::msg::Time &stamp) const
  {
    visualization_msgs::msg::MarkerArray markers;

    // 0. Always start with a DELETEALL marker to prevent stale remnants across plan type changes
    visualization_msgs::msg::Marker clear_all;
    clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_all.header.stamp = stamp;
    clear_all.header.frame_id = map_frame_;
    markers.markers.push_back(clear_all);

    // 1. Layer 5: HUD Status Billboard (3D text floating above chessboard)
    append_hud(markers, resp, move_uci, stamp);

    // If feasibility check failed completely, return HUD only with error diagnosis
    if (!resp.feasible)
    {
      return markers;
    }

    // 2. Layer 1: Target Points in chessboard_frame (Pick, Place, Clear + approach rays)
    append_target_points(markers, resp, stamp);

    // 3. Layer 2: Base Footprints & Heading Arrows in map frame
    append_base_footprints(markers, resp, stamp);

    // 4. Layer 3: Kinematic Reach Envelopes (SO-101 working radii)
    append_reach_envelopes(markers, resp, stamp);

    // 5. Layer 4: Multi-base Navigation Trajectory & Sequence Waypoint Labels
    append_nav_trajectory(markers, resp, stamp);

    return markers;
  }

  void FeasibilityMarkerBuilder::append_hud(
      visualization_msgs::msg::MarkerArray &out,
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
      const std::string &move_uci,
      const builtin_interfaces::msg::Time &stamp) const
  {
    visualization_msgs::msg::Marker hud;
    hud.header.frame_id = board_frame_;
    hud.header.stamp = stamp;
    hud.ns = "feasibility/hud_status";
    hud.id = 0;
    hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    hud.action = visualization_msgs::msg::Marker::ADD;
    hud.pose.position.x = 0.0;
    hud.pose.position.y = 0.25;
    hud.pose.position.z = config_.hud_z_offset;
    hud.pose.orientation.w = 1.0;
    hud.scale.z = 0.018; // 18mm font height in 3D scene

    if (resp.feasible)
    {
      // Neon green for accepted feasibility
      hud.color.r = 0.1f;
      hud.color.g = 1.0f;
      hud.color.b = 0.2f;
      hud.color.a = 1.0f;
      hud.text = "[FEASIBLE] Move: " + move_uci + "\nPlan Type: " + std::to_string(resp.plan_type) + "\n" + resp.message;
    }
    else
    {
      // Vivid alert red for rejection
      hud.color.r = 1.0f;
      hud.color.g = 0.1f;
      hud.color.b = 0.1f;
      hud.color.a = 1.0f;
      hud.text = "[NOT FEASIBLE] Move: " + move_uci + "\nReason: " + resp.message;
    }

    out.markers.push_back(std::move(hud));
  }

  void FeasibilityMarkerBuilder::append_target_points(
      visualization_msgs::msg::MarkerArray &out,
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
      const builtin_interfaces::msg::Time &stamp) const
  {
    auto add_target_marker = [&](int id, const geometry_msgs::msg::Point &pt,
                                 float r, float g, float b, float radius,
                                 const std::string &label)
    {
      // 1. Target Sphere
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = board_frame_;
      sphere.header.stamp = stamp;
      sphere.ns = "feasibility/target_points";
      sphere.id = id;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = pt;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = radius * 2.0;
      sphere.color.r = r;
      sphere.color.g = g;
      sphere.color.b = b;
      sphere.color.a = 0.90f;
      out.markers.push_back(std::move(sphere));

      // 2. Approach Vector Line (Vertical approach along Z)
      visualization_msgs::msg::Marker ray;
      ray.header.frame_id = board_frame_;
      ray.header.stamp = stamp;
      ray.ns = "feasibility/target_points";
      ray.id = id + 1000;
      ray.type = visualization_msgs::msg::Marker::LINE_LIST;
      ray.action = visualization_msgs::msg::Marker::ADD;
      ray.pose.orientation.w = 1.0;
      ray.scale.x = 0.003; // Line width: 3mm
      ray.color.r = r;
      ray.color.g = g;
      ray.color.b = b;
      ray.color.a = 0.75f;

      geometry_msgs::msg::Point top_pt = pt;
      top_pt.z += config_.approach_ray_height;
      ray.points.push_back(pt);
      ray.points.push_back(top_pt);
      out.markers.push_back(std::move(ray));

      // 3. Text label floating slightly above approach ray
      visualization_msgs::msg::Marker text;
      text.header.frame_id = board_frame_;
      text.header.stamp = stamp;
      text.ns = "feasibility/target_points";
      text.id = id + 2000;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = top_pt;
      text.pose.position.z += 0.01;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.012;
      text.color.r = r;
      text.color.g = g;
      text.color.b = b;
      text.color.a = 0.95f;
      text.text = label;
      out.markers.push_back(std::move(text));
    };

    // Pick Point: Amber / Golden Yellow
    add_target_marker(10, resp.pick_point, 1.0f, 0.8f, 0.0f, 0.012f, "Pick");

    // Place Point: Cyan / Blue-Green
    add_target_marker(11, resp.place_point, 0.0f, 0.85f, 1.0f, 0.012f, "Place");

    // Clear Point (Capture moves: plan_type >= 3): Bright Red
    if (resp.plan_type >= lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_ZERO_NAV)
    {
      add_target_marker(12, resp.clear_point, 1.0f, 0.1f, 0.1f, 0.015f, "Clear (Capture)");
    }
  }

  void FeasibilityMarkerBuilder::append_single_base_markers(
      visualization_msgs::msg::MarkerArray &out,
      int id,
      const geometry_msgs::msg::PoseStamped &pose,
      float r, float g, float b,
      const std::string &label,
      const builtin_interfaces::msg::Time &stamp) const
  {
    // 1. Circular chassis footprint cylinder disc
    visualization_msgs::msg::Marker disk;
    disk.header = pose.header;
    if (disk.header.frame_id.empty())
    {
      disk.header.frame_id = map_frame_;
    }
    disk.header.stamp = stamp;
    disk.ns = "feasibility/base_footprints";
    disk.id = id;
    disk.type = visualization_msgs::msg::Marker::CYLINDER;
    disk.action = visualization_msgs::msg::Marker::ADD;
    disk.pose = pose.pose;
    disk.pose.position.z = config_.footprint_thickness * 0.5;
    disk.scale.x = disk.scale.y = config_.chassis_radius * 2.0;
    disk.scale.z = config_.footprint_thickness;
    disk.color.r = r;
    disk.color.g = g;
    disk.color.b = b;
    disk.color.a = 0.45f;
    out.markers.push_back(std::move(disk));

    // 2. Heading Arrow along vehicle yaw orientation
    visualization_msgs::msg::Marker arrow;
    arrow.header = pose.header;
    if (arrow.header.frame_id.empty())
    {
      arrow.header.frame_id = map_frame_;
    }
    arrow.header.stamp = stamp;
    arrow.ns = "feasibility/base_footprints";
    arrow.id = id + 100;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = pose.pose;
    arrow.scale.x = 0.20;
    arrow.scale.y = 0.025;
    arrow.scale.z = 0.025;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 0.90f;
    out.markers.push_back(std::move(arrow));

    // 3. Standoff Base Label
    visualization_msgs::msg::Marker text;
    text.header = pose.header;
    if (text.header.frame_id.empty())
    {
      text.header.frame_id = map_frame_;
    }
    text.header.stamp = stamp;
    text.ns = "feasibility/base_footprints";
    text.id = id + 200;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = pose.pose;
    text.pose.position.z += 0.06;
    text.scale.z = 0.014;
    text.color.r = r;
    text.color.g = g;
    text.color.b = b;
    text.color.a = 0.95f;
    text.text = label;
    out.markers.push_back(std::move(text));
  }

  void FeasibilityMarkerBuilder::append_base_footprints(
      visualization_msgs::msg::MarkerArray &out,
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
      const builtin_interfaces::msg::Time &stamp) const
  {
    switch (resp.plan_type)
    {
    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_ZERO_NAV:
    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_ZERO_NAV:
      break;

    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_SINGLE_BASE:
    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_SINGLE_BASE:
      append_single_base_markers(out, 20, resp.pick_base_pose, 0.2f, 1.0f, 0.3f, "Single Standoff", stamp);
      break;

    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_DUAL_BASE:
    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_DUAL_BASE:
      append_single_base_markers(out, 21, resp.pick_base_pose, 1.0f, 0.8f, 0.0f, "Pick Base", stamp);
      append_single_base_markers(out, 22, resp.place_base_pose, 0.0f, 0.8f, 1.0f, "Place Base", stamp);
      break;

    case lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_TRIPLE_BASE:
      append_single_base_markers(out, 23, resp.clear_base_pose, 1.0f, 0.1f, 0.1f, "Clear Base", stamp);
      append_single_base_markers(out, 24, resp.pick_base_pose, 1.0f, 0.8f, 0.0f, "Pick Base", stamp);
      append_single_base_markers(out, 25, resp.place_base_pose, 0.0f, 0.8f, 1.0f, "Place Base", stamp);
      break;

    default:
      break;
    }
  }

  void FeasibilityMarkerBuilder::append_reach_envelopes(
      visualization_msgs::msg::MarkerArray &out,
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
      const builtin_interfaces::msg::Time &stamp) const
  {
    auto add_reach_ring = [&](int id, const geometry_msgs::msg::PoseStamped &pose,
                              double radius, float r, float g, float b)
    {
      visualization_msgs::msg::Marker ring;
      ring.header = pose.header;
      if (ring.header.frame_id.empty())
      {
        ring.header.frame_id = map_frame_;
      }
      ring.header.stamp = stamp;
      ring.ns = "feasibility/reach_envelope";
      ring.id = id;
      ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ring.action = visualization_msgs::msg::Marker::ADD;
      ring.pose.position.z = 0.005; // Slightly elevated
      ring.pose.orientation.w = 1.0;
      ring.scale.x = 0.002; // Thin ring: 2mm
      ring.color.r = r;
      ring.color.g = g;
      ring.color.b = b;
      ring.color.a = 0.60f;

      constexpr int SAMPLES = 36;
      ring.points.reserve(SAMPLES + 1);
      for (int i = 0; i <= SAMPLES; ++i)
      {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(SAMPLES);
        geometry_msgs::msg::Point pt;
        pt.x = pose.pose.position.x + radius * std::cos(angle);
        pt.y = pose.pose.position.y + radius * std::sin(angle);
        pt.z = 0.005;
        ring.points.push_back(pt);
      }
      out.markers.push_back(std::move(ring));
    };

    int ring_id = 30;
    if (resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_SINGLE_BASE ||
        resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_SINGLE_BASE)
    {
      add_reach_ring(ring_id++, resp.pick_base_pose, config_.reach_max, 0.2f, 1.0f, 0.3f);
    }
    else if (resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_DUAL_BASE ||
             resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_DUAL_BASE)
    {
      add_reach_ring(ring_id++, resp.pick_base_pose, config_.reach_max, 1.0f, 0.8f, 0.0f);
      add_reach_ring(ring_id++, resp.place_base_pose, config_.reach_max, 0.0f, 0.8f, 1.0f);
    }
    else if (resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_TRIPLE_BASE)
    {
      add_reach_ring(ring_id++, resp.clear_base_pose, config_.reach_max, 1.0f, 0.1f, 0.1f);
      add_reach_ring(ring_id++, resp.pick_base_pose, config_.reach_max, 1.0f, 0.8f, 0.0f);
      add_reach_ring(ring_id++, resp.place_base_pose, config_.reach_max, 0.0f, 0.8f, 1.0f);
    }
  }

  void FeasibilityMarkerBuilder::append_nav_trajectory(
      visualization_msgs::msg::MarkerArray &out,
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
      const builtin_interfaces::msg::Time &stamp) const
  {
    std::vector<std::pair<geometry_msgs::msg::Point, std::string>> waypoints;

    if (resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_SINGLE_BASE ||
        resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_SINGLE_BASE)
    {
      waypoints.push_back({resp.pick_base_pose.pose.position, "Standoff"});
    }
    else if (resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_DUAL_BASE ||
             resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_DUAL_BASE)
    {
      waypoints.push_back({resp.pick_base_pose.pose.position, "Step 1: Pick Base"});
      waypoints.push_back({resp.place_base_pose.pose.position, "Step 2: Place Base"});
    }
    else if (resp.plan_type == lekiwi_interfaces::srv::CheckMoveFeasibility::Response::PLAN_CAPTURE_TRIPLE_BASE)
    {
      waypoints.push_back({resp.clear_base_pose.pose.position, "Step 1: Clear Base"});
      waypoints.push_back({resp.pick_base_pose.pose.position, "Step 2: Pick Base"});
      waypoints.push_back({resp.place_base_pose.pose.position, "Step 3: Place Base"});
    }

    if (waypoints.size() < 2)
    {
      return;
    }

    // Nav Path line strip linking sequential bases
    visualization_msgs::msg::Marker path;
    path.header.frame_id = map_frame_;
    path.header.stamp = stamp;
    path.ns = "feasibility/nav_trajectory";
    path.id = 50;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.006; // Line width: 6mm
    path.color.r = 1.0f;
    path.color.g = 0.85f;
    path.color.b = 0.2f;
    path.color.a = 0.85f;

    for (const auto &wp : waypoints)
    {
      geometry_msgs::msg::Point pt = wp.first;
      pt.z = 0.01;
      path.points.push_back(pt);
    }
    out.markers.push_back(std::move(path));
  }

} // namespace lekiwi_motion::visualization

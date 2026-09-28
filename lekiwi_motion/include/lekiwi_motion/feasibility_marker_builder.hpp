// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_MOTION__FEASIBILITY_MARKER_BUILDER_HPP_
#define LEKIWI_MOTION__FEASIBILITY_MARKER_BUILDER_HPP_

#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace lekiwi_motion::visualization
{

  /**
   * @brief Configuration parameters for feasibility visualization layers.
   */
  struct FeasibilityVisualConfig
  {
    double chassis_radius{0.15};       // Robot chassis radius in meters (LeKiwi diameter: 0.30m)
    double footprint_thickness{0.002}; // Thickness of footprint cylinder disc in meters
    double reach_min{0.10};            // Min working envelope of arm (m)
    double reach_max{0.28};            // Max working reach of SO-101 arm (m)
    double approach_ray_height{0.05};  // Vertical approach line height (m)
    double hud_z_offset{0.15};         // HUD 3D billboard height above board (m)
  };

  /**
   * @brief Pure builder class producing standardized ROS 2 MarkerArray
   *        for CheckMoveFeasibility responses across 5 distinct visual layers.
   *
   * Designed according to SRP (Single Responsibility Principle) and fully
   * decoupled from ROS node/TF context for high portability and unit-testability.
   */
  class FeasibilityMarkerBuilder
  {
  public:
    explicit FeasibilityMarkerBuilder(
        std::string map_frame = "map",
        std::string board_frame = "chessboard_frame",
        FeasibilityVisualConfig config = {});

    /**
     * @brief Build a MarkerArray representing the feasibility plan.
     * @param resp Service response containing feasibility status, plan_type, poses, and points.
     * @param move_uci Micro move notation (e.g., "e2e4").
     * @param stamp Timestamp to apply across generated markers.
     * @return Completed visualization_msgs::msg::MarkerArray.
     */
    [[nodiscard]] visualization_msgs::msg::MarkerArray build(
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const std::string &move_uci,
        const builtin_interfaces::msg::Time &stamp) const;

    [[nodiscard]] const std::string &map_frame() const noexcept { return map_frame_; }
    [[nodiscard]] const std::string &board_frame() const noexcept { return board_frame_; }
    [[nodiscard]] const FeasibilityVisualConfig &config() const noexcept { return config_; }

    void set_frames(std::string map_frame, std::string board_frame) noexcept
    {
      map_frame_ = std::move(map_frame);
      board_frame_ = std::move(board_frame);
    }

  private:
    void append_hud(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const std::string &move_uci,
        const builtin_interfaces::msg::Time &stamp) const;

    void append_target_points(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    void append_base_footprints(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    void append_reach_envelopes(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    void append_nav_trajectory(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    std::string map_frame_;
    std::string board_frame_;
    FeasibilityVisualConfig config_;
  };

} // namespace lekiwi_motion::visualization

#endif // LEKIWI_MOTION__FEASIBILITY_MARKER_BUILDER_HPP_

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
 * @file feasibility_marker_builder.hpp
 * @brief Visualizer generating ROS 2 MarkerArray layers for workspace feasibility plans.
 * @details Generates layered 3D RViz markers including HUD status billboards,
 *          target points with approach vectors, robot base footprints, reach envelopes,
 *          and navigation trajectories.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

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
   * @struct FeasibilityVisualConfig
   * @brief Visual layout and dimension parameters for RViz markers.
   */
  struct FeasibilityVisualConfig
  {
    double chassis_radius{0.15};       ///< Robot chassis radius in meters (LeKiwi diameter: 0.30m).
    double footprint_thickness{0.002}; ///< Thickness of footprint cylinder disc in meters.
    double reach_min{0.10};            ///< Min working envelope of arm (m).
    double reach_max{0.28};            ///< Max working reach of SO-101 arm (m).
    double approach_ray_height{0.05};  ///< Vertical approach line height (m).
    double hud_z_offset{0.15};         ///< HUD 3D billboard height above board (m).
  };

  /**
   * @class FeasibilityMarkerBuilder
   * @brief Pure builder class producing standardized ROS 2 MarkerArray across 5 visual layers.
   * @details Decoupled from ROS node/TF context for high portability and unit-testability:
   *          - Layer 1: Target manipulation points (Pick, Place, Clear) with approach rays.
   *          - Layer 2: Robot chassis footprints and heading arrows.
   *          - Layer 3: Kinematic arm reach envelopes.
   *          - Layer 4: Multi-base navigation trajectory lines.
   *          - Layer 5: HUD 3D billboard floating over chessboard.
   */
  class FeasibilityMarkerBuilder
  {
  public:
    /**
     * @brief Constructs builder with frame IDs and visual dimensions.
     * @param[in] map_frame Global map frame ID.
     * @param[in] board_frame Chessboard target frame ID.
     * @param[in] config Visual sizing parameters.
     */
    explicit FeasibilityMarkerBuilder(
        std::string map_frame = "map",
        std::string board_frame = "chessboard_frame",
        FeasibilityVisualConfig config = {});

    /**
     * @brief Builds a MarkerArray representing the complete feasibility plan.
     * @param[in] resp Feasibility service response containing status, plan type, and poses.
     * @param[in] move_uci UCI move notation string (e.g. "e2e4").
     * @param[in] stamp Timestamp applied to all generated markers.
     * @return Completed visualization_msgs::msg::MarkerArray.
     */
    [[nodiscard]] visualization_msgs::msg::MarkerArray build(
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const std::string &move_uci,
        const builtin_interfaces::msg::Time &stamp) const;

    /**
     * @brief Retrieves active map frame ID.
     * @return Const reference to map frame string.
     */
    [[nodiscard]] const std::string &map_frame() const noexcept { return map_frame_; }

    /**
     * @brief Retrieves active board frame ID.
     * @return Const reference to board frame string.
     */
    [[nodiscard]] const std::string &board_frame() const noexcept { return board_frame_; }

    /**
     * @brief Retrieves visual configuration parameters.
     * @return Const reference to configuration struct.
     */
    [[nodiscard]] const FeasibilityVisualConfig &config() const noexcept { return config_; }

    /**
     * @brief Updates coordinate frame identifiers.
     * @param[in] map_frame Global map frame ID.
     * @param[in] board_frame Chessboard frame ID.
     */
    void set_frames(std::string map_frame, std::string board_frame) noexcept
    {
      map_frame_ = std::move(map_frame);
      board_frame_ = std::move(board_frame);
    }

  private:
    /**
     * @brief Appends 3D text HUD billboard over the board displaying plan outcome.
     * @param[in,out] out MarkerArray receiving generated HUD marker.
     * @param[in] resp Service response.
     * @param[in] move_uci UCI move string.
     * @param[in] stamp Marker timestamp.
     */
    void append_hud(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const std::string &move_uci,
        const builtin_interfaces::msg::Time &stamp) const;

    /**
     * @brief Appends target spheres, vertical approach rays, and labels for pick/place/clear.
     * @param[in,out] out MarkerArray receiving generated markers.
     * @param[in] resp Service response.
     * @param[in] stamp Marker timestamp.
     */
    void append_target_points(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    /**
     * @brief Appends chassis cylinder discs and heading arrows for planned base standoffs.
     * @param[in,out] out MarkerArray receiving generated markers.
     * @param[in] resp Service response.
     * @param[in] stamp Marker timestamp.
     */
    void append_base_footprints(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    /**
     * @brief Appends horizontal workspace reach circles around base standoffs.
     * @param[in,out] out MarkerArray receiving generated markers.
     * @param[in] resp Service response.
     * @param[in] stamp Marker timestamp.
     */
    void append_reach_envelopes(
        visualization_msgs::msg::MarkerArray &out,
        const lekiwi_interfaces::srv::CheckMoveFeasibility::Response &resp,
        const builtin_interfaces::msg::Time &stamp) const;

    /**
     * @brief Appends line strip representing multi-base navigation paths between standoffs.
     * @param[in,out] out MarkerArray receiving generated markers.
     * @param[in] resp Service response.
     * @param[in] stamp Marker timestamp.
     */
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

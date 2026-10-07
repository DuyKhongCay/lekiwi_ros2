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
 * @file workspace_checker_node.hpp
 * @brief ROS 2 service node evaluating mobile manipulator reachability for chess moves.
 * @details Integrates URDF kinematics parsing, TF tree lookups, chessboard coordinate mapping,
 *          multi-tier workspace standoff planning, and 3D visualization publishing.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__WORKSPACE_CHECKER_NODE_HPP_
#define LEKIWI_MOTION__WORKSPACE_CHECKER_NODE_HPP_

#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lekiwi_motion/kinematics/chessboard_mapper.hpp"
#include "lekiwi_motion/kinematics/feasibility_marker_builder.hpp"
#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"
#include "lekiwi_motion/kinematics/workspace_planner.hpp"

namespace lekiwi_motion
{

    /**
     * @class WorkspaceCheckerNode
     * @brief Node evaluating inverse kinematics and mobile base reachability for chess moves.
     * @details Provides the `/workspace/check_move_feasibility` service, publishes diagnostic metrics,
     *          and emits multi-layer RViz markers for planned standoff positions. Uses RCU pattern
     *          with `std::shared_mutex` for lock-free reader access to kinematics models.
     */
    class WorkspaceCheckerNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructs WorkspaceCheckerNode and declares configuration parameters.
         * @param[in] options Node options passed to base rclcpp::Node.
         */
        explicit WorkspaceCheckerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        /**
         * @struct TransformContext
         * @brief Cached transform snapshots and planar base pose for a planning cycle.
         */
        struct TransformContext
        {
            geometry_msgs::msg::TransformStamped base_tf;       ///< TF from board to base_footprint.
            geometry_msgs::msg::TransformStamped board_to_map;  ///< TF from map to chessboard.
            workspace::BasePose current_base;                   ///< Planar coordinates of base.
            double base_z{0.0};                                 ///< Elevation of base on board surface.
            rclcpp::Time stamp;                                 ///< Timestamp of lookup.
        };

        /**
         * @struct TargetPoints
         * @brief Cartesian target positions and ROS message wrappers for a chess move.
         */
        struct TargetPoints
        {
            workspace::Point3D clear;                  ///< Target point for captured piece.
            workspace::Point3D pick;                   ///< Target point for piece origin square.
            workspace::Point3D place;                  ///< Target point for piece destination square.
            geometry_msgs::msg::Point clear_pt_msg;    ///< ROS message for clear point.
            geometry_msgs::msg::Point pick_pt_msg;     ///< ROS message for pick point.
            geometry_msgs::msg::Point place_pt_msg;    ///< ROS message for place point.
            bool is_capture{false};                    ///< Flag indicating capture move.
        };

        /**
         * @brief Declares and parses node parameters with range constraints.
         */
        void load_parameters();

        /**
         * @brief Sets up robot_description subscription or parameter-based loading.
         */
        void init_kinematics();

        /**
         * @brief Computes board half-dimensions and instantiates ChessboardMapper.
         */
        void init_workspace_bounds();

        /**
         * @brief Parses URDF XML and builds KinematicsModel and WorkspacePlanner under write lock.
         * @param[in] xml URDF XML string.
         * @param[in] source Source descriptor ("parameter" or "topic").
         */
        void accept_robot_description(const std::string &xml, const std::string &source);

        /**
         * @brief Callback receiving URDF model updates from `/robot_description`.
         * @param[in] msg URDF XML message.
         */
        void on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg);

        /**
         * @brief Core service handler evaluating move feasibility.
         * @param[in] request Move details including from, to, and captured square.
         * @param[out] response Feasibility result, standoff poses, and joint states.
         */
        void handle_check_move_feasibility(
            const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request> request,
            std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> response);

        /**
         * @brief Obtains read-only pointer snapshot of active WorkspacePlanner under shared lock.
         * @return Shared pointer to const WorkspacePlanner, or nullptr if not ready.
         */
        std::shared_ptr<const workspace::WorkspacePlanner> get_planner_snapshot() const;

        /**
         * @brief Validates algebraic square notation in incoming service request.
         * @param[in] request Request to validate.
         * @return Error message if invalid; std::nullopt if valid.
         */
        std::optional<std::string> validate_request(
            const lekiwi_interfaces::srv::CheckMoveFeasibility::Request &request) const noexcept;

        /**
         * @brief Resolves and checks freshness of TF transforms between map, board, and base.
         * @param[in] now Current ROS clock timestamp.
         * @param[out] out_status Feasibility status populated upon failure.
         * @param[out] out_error Error description message populated upon failure.
         * @return TransformContext if lookups succeed and are fresh.
         */
        std::optional<TransformContext> resolve_base_transforms(
            const rclcpp::Time &now,
            workspace::FeasibilityStatus &out_status,
            std::string &out_error) const;

        /**
         * @brief Converts algebraic squares into 3D metric coordinates.
         * @param[in] request Service request.
         * @param[out] out_error Error description if conversion fails.
         * @return TargetPoints struct if mapping succeeds.
         */
        std::optional<TargetPoints> resolve_target_points(
            const lekiwi_interfaces::srv::CheckMoveFeasibility::Request &request,
            std::string &out_error);

        /**
         * @brief Assembles successful PlanResult into ROS service response message.
         * @param[in] plan PlanResult from WorkspacePlanner.
         * @param[in] targets 3D target points.
         * @param[in] tf_ctx Transform context with board-to-map frame data.
         * @param[out] response Response message to populate.
         */
        void populate_success_response(
            const workspace::PlanResult &plan,
            const TargetPoints &targets,
            const TransformContext &tf_ctx,
            lekiwi_interfaces::srv::CheckMoveFeasibility::Response &response) const;

        /**
         * @brief Fills error response and logs failure status.
         * @param[out] response Service response.
         * @param[in] status Feasibility status enum.
         * @param[in] message Explanation string.
         */
        void set_error_response(
            lekiwi_interfaces::srv::CheckMoveFeasibility::Response &response,
            workspace::FeasibilityStatus status,
            const std::string &message);

        /**
         * @brief Diagnostics updater callback reporting kinematics status and workspace bounds.
         * @param[out] stat Diagnostics status wrapper to populate.
         */
        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

        /**
         * @brief Transforms planar base pose into PoseStamped in global map frame.
         * @param[in] base Planar base coordinates in chessboard frame.
         * @param[in] board_to_map Transform from board frame to map frame.
         * @param[in] stamp Header timestamp.
         * @return Transformed PoseStamped in map frame.
         */
        geometry_msgs::msg::PoseStamped make_pose_stamped(
            const workspace::BasePose &base,
            const geometry_msgs::msg::TransformStamped &board_to_map,
            const rclcpp::Time &stamp) const;

        /// ROS Entities
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Service<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr service_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr feasibility_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
        std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
        std::unique_ptr<visualization::FeasibilityMarkerBuilder> marker_builder_;

        /// Coordinate Frame Names
        std::string map_frame_{"map"};
        std::string board_frame_{"chessboard_frame"};
        std::string base_frame_{"base_footprint"};
        std::string tip_frame_{"gripperframe"};

        /// Tolerances & Physical Constants
        static constexpr double CLOCK_JITTER_TOLERANCE_SEC{0.050}; ///< 50ms tolerance for clock skew.
        static constexpr double DEFAULT_PIECE_GRASP_Z_M{0.025};    ///< Default grasp height fallback (m).
        double max_tf_age_sec_{0.30};
        double planar_tolerance_rad_{0.01};
        double safety_margin_rad_{0.05};
        double grasp_z_{DEFAULT_PIECE_GRASP_Z_M};

        /// Thread-safe Kinematics Snapshot (RCU Pattern)
        mutable std::shared_mutex kinematics_mutex_;
        std::shared_ptr<const workspace::KinematicsModel> kinematics_model_;
        std::shared_ptr<const workspace::WorkspacePlanner> workspace_planner_;
        std::vector<std::string> arm_joint_names_;
        std::string model_source_{"none"};
        std::string model_status_{"Waiting for robot_description"};
        workspace::WorkspaceConfig workspace_;
        std::optional<ChessboardMapper> chessboard_mapper_;
        bool last_feasible_{true};
        std::string last_status_{"Configured; waiting for query"};
    };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__WORKSPACE_CHECKER_NODE_HPP_

// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_MOTION__WORKSPACE_CHECKER_NODE_HPP_
#define LEKIWI_MOTION__WORKSPACE_CHECKER_NODE_HPP_

#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lekiwi_motion/chessboard_mapper.hpp"
#include "lekiwi_motion/workspace_kinematics.hpp"
#include "lekiwi_motion/workspace_planner.hpp"

namespace lekiwi_motion
{

    class WorkspaceCheckerNode : public rclcpp::Node
    {
    public:
        explicit WorkspaceCheckerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        struct TransformContext
        {
            geometry_msgs::msg::TransformStamped base_tf;
            geometry_msgs::msg::TransformStamped board_to_map;
            workspace::BasePose current_base;
            double base_z{0.0};
            rclcpp::Time stamp;
        };

        struct TargetPoints
        {
            workspace::Point3D clear;
            workspace::Point3D pick;
            workspace::Point3D place;
            geometry_msgs::msg::Point clear_pt_msg;
            geometry_msgs::msg::Point pick_pt_msg;
            geometry_msgs::msg::Point place_pt_msg;
            bool is_capture{false};
        };

        void load_parameters();
        void init_kinematics();
        void init_workspace_bounds();
        void accept_robot_description(const std::string &xml, const std::string &source);
        void on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg);

        void handle_check_move_feasibility(
            const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request> request,
            std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> response);

        std::shared_ptr<const workspace::WorkspacePlanner> get_planner_snapshot() const;

        std::optional<std::string> validate_request(
            const lekiwi_interfaces::srv::CheckMoveFeasibility::Request &request) const noexcept;

        std::optional<TransformContext> resolve_base_transforms(
            const rclcpp::Time &now,
            workspace::FeasibilityStatus &out_status,
            std::string &out_error) const;

        std::optional<TargetPoints> resolve_target_points(
            const lekiwi_interfaces::srv::CheckMoveFeasibility::Request &request,
            std::string &out_error);

        void populate_success_response(
            const workspace::PlanResult &plan,
            const TargetPoints &targets,
            const TransformContext &tf_ctx,
            lekiwi_interfaces::srv::CheckMoveFeasibility::Response &response) const;

        void set_error_response(
            lekiwi_interfaces::srv::CheckMoveFeasibility::Response &response,
            workspace::FeasibilityStatus status,
            const std::string &message);

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

        geometry_msgs::msg::PoseStamped make_pose_stamped(
            const workspace::BasePose &base,
            const geometry_msgs::msg::TransformStamped &board_to_map,
            const rclcpp::Time &stamp) const;

        // ROS Entities
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Service<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr service_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
        std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

        // Frame names
        std::string map_frame_{"map"};
        std::string board_frame_{"chessboard_frame"};
        std::string base_frame_{"base_footprint"};
        std::string tip_frame_{"gripperframe"};

        // Tolerances & Constants
        static constexpr double CLOCK_JITTER_TOLERANCE_SEC{0.050}; // 50ms tolerance for clock skew
        static constexpr double DEFAULT_PIECE_GRASP_Z_M{0.025};    // Default grasp height fallback (m)
        double max_tf_age_sec_{0.30};
        double planar_tolerance_rad_{0.01};
        double safety_margin_rad_{0.05};
        double grasp_z_{DEFAULT_PIECE_GRASP_Z_M};

        // Thread-safe Kinematics Snapshot (RCU Pattern)
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

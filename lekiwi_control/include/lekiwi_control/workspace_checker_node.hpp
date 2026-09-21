// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_CONTROL__WORKSPACE_CHECKER_NODE_HPP_
#define LEKIWI_CONTROL__WORKSPACE_CHECKER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lekiwi_control/workspace/types.hpp"

namespace lekiwi_control
{

  class WorkspaceCheckerNode : public rclcpp::Node
  {
  public:
    explicit WorkspaceCheckerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void load_parameters();
    void init_kinematics();
    void init_workspace_bounds();

    void on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg);

    void handle_check_move_feasibility(
        const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request> request,
        std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> response);

    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    workspace::Point3D transform_point(
        const geometry_msgs::msg::Point &pt,
        const std::string &src_frame,
        const rclcpp::Time &stamp);

    geometry_msgs::msg::PoseStamped make_pose_stamped(
        const workspace::BasePose &base,
        const geometry_msgs::msg::TransformStamped &board_to_map,
        const rclcpp::Time &stamp);

    sensor_msgs::msg::JointState make_joint_state(
        const std::array<double, 5> &joints,
        const rclcpp::Time &stamp);

    // ROS entities
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Service<lekiwi_interfaces::srv::CheckMoveFeasibility>::SharedPtr service_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
    std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

    // Frame names
    std::string map_frame_{"map"};
    std::string board_frame_{"chessboard_frame"};
    std::string base_frame_{"base_footprint"};

    // Tolerances
    double max_tf_age_sec_{0.30};
    double planar_tolerance_rad_{0.01};

    // State
    workspace::KinematicsConfig kinematics_;
    workspace::WorkspaceConfig workspace_;
    bool kinematics_loaded_{false};
    bool last_feasible_{true};
    std::string last_status_{"Configured; waiting for query"};
  };

} // namespace lekiwi_control

#endif // LEKIWI_CONTROL__WORKSPACE_CHECKER_NODE_HPP_

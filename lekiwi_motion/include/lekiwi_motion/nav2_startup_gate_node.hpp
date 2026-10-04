// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_MOTION__NAV2_STARTUP_GATE_NODE_HPP_
#define LEKIWI_MOTION__NAV2_STARTUP_GATE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lekiwi_motion
{

  class Nav2StartupGateNode : public rclcpp::Node
  {
  public:
    explicit Nav2StartupGateNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~Nav2StartupGateNode() override = default;

    // Pure logic helpers for testability
    bool is_started() const noexcept { return started_.load(); }
    int get_consecutive_count() const noexcept { return consecutive_count_; }

  private:
    void check_and_startup();
    void on_startup_response(
        rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future);

    // Configuration parameters
    std::string map_frame_;
    std::string base_frame_;
    std::string lifecycle_service_;
    double check_frequency_hz_;
    int consecutive_success_threshold_;
    bool exit_on_success_;

    // State
    std::atomic<bool> started_{false};
    bool request_in_flight_{false};
    int consecutive_count_{0};

    // ROS 2 Primitives
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__NAV2_STARTUP_GATE_NODE_HPP_

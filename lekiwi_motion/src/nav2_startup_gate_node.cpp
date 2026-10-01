// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/nav2_startup_gate_node.hpp"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace lekiwi_motion
{

Nav2StartupGateNode::Nav2StartupGateNode(const rclcpp::NodeOptions & options)
: Node("nav2_startup_gate", options)
{
  declare_parameter("map_frame", std::string("map"));
  declare_parameter("base_frame", std::string("base_footprint"));
  declare_parameter("lifecycle_service", std::string("/lifecycle_manager_navigation/manage_nodes"));
  declare_parameter("check_frequency_hz", 2.0);
  declare_parameter("consecutive_success_threshold", 2);
  declare_parameter("exit_on_success", false);

  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  lifecycle_service_ = get_parameter("lifecycle_service").as_string();
  check_frequency_hz_ = get_parameter("check_frequency_hz").as_double();
  consecutive_success_threshold_ = get_parameter("consecutive_success_threshold").as_int();
  exit_on_success_ = get_parameter("exit_on_success").as_bool();

  if (check_frequency_hz_ <= 0.0) {
    check_frequency_hz_ = 2.0;
  }
  if (consecutive_success_threshold_ < 1) {
    consecutive_success_threshold_ = 1;
  }

  // MutuallyExclusive callback group ensures no concurrent ticks while handling response
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // Service client for Nav2 lifecycle manager
  client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(
    lifecycle_service_, rmw_qos_profile_services_default, cb_group_);

  // Wall timer for periodic TF verification
  const auto period = std::chrono::duration<double>(1.0 / check_frequency_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Nav2StartupGateNode::check_and_startup, this),
    cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "[nav2_startup_gate] Initialized. Monitoring TF chain '%s' -> '%s' to trigger '%s'.",
    map_frame_.c_str(), base_frame_.c_str(), lifecycle_service_.c_str());
}

void Nav2StartupGateNode::check_and_startup()
{
  if (started_.load() || request_in_flight_) {
    return;
  }

  // 1. Ensure Nav2 lifecycle manager service is reachable
  if (!client_->service_is_ready()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[nav2_startup_gate] Waiting for Nav2 lifecycle service '%s' to become available...",
      lifecycle_service_.c_str());
    return;
  }

  // 2. Query TF availability without throwing exceptions
  std::string tf_error;
  const bool can_transform = tf_buffer_->canTransform(
    map_frame_, base_frame_, tf2::TimePointZero, &tf_error);

  if (can_transform) {
    consecutive_count_++;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[nav2_startup_gate] TF chain '%s' -> '%s' detected (streak: %d/%d)...",
      map_frame_.c_str(), base_frame_.c_str(), consecutive_count_,
      consecutive_success_threshold_);

    if (consecutive_count_ >= consecutive_success_threshold_) {
      request_in_flight_ = true;
      auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;

      RCLCPP_INFO(
        get_logger(),
        "[nav2_startup_gate] TF tree fully connected! Dispatching STARTUP to '%s'...",
        lifecycle_service_.c_str());

      client_->async_send_request(
        request,
        std::bind(&Nav2StartupGateNode::on_startup_response, this, std::placeholders::_1));
    }
  } else {
    if (consecutive_count_ > 0) {
      RCLCPP_WARN(
        get_logger(),
        "[nav2_startup_gate] TF chain '%s' -> '%s' dropped during debounce. Resetting streak.",
        map_frame_.c_str(), base_frame_.c_str());
      consecutive_count_ = 0;
    }
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[nav2_startup_gate] Waiting for TF '%s' -> '%s': %s",
      map_frame_.c_str(), base_frame_.c_str(), tf_error.c_str());
  }
}

void Nav2StartupGateNode::on_startup_response(
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future)
{
  request_in_flight_ = false;

  try {
    const auto response = future.get();
    if (response->success) {
      started_.store(true);
      RCLCPP_INFO(
        get_logger(),
        "[nav2_startup_gate] >>> Nav2 stack successfully transitioned to ACTIVE! Entering idle state.");

      // Disarm timer and release TF buffer/listener to consume zero CPU/RAM
      timer_->cancel();
      tf_listener_.reset();
      tf_buffer_.reset();

      if (exit_on_success_) {
        RCLCPP_INFO(get_logger(), "[nav2_startup_gate] Auto-exit on success requested. Shutting down node.");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[nav2_startup_gate] Nav2 lifecycle manager returned success=false! Will re-evaluate TF and retry.");
      consecutive_count_ = 0;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "[nav2_startup_gate] Exception while waiting for Nav2 startup response: %s",
      e.what());
    consecutive_count_ = 0;
  }
}

}  // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::Nav2StartupGateNode)

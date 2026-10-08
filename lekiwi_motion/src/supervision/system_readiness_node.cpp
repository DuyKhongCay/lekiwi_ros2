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
 * @file system_readiness_node.cpp
 * @brief Implementation of ROS 2 system readiness monitoring node.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/supervision/system_readiness_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace lekiwi_motion
{

  SystemReadinessNode::SystemReadinessNode(const rclcpp::NodeOptions &options)
      : Node("system_readiness_node", options),
        diagnostic_updater_(this)
  {
    auto declare = [this](const auto &name, const auto &value, const std::string &desc_str)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = desc_str;
      declare_parameter(name, value, desc);
    };

    declare("max_pos_variance", 0.0012, "Max position variance for grasp readiness (m^2)");
    declare("max_yaw_variance", 0.0030, "Max yaw variance for grasp readiness (rad^2)");
    declare("max_transform_age_sec", 0.30, "Max age for precision TF / grasp signals (sec)");
    declare("nav_odom_max_age_sec", 0.50, "Max age for local odometry in nav readiness (sec)");
    declare("max_stop_velocity", 0.03, "Max linear speed to be considered stationary (m/s)");
    declare("max_stop_angular_vel", 0.08, "Max angular speed to be considered stationary (rad/s)");
    declare("check_frequency_hz", 10.0, "Frequency of readiness evaluation loop (Hz)");
    declare("require_global_ekf_seed", true, "Require global EKF to converge at least once before nav ready");

    declare("map_frame", std::string("map"), "Global map frame");
    declare("odom_frame", std::string("odom"), "Odometry frame");
    declare("base_frame", std::string("base_footprint"), "Base footprint frame");
    declare("tip_frame", std::string("gripperframe"), "End-effector / gripper frame");
    declare("board_frame", std::string("chessboard_frame"), "Chessboard target frame");
    declare("local_odom_topic", std::string("/odometry/local"), "Topic name for local filtered odometry");
    declare("global_odom_topic", std::string("/odometry/global"), "Topic name for global fused odometry");
    declare("joint_states_topic", std::string("/joint_states"), "Topic name for robot joint states");
    declare("arm_joints", std::vector<std::string>{"arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"},
            "List of required joints for arm grasp readiness");

    EvaluatorConfig config;
    config.max_pos_variance = get_parameter("max_pos_variance").as_double();
    config.max_yaw_variance = get_parameter("max_yaw_variance").as_double();
    config.max_transform_age_sec = get_parameter("max_transform_age_sec").as_double();
    config.nav_odom_max_age_sec = get_parameter("nav_odom_max_age_sec").as_double();
    config.max_stop_velocity = get_parameter("max_stop_velocity").as_double();
    config.max_stop_angular_vel = get_parameter("max_stop_angular_vel").as_double();
    config.require_global_ekf_seed = get_parameter("require_global_ekf_seed").as_bool();
    config.arm_joints = get_parameter("arm_joints").as_string_array();

    check_frequency_hz_ = get_parameter("check_frequency_hz").as_double();
    max_transform_age_sec_ = config.max_transform_age_sec;
    nav_odom_max_age_sec_ = config.nav_odom_max_age_sec;

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    tip_frame_ = get_parameter("tip_frame").as_string();
    board_frame_ = get_parameter("board_frame").as_string();
    local_odom_topic_ = get_parameter("local_odom_topic").as_string();
    global_odom_topic_ = get_parameter("global_odom_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();

    evaluator_ = SystemReadinessEvaluator(config);

    // Validate numeric parameters
    if (check_frequency_hz_ <= 0.0 || check_frequency_hz_ > 1000.0 ||
        max_transform_age_sec_ <= 0.0 || nav_odom_max_age_sec_ <= 0.0 ||
        map_frame_.empty() || odom_frame_.empty() ||
        base_frame_.empty() || local_odom_topic_.empty() || global_odom_topic_.empty() ||
        joint_states_topic_.empty())
    {
      throw std::invalid_argument("Invalid parameter values configured in SystemReadinessNode");
    }

    // TF2 Buffer & Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Subscriptions (Sensor QoS: depth 10, best effort)
    auto sensor_qos = rclcpp::QoS(10).best_effort();
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        local_odom_topic_, sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        { on_local_odom(msg); });

    global_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        global_odom_topic_, sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        { on_global_odom(msg); });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, sensor_qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
        { on_joint_states(msg); });

    // Latched Publishers (Transient Local, Reliable)
    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    nav_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/nav_ready", latched_qos);
    grasp_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/grasp_ready", latched_qos);

    // Publish initial false state
    std_msgs::msg::Bool initial_false;
    initial_false.data = false;
    nav_ready_pub_->publish(initial_false);
    grasp_ready_pub_->publish(initial_false);

    // Services
    nav_readiness_srv_ = create_service<std_srvs::srv::Trigger>(
        "/system/check_nav_readiness",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        { handle_nav_query(req, res); });

    grasp_readiness_srv_ = create_service<std_srvs::srv::Trigger>(
        "/system/check_grasp_readiness",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        { handle_grasp_query(req, res); });

    // Diagnostics
    diagnostic_updater_.setHardwareID("lekiwi_system_readiness");
    diagnostic_updater_.add("Readiness Status", this, &SystemReadinessNode::produce_diagnostics);

    // Periodic Timer
    const auto period = std::chrono::duration<double>(1.0 / check_frequency_hz_);
    eval_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&SystemReadinessNode::evaluate_readiness, this));

    RCLCPP_INFO(get_logger(),
                "[System Readiness] Node started (%.1f Hz) — Sensor & Hardware Inspector ready",
                check_frequency_hz_);
  }

  void SystemReadinessNode::on_local_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    local_odom_snapshot_.valid = true;
    local_odom_snapshot_.stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    local_odom_snapshot_.vx = msg->twist.twist.linear.x;
    local_odom_snapshot_.vy = msg->twist.twist.linear.y;
    local_odom_snapshot_.wz = msg->twist.twist.angular.z;
  }

  void SystemReadinessNode::on_global_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    global_odom_snapshot_.valid = true;
    global_odom_snapshot_.stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    const auto &cov = msg->pose.covariance;
    global_odom_snapshot_.pos_variance = cov[0] + cov[7];
    global_odom_snapshot_.yaw_variance = cov[35];
  }

  void SystemReadinessNode::on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    joints_snapshot_.valid = true;
    joints_snapshot_.stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    for (const auto &name : msg->name)
    {
      joints_snapshot_.joint_names.insert(name);
    }
  }

  bool SystemReadinessNode::is_transform_fresh(
      const std::string &target,
      const std::string &source,
      double now_sec,
      double max_age_sec,
      bool is_static) const
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      if (is_static)
      {
        return true;
      }
      const double stamp_sec = rclcpp::Time(tf.header.stamp).seconds();
      return policy::fresh(now_sec, stamp_sec, max_age_sec);
    }
    catch (const tf2::TransformException &)
    {
      return false;
    }
  }

  void SystemReadinessNode::evaluate_readiness()
  {
    const double now_sec = get_clock()->now().seconds();
    if (last_eval_time_sec_ > 0.0 && now_sec < last_eval_time_sec_)
    {
      tf_buffer_->clear();
      evaluator_.reset_global_ekf_seed();
    }
    last_eval_time_sec_ = now_sec;

    // 1. Inspect TF chains
    TfChainSnapshot tf_chains;
    tf_chains.odom_to_base_fresh = is_transform_fresh(
        odom_frame_, base_frame_, now_sec, nav_odom_max_age_sec_, false);
    tf_chains.base_to_gripper_fresh = is_transform_fresh(
        base_frame_, tip_frame_, now_sec, max_transform_age_sec_, false);
    tf_chains.map_to_board_fresh = is_transform_fresh(
        map_frame_, board_frame_, now_sec, max_transform_age_sec_, true);

    // 2. Evaluate Pure Domain Logic
    last_report_ = evaluator_.evaluate(
        now_sec, local_odom_snapshot_, global_odom_snapshot_, joints_snapshot_, tf_chains);

    // 3. Publish Heartbeat & Log Transitions
    if (last_report_.nav_ready != last_published_nav_ready_)
    {
      last_published_nav_ready_ = last_report_.nav_ready;
      RCLCPP_INFO(get_logger(),
                  "[System Readiness] /system/nav_ready transition: %s",
                  last_published_nav_ready_ ? "TRUE" : "FALSE");
    }

    if (last_report_.grasp_ready != last_published_grasp_ready_)
    {
      last_published_grasp_ready_ = last_report_.grasp_ready;
      RCLCPP_INFO(get_logger(),
                  "[System Readiness] /system/grasp_ready transition: %s (Status: %s)",
                  last_published_grasp_ready_ ? "TRUE" : "FALSE",
                  last_report_.grasp_blocker_reason.c_str());
    }

    // Continuous Heartbeat at check_frequency_hz (10Hz) to satisfy watchdog leases
    std_msgs::msg::Bool nav_msg;
    nav_msg.data = last_report_.nav_ready;
    nav_ready_pub_->publish(nav_msg);

    std_msgs::msg::Bool grasp_msg;
    grasp_msg.data = last_report_.grasp_ready;
    grasp_ready_pub_->publish(grasp_msg);

    // 4. Diagnostics
    diagnostic_updater_.force_update();
  }

  void SystemReadinessNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (last_report_.grasp_ready && last_report_.nav_ready)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Full System Ready (Nav & Grasp)");
    }
    else if (last_report_.nav_ready)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Nav Ready (Grasp Standby/Moving)");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "System Unready (Nav Disabled)");
    }

    stat.add("Nav Ready", last_report_.nav_ready ? "true" : "false");
    stat.add("Grasp Ready", last_report_.grasp_ready ? "true" : "false");
    stat.add("Grasp Status Detail", last_report_.grasp_blocker_reason);
    stat.add("Robot Stationary", last_report_.is_stationary ? "true" : "false");
    stat.add("Speed (m/s)", last_report_.current_speed);
    stat.add("Pos Variance (m^2)", last_report_.current_pos_variance);

    const char *drift_str = "None";
    if (last_report_.drift_type == DriftType::MOVING)
    {
      drift_str = "Moving";
    }
    else if (last_report_.drift_type == DriftType::STATIONARY_NO_TAG)
    {
      drift_str = "Stationary without AprilTag";
    }
    else if (last_report_.drift_type == DriftType::NOT_SEEDED)
    {
      drift_str = "Not Seeded";
    }
    stat.add("Drift Type", drift_str);
    stat.add("Stationary Latched", last_report_.stationary_latched ? "true" : "false");
  }

  void SystemReadinessNode::handle_nav_query(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    res->success = last_report_.nav_ready;
    res->message = last_report_.nav_ready ? "Navigation is ready" : "Navigation not ready";
  }

  void SystemReadinessNode::handle_grasp_query(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    res->success = last_report_.grasp_ready;
    res->message = last_report_.grasp_ready ? "Grasp precision is ready" : "Grasp not ready: " + last_report_.grasp_blocker_reason;
  }

} // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::SystemReadinessNode)

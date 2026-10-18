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
 * @file cartesian_jog_server_node.cpp
 * @brief Implementation of standalone Cartesian Jog Server node.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/arm/cartesian_jog_server_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <urdf_parser/urdf_parser.h>

namespace lekiwi_motion::arm
{

CartesianJogServerNode::CartesianJogServerNode(const rclcpp::NodeOptions & options)
: Node("cartesian_jog_server_node", options)
{
  declare_all_parameters();
  load_parameters();

  // Create Subscriptions
  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianJogServerNode::cmd_vel_callback, this, std::placeholders::_1));

  sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&CartesianJogServerNode::joint_states_callback, this, std::placeholders::_1));

  // Initialize URDF from parameter if provided, otherwise subscribe
  const std::string urdf_xml = get_parameter("robot_description").as_string();
  if (!urdf_xml.empty()) {
    if (!init_kinematics_from_urdf(urdf_xml)) {
      throw std::runtime_error(
        "CartesianJogServerNode: Failed to initialize kinematics from 'robot_description' parameter: " +
        kinematics_error_msg());
    }
  } else {
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();
    qos_profile.reliable();
    sub_robot_description_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, qos_profile,
      std::bind(&CartesianJogServerNode::robot_description_callback, this, std::placeholders::_1));
  }

  rclcpp::QoS busy_qos(1);
  busy_qos.transient_local();
  busy_qos.reliable();
  sub_arm_busy_ = create_subscription<std_msgs::msg::Bool>(
    arm_busy_topic_, busy_qos,
    std::bind(&CartesianJogServerNode::arm_busy_callback, this, std::placeholders::_1));

  // Create Publishers
  if (controller_type_ == "forward_position") {
    pub_forward_commands_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      arm_forward_topic_, rclcpp::SystemDefaultsQoS());
  } else {
    pub_trajectory_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      arm_trajectory_topic_, rclcpp::SystemDefaultsQoS());
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CartesianJogServerNode::control_loop, this));

  RCLCPP_INFO(
    get_logger(),
    "CartesianJogServerNode initialized at %.1f Hz. Controller: '%s'. Output topic: '%s'",
    publish_rate_hz_, controller_type_.c_str(),
    (controller_type_ == "forward_position" ?
      arm_forward_topic_.c_str() : arm_trajectory_topic_.c_str()));
}

void CartesianJogServerNode::declare_all_parameters()
{
  declare_parameter<double>("publish_rate_hz", 50.0);
  declare_parameter<double>("point_dt_s", 0.08);
  declare_parameter<double>("stale_timeout_s", 0.25);
  declare_parameter<double>("lpf_alpha", 0.85);
  declare_parameter<std::string>("controller_type", "joint_trajectory");

  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("tip_frame", "gripperframe");
  declare_parameter<double>("safety_margin_rad", 0.0);

  declare_parameter<std::vector<std::string>>(
    "arm_joint_names",
    std::vector<std::string>{
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
      "arm_wrist_flex", "arm_wrist_roll"});

  declare_parameter<std::string>(
    "arm_forward_topic", "/arm_forward_controller/commands");
  declare_parameter<std::string>(
    "arm_trajectory_topic", "/arm_trajectory_controller/joint_trajectory");
  declare_parameter<std::string>("arm_busy_topic", "/arm/is_busy");
  declare_parameter<std::string>("joint_states_topic", "/joint_states");
  declare_parameter<std::string>("robot_description_topic", "/robot_description");
  declare_parameter<std::string>("robot_description", "");
}

void CartesianJogServerNode::load_parameters()
{
  publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
  point_dt_s_ = std::max(0.01, get_parameter("point_dt_s").as_double());
  stale_timeout_s_ = std::max(0.05, get_parameter("stale_timeout_s").as_double());
  lpf_alpha_ = std::clamp(get_parameter("lpf_alpha").as_double(), 0.0, 1.0);
  controller_type_ = get_parameter("controller_type").as_string();
  if (controller_type_ != "joint_trajectory" && controller_type_ != "forward_position") {
    RCLCPP_WARN(
      get_logger(), "Unknown controller_type '%s', defaulting to 'joint_trajectory'",
      controller_type_.c_str());
    controller_type_ = "joint_trajectory";
  }

  base_frame_ = get_parameter("base_frame").as_string();
  tip_frame_ = get_parameter("tip_frame").as_string();
  safety_margin_rad_ = get_parameter("safety_margin_rad").as_double();
  arm_joint_names_ = get_parameter("arm_joint_names").as_string_array();

  arm_forward_topic_ = get_parameter("arm_forward_topic").as_string();
  arm_trajectory_topic_ = get_parameter("arm_trajectory_topic").as_string();
  arm_busy_topic_ = get_parameter("arm_busy_topic").as_string();
  joint_states_topic_ = get_parameter("joint_states_topic").as_string();
  robot_description_topic_ = get_parameter("robot_description_topic").as_string();

  motion_filter_.set_alpha(lpf_alpha_);
}

bool CartesianJogServerNode::is_kinematics_initialized() const noexcept
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  return kinematics_state_ == KinematicsState::INITIALIZED;
}

bool CartesianJogServerNode::has_kinematics_failed() const noexcept
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  return kinematics_state_ == KinematicsState::FAILED;
}

std::string CartesianJogServerNode::kinematics_error_msg() const
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  return kinematics_error_msg_;
}

bool CartesianJogServerNode::is_active() const noexcept
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_jogging_;
}

bool CartesianJogServerNode::is_arm_busy() const noexcept
{
  return is_arm_busy_.load(std::memory_order_acquire);
}

bool CartesianJogServerNode::init_kinematics_from_urdf(const std::string & urdf_xml)
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  if (kinematics_state_ == KinematicsState::INITIALIZED) {
    return true;
  }

  auto urdf_model = urdf::parseURDF(urdf_xml);
  if (!urdf_model) {
    kinematics_state_ = KinematicsState::FAILED;
    kinematics_error_msg_ = "Failed to parse URDF XML";
    RCLCPP_ERROR(get_logger(), "CartesianJogServerNode kinematics error: %s", kinematics_error_msg_.c_str());
    return false;
  }

  std::string err_msg;
  if (!workspace::extract_kinematics_from_urdf(
        *urdf_model, base_frame_, tip_frame_, arm_joint_names_,
        kinematics_model_, err_msg, safety_margin_rad_))
  {
    kinematics_state_ = KinematicsState::FAILED;
    kinematics_error_msg_ = "Kinematics extraction failed: " + err_msg;
    RCLCPP_ERROR(get_logger(), "CartesianJogServerNode kinematics error: %s", kinematics_error_msg_.c_str());
    return false;
  }

  try {
    ik_solver_ = std::make_shared<workspace::SO101AnalyticalSolver>(kinematics_model_);
    jogger_ = std::make_unique<CartesianJogger>(ik_solver_, kinematics_model_);
    kinematics_state_ = KinematicsState::INITIALIZED;
    kinematics_error_msg_.clear();
    RCLCPP_INFO(
      get_logger(),
      "Kinematics & Analytical IK initialized successfully. Reach envelope radius: %.3f m",
      ik_solver_->reach_bound());
    return true;
  } catch (const std::exception & e) {
    kinematics_state_ = KinematicsState::FAILED;
    kinematics_error_msg_ = "Exception initializing IK solver: " + std::string(e.what());
    RCLCPP_ERROR(get_logger(), "CartesianJogServerNode kinematics error: %s", kinematics_error_msg_.c_str());
    return false;
  }
}

void CartesianJogServerNode::robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || msg->data.empty()) {
    return;
  }
  const bool ok = init_kinematics_from_urdf(msg->data);
  sub_robot_description_.reset();
  if (!ok) {
    RCLCPP_ERROR(
      get_logger(),
      "Fatal: robot_description received on '%s' failed kinematics initialization: %s. Cartesian jogging disabled.",
      robot_description_topic_.c_str(), kinematics_error_msg().c_str());
  }
}

void CartesianJogServerNode::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_twist_ = msg->twist;
  last_twist_time_ = now();
}

void CartesianJogServerNode::arm_busy_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const bool busy = msg->data;
  const bool prev = is_arm_busy_.exchange(busy, std::memory_order_acq_rel);
  if (busy && !prev) {
    RCLCPP_INFO(get_logger(), "Arm trajectory lock active (/arm/is_busy = true). Jogging suppressed.");
  } else if (!busy && prev) {
    RCLCPP_INFO(get_logger(), "Arm trajectory lock released (/arm/is_busy = false). Jogging resumed.");
  }
}

void CartesianJogServerNode::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg || msg->name.empty() || msg->position.size() < msg->name.size()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (joint_state_indices_.size() != 5) {
    joint_state_indices_.assign(5, -1);
  }

  for (size_t i = 0; i < arm_joint_names_.size() && i < 5; ++i) {
    const int idx = joint_state_indices_[i];
    if (idx < 0 || idx >= static_cast<int>(msg->name.size()) ||
      msg->name[idx] != arm_joint_names_[i])
    {
      auto it = std::find(msg->name.begin(), msg->name.end(), arm_joint_names_[i]);
      if (it != msg->name.end()) {
        joint_state_indices_[i] = static_cast<int>(std::distance(msg->name.begin(), it));
      }
    }
  }

  bool all_mapped = true;
  for (size_t i = 0; i < 5; ++i) {
    const int idx = joint_state_indices_[i];
    if (idx >= 0 && idx < static_cast<int>(msg->position.size())) {
      current_physical_joints_[i] = msg->position[idx];
    } else {
      all_mapped = false;
    }
  }

  if (all_mapped) {
    have_joint_states_ = true;
  }
}

void CartesianJogServerNode::control_loop()
{
  if (has_kinematics_failed()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cartesian jogging is DISABLED due to kinematics initialization failure: %s",
      kinematics_error_msg().c_str());
    return;
  }

  if (!is_kinematics_initialized()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for URDF / robot_description to initialize kinematics...");
    return;
  }

  geometry_msgs::msg::Twist twist;
  rclcpp::Time twist_time;
  std::array<double, 5> physical_joints{};
  bool have_js = false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    twist = latest_twist_;
    twist_time = last_twist_time_;
    physical_joints = current_physical_joints_;
    have_js = have_joint_states_;
  }

  if (!have_js) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for /joint_states telemetry to establish initial configuration...");
    return;
  }

  const double elapsed_twist_s = (now() - twist_time).seconds();
  const bool is_twist_fresh = (elapsed_twist_s >= 0.0 && elapsed_twist_s <= stale_timeout_s_);

  std::lock_guard<std::mutex> lock(kinematics_mutex_);

  const bool is_busy = is_arm_busy_.load(std::memory_order_acquire);
  if (is_busy) {
    if (active_jogging_) {
      active_jogging_ = false;
      motion_filter_.reset();
    }
    // Bumpless Transfer: Continuous synchronization from physical joint telemetry while suppressed
    jogger_->sync_from_fk(physical_joints);
    return;
  }

  if (!is_twist_fresh) {
    if (active_jogging_) {
      RCLCPP_INFO(get_logger(), "Cartesian jog stream timed out (> %.2fs). Idle.", stale_timeout_s_);
      active_jogging_ = false;
      motion_filter_.reset();
    }
    // Bumpless Transfer: Continuous synchronization from physical joint telemetry while idle
    jogger_->sync_from_fk(physical_joints);
    return;
  }

  // Active Jogging
  const double dt = 1.0 / publish_rate_hz_;
  CartesianDelta delta;
  delta.dx = twist.linear.x * dt;
  delta.dy = twist.linear.y * dt;
  delta.dz = twist.linear.z * dt;
  delta.droll = twist.angular.x * dt;
  delta.dpitch = twist.angular.y * dt;

  const auto jog_res = jogger_->process_jog(delta);
  if (!jog_res.success) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Cartesian target infeasible: %s", jog_res.reason.c_str());
  }

  dispatch_command(jog_res.joints, dt);
  active_jogging_ = true;
}

void CartesianJogServerNode::dispatch_command(
  const std::array<double, 5> & target_joints, double dt)
{
  const auto filtered = motion_filter_.update(target_joints, dt);

  if (controller_type_ == "forward_position" && pub_forward_commands_) {
    std_msgs::msg::Float64MultiArray fwd_msg;
    fwd_msg.data.assign(filtered.positions.begin(), filtered.positions.end());
    pub_forward_commands_->publish(fwd_msg);
  } else if (pub_trajectory_) {
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = now();
    traj_msg.joint_names = arm_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(filtered.positions.begin(), filtered.positions.end());
    pt.velocities.assign(filtered.velocities.begin(), filtered.velocities.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(point_dt_s_);
    traj_msg.points.push_back(std::move(pt));

    pub_trajectory_->publish(traj_msg);
  }
}

}  // namespace lekiwi_motion::arm

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::arm::CartesianJogServerNode)


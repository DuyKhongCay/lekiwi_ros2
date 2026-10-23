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
 * @brief Implementation of standalone Cartesian jog server and interactive services.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/arm/cartesian_jog_server_node.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace lekiwi_motion::arm
{

CartesianJogServerNode::CartesianJogServerNode(const rclcpp::NodeOptions & options)
: Node("cartesian_jog_server_node", options)
{
  cb_group_services_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  declare_all_parameters();
  init_executor_and_kinematics();

  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianJogServerNode::cmd_vel_callback, this, std::placeholders::_1));

  named_pose_srv_ = create_service<GoToNamedPose>(
    "~/go_to_named_pose",
    std::bind(
      &CartesianJogServerNode::handle_named_pose_request, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_services_);

  toggle_gripper_srv_ = create_service<Trigger>(
    "~/toggle_gripper",
    std::bind(
      &CartesianJogServerNode::handle_toggle_gripper, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_services_);

  if (controller_mode_ == ControllerMode::TRAJECTORY) {
    pub_trajectory_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      arm_trajectory_topic_, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      get_logger(),
      "CartesianJogServerNode initialized in TRAJECTORY mode at %.1f Hz. Streaming JointTrajectory to '%s'",
      publish_rate_hz_, arm_trajectory_topic_.c_str());
  } else {
    pub_forward_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      arm_forward_topic_, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      get_logger(),
      "CartesianJogServerNode initialized in FORWARD mode at %.1f Hz. Streaming Float64MultiArray to '%s'",
      publish_rate_hz_, arm_forward_topic_.c_str());
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CartesianJogServerNode::control_loop, this));
}

CartesianJogServerNode::~CartesianJogServerNode()
{
  if (executor_) {
    executor_->stop();
  }
}

void CartesianJogServerNode::declare_all_parameters()
{
  publish_rate_hz_ = std::max(1.0, declare_or_get_param<double>(this, "publish_rate_hz", 50.0));
  point_dt_s_ = std::max(0.01, declare_or_get_param<double>(this, "point_dt_s", 0.08));
  stale_timeout_s_ = std::max(0.01, declare_or_get_param<double>(this, "stale_timeout_s", 0.25));
  lpf_alpha_ = std::clamp(declare_or_get_param<double>(this, "lpf_alpha", 0.85), 0.0, 1.0);
  max_joint_vel_rad_s_ = std::max(0.01, declare_or_get_param<double>(this, "max_joint_vel_rad_s", 2.0));
  base_frame_ = declare_or_get_param<std::string>(this, "base_frame", "base_footprint");

  const std::string mode_str = declare_or_get_param<std::string>(this, "controller_mode", "trajectory");
  if (mode_str == "trajectory") {
    controller_mode_ = ControllerMode::TRAJECTORY;
  } else if (mode_str == "forward") {
    controller_mode_ = ControllerMode::FORWARD;
  } else {
    throw std::invalid_argument(
      "Invalid controller_mode: '" + mode_str + "'. Expected 'trajectory' or 'forward'.");
  }

  arm_trajectory_topic_ = declare_or_get_param<std::string>(
    this, "arm_trajectory_topic", "/arm_trajectory_controller/joint_trajectory");
  arm_forward_topic_ = declare_or_get_param<std::string>(
    this, "arm_forward_topic", "/arm_forward_controller/commands");

  motion_filter_ = JointMotionFilter<5>(lpf_alpha_, max_joint_vel_rad_s_);
}

void CartesianJogServerNode::init_executor_and_kinematics()
{
  ArmTrajectoryExecutorConfig exec_cfg;
  exec_cfg.base_frame = base_frame_;
  executor_ = std::make_unique<ArmTrajectoryExecutor>(this, std::move(exec_cfg));

  const auto & arm_joints = executor_->arm_joints();
  if (arm_joints.size() >= 5) {
    jog_joint_names_.assign(arm_joints.begin(), arm_joints.begin() + 5);
  }

  executor_->set_kinematics_callback(
    [this](const std::shared_ptr<workspace::SO101AnalyticalSolver> & solver,
           const workspace::KinematicsModel & model) {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      ik_solver_ = solver;
      kinematics_model_ = model;
      jogger_ = std::make_unique<CartesianJogger>(ik_solver_, kinematics_model_);
    });
}

bool CartesianJogServerNode::is_kinematics_initialized() const noexcept
{
  return executor_ ? executor_->is_kinematics_initialized() : false;
}

bool CartesianJogServerNode::has_kinematics_failed() const noexcept
{
  return executor_ ? executor_->has_kinematics_failed() : false;
}

std::string CartesianJogServerNode::kinematics_error_msg() const
{
  return executor_ ? executor_->kinematics_error_msg() : "";
}

bool CartesianJogServerNode::is_active() const noexcept
{
  return active_jogging_;
}

bool CartesianJogServerNode::is_executing_trajectory() const noexcept
{
  return executor_ ? executor_->is_trajectory_busy() : false;
}

void CartesianJogServerNode::cmd_vel_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_twist_ = msg->twist;
  last_twist_time_ = now();
}

bool CartesianJogServerNode::check_jog_prerequisites()
{
  if (has_kinematics_failed()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cartesian jogging is DISABLED due to kinematics initialization failure: %s",
      kinematics_error_msg().c_str());
    return false;
  }
  if (!is_kinematics_initialized()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for URDF / robot_description to initialize kinematics...");
    return false;
  }
  if (!executor_ || !executor_->has_joint_states()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for /joint_states telemetry to establish initial configuration...");
    return false;
  }
  return true;
}

void CartesianJogServerNode::control_loop()
{
  if (!check_jog_prerequisites()) {
    return;
  }

  const auto arm_joints_opt = executor_->get_active_arm_positions();
  if (!arm_joints_opt.has_value()) {
    return;
  }
  const auto physical_joints = *arm_joints_opt;

  geometry_msgs::msg::Twist twist;
  rclcpp::Time twist_time;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    twist = latest_twist_;
    twist_time = last_twist_time_;
  }

  const double elapsed_twist_s = (now() - twist_time).seconds();
  const bool is_twist_fresh = (elapsed_twist_s >= 0.0 && elapsed_twist_s <= stale_timeout_s_);

  std::lock_guard<std::mutex> lock(kinematics_mutex_);

  if (is_executing_trajectory() || !is_twist_fresh) {
    handle_suppressed_or_idle(physical_joints, !is_twist_fresh);
    return;
  }

  execute_active_jog(twist, physical_joints);
}

void CartesianJogServerNode::handle_suppressed_or_idle(
  const std::array<double, 5> & physical_joints,
  bool is_idle_timeout)
{
  if (active_jogging_) {
    if (is_idle_timeout) {
      RCLCPP_INFO(get_logger(), "Cartesian jog stream timed out (> %.2fs). Idle.", stale_timeout_s_);
    }
    active_jogging_ = false;
  }
  if (jogger_) {
    jogger_->sync_from_fk(physical_joints);
  }
  motion_filter_.sync(physical_joints);
}

void CartesianJogServerNode::execute_active_jog(
  const geometry_msgs::msg::Twist & twist,
  const std::array<double, 5> & physical_joints)
{
  if (!jogger_) {
    return;
  }
  const double dt = 1.0 / publish_rate_hz_;
  CartesianDelta delta;
  delta.dx = twist.linear.x * dt;
  delta.dy = twist.linear.y * dt;
  delta.dz = twist.linear.z * dt;
  delta.dpitch = twist.angular.y * dt;
  delta.droll = twist.angular.x * dt;

  if (!active_jogging_) {
    jogger_->sync_from_fk(physical_joints);
    motion_filter_.sync(physical_joints);
    active_jogging_ = true;
  }

  const auto jog_res = jogger_->process_jog(delta);
  if (!jog_res.success) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Cartesian target infeasible: %s", jog_res.reason.c_str());
    return;
  }

  const auto filtered = motion_filter_.update(jog_res.joints, dt);
  dispatch_command(filtered.positions, dt);
}

void CartesianJogServerNode::dispatch_command(
  const std::array<double, 5> & target_joints,
  double dt)
{
  if (controller_mode_ == ControllerMode::TRAJECTORY) {
    if (!pub_trajectory_) {
      return;
    }
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.header.frame_id = base_frame_;
    traj.joint_names = jog_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(target_joints.begin(), target_joints.end());
    const double pt_time = std::max(point_dt_s_, dt * 1.5);
    pt.time_from_start = rclcpp::Duration::from_seconds(pt_time);
    traj.points.push_back(std::move(pt));

    pub_trajectory_->publish(traj);
  } else if (controller_mode_ == ControllerMode::FORWARD) {
    if (!pub_forward_cmd_) {
      return;
    }
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.reserve(6);
    cmd.data.insert(cmd.data.end(), target_joints.begin(), target_joints.end());

    double gripper_pos = 0.0;
    if (executor_) {
      const auto current_pos = executor_->get_current_positions_or_default();
      if (!current_pos.empty()) {
        gripper_pos = current_pos.back();
      }
    }
    cmd.data.push_back(gripper_pos);

    pub_forward_cmd_->publish(cmd);
  }
}

void CartesianJogServerNode::handle_named_pose_request(
  const std::shared_ptr<GoToNamedPose::Request> request,
  std::shared_ptr<GoToNamedPose::Response> response)
{
  if (!request || request->pose_name.empty()) {
    response->success = false;
    response->message = "Invalid empty pose name";
    return;
  }

  std::string key = request->pose_name;
  std::transform(key.begin(), key.end(), key.begin(), ::tolower);
  if (key == "approach") {
    response->success = false;
    response->message =
      "Pose 'approach' is dynamically managed by /manipulation_action_server/go_to_named_pose";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto [ok, msg] = executor_->execute_named_pose(request->pose_name);
  response->success = ok;
  response->message = msg;
  if (!ok) {
    RCLCPP_WARN(get_logger(), "Rejected named pose request '%s': %s",
      request->pose_name.c_str(), msg.c_str());
  }
}

void CartesianJogServerNode::handle_toggle_gripper(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  const auto current_pos = executor_->get_current_positions_or_default();
  if (current_pos.empty()) {
    response->success = false;
    response->message = "No joint telemetry available";
    return;
  }

  const double curr_gripper = current_pos.back();
  const double target_gripper = executor_->gripper_config().toggled_target(curr_gripper);
  auto target_pos = current_pos;
  target_pos.back() = target_gripper;

  auto [ok, msg] = executor_->execute_joint_target(target_pos, "toggle_gripper");
  response->success = ok;
  response->message = msg;
}

}  // namespace lekiwi_motion::arm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::arm::CartesianJogServerNode)


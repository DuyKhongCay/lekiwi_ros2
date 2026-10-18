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

#include "lekiwi_motion/arm/arm_trajectory_executor.hpp"

#include <chrono>
#include <future>
#include <thread>
#include <utility>

#include <tf2/exceptions.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

namespace lekiwi_motion::arm
{

namespace
{

[[nodiscard]] double compute_trajectory_duration_sec(
  const trajectory_msgs::msg::JointTrajectory & traj) noexcept
{
  if (traj.points.empty()) {
    return 1.0;
  }
  const auto & t = traj.points.back().time_from_start;
  const double sec = static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  return (sec > 0.0) ? sec : 1.0;
}

}  // namespace

ArmTrajectoryExecutor::ArmTrajectoryExecutor(
  rclcpp::Node * node,
  ArmTrajectoryExecutorConfig config)
: node_(node),
  config_(std::move(config))
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & j : config_.arm_joints) {
      current_joints_[j] = 0.0;
    }
  }
  init_trajectory_client_and_subscribers();
  init_kinematics_and_tf();
}

ArmTrajectoryExecutor::~ArmTrajectoryExecutor()
{
  stop();
}

void ArmTrajectoryExecutor::stop() noexcept
{
  is_shutting_down_.store(true, std::memory_order_release);
}

void ArmTrajectoryExecutor::init_trajectory_client_and_subscribers()
{
  cb_group_controller_client_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_joint_state_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_joint_state_;

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_states_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ArmTrajectoryExecutor::on_joint_state, this, std::placeholders::_1),
    sub_options);

  if (config_.mock_execution) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ArmTrajectoryExecutor initialized in MOCK mode (simulated duration enabled)");
  } else {
    controller_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_,
      config_.controller_action_name,
      cb_group_controller_client_);
    RCLCPP_INFO(
      node_->get_logger(),
      "ArmTrajectoryExecutor connected to controller action: '%s'",
      config_.controller_action_name.c_str());
  }
}

void ArmTrajectoryExecutor::init_kinematics_and_tf()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  qos_profile.reliable();
  robot_desc_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "robot_description",
    qos_profile,
    std::bind(&ArmTrajectoryExecutor::on_robot_description, this, std::placeholders::_1));
}

void ArmTrajectoryExecutor::on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg) {
    accept_robot_description(msg->data);
  }
}

void ArmTrajectoryExecutor::accept_robot_description(const std::string & urdf_xml)
{
  try {
    auto urdf_model = urdf::parseURDF(urdf_xml);
    if (!urdf_model) {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse robot_description XML");
      return;
    }
    if (config_.arm_joints.size() < 5) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Kinematics extraction failed: Expected at least 5 arm joints, but only %zu configured.",
        config_.arm_joints.size());
      return;
    }

    workspace::KinematicsModel new_model;
    std::string err;
    const std::vector<std::string> arm_5_joints(
      config_.arm_joints.begin(), config_.arm_joints.begin() + 5);

    if (!workspace::extract_kinematics_from_urdf(
        *urdf_model, config_.base_frame, config_.tip_frame,
        arm_5_joints, new_model, err, 0.05))
    {
      RCLCPP_WARN(node_->get_logger(), "Kinematics extraction failed: %s", err.c_str());
      return;
    }

    auto solver = std::make_unique<workspace::SO101AnalyticalSolver>(new_model);
    const double reach = solver->reach_bound();
    {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      analytical_solver_ = std::move(solver);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "ManipulationActionServer IK solver ready (reach: %.3f m)",
      reach);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Exception during kinematics initialization: %s", e.what());
  }
}

void ArmTrajectoryExecutor::on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    auto it = current_joints_.find(msg->name[i]);
    if (it != current_joints_.end()) {
      it->second = msg->position[i];
    }
  }
}

std::vector<double> ArmTrajectoryExecutor::get_current_positions_or_default() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<double> pos(config_.arm_joints.size(), 0.0);
  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    auto it = current_joints_.find(config_.arm_joints[i]);
    if (it != current_joints_.end()) {
      pos[i] = it->second;
    }
  }
  return pos;
}

std::optional<geometry_msgs::msg::PointStamped> ArmTrajectoryExecutor::transform_to_base(
  const geometry_msgs::msg::PointStamped & in_pt) const
{
  if (in_pt.header.frame_id == config_.base_frame) {
    return in_pt;
  }
  if (!tf_buffer_) {
    RCLCPP_WARN(node_->get_logger(), "TF buffer unavailable for JIT IK resolution");
    return std::nullopt;
  }
  try {
    return tf_buffer_->transform(in_pt, config_.base_frame, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(), "TF transform failed (%s -> %s): %s",
      in_pt.header.frame_id.c_str(), config_.base_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<std::vector<double>> ArmTrajectoryExecutor::resolve_point_ik(
  const geometry_msgs::msg::Point & point,
  const std::string & target_frame,
  double clamped_gripper_rad) const
{
  geometry_msgs::msg::PointStamped in_pt;
  in_pt.header.frame_id = target_frame.empty() ? config_.board_frame : target_frame;
  in_pt.point = point;

  const auto out_pt = transform_to_base(in_pt);
  if (!out_pt.has_value()) {
    return std::nullopt;
  }

  workspace::IkResult ik_res;
  {
    std::lock_guard<std::mutex> lock(kinematics_mutex_);
    if (!analytical_solver_) {
      RCLCPP_WARN(node_->get_logger(), "SO101AnalyticalSolver not initialized");
      return std::nullopt;
    }
    ik_res = analytical_solver_->solve(
      out_pt->point.x, out_pt->point.y, out_pt->point.z,
      config_.default_pitch, config_.default_roll);
  }

  if (!ik_res.success) {
    RCLCPP_WARN(
      node_->get_logger(), "JIT IK unsolvable for target (%.3f, %.3f, %.3f) in frame '%s'",
      out_pt->point.x, out_pt->point.y, out_pt->point.z, config_.base_frame.c_str());
    return std::nullopt;
  }

  std::vector<double> joint_positions(config_.arm_joints.size(), 0.0);
  for (size_t i = 0; i < 5 && i < config_.arm_joints.size(); ++i) {
    joint_positions[i] = ik_res.joints[i];
  }
  if (!joint_positions.empty()) {
    joint_positions.back() = clamped_gripper_rad;
  }
  return joint_positions;
}

std::pair<bool, std::string> ArmTrajectoryExecutor::execute_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::function<bool()> is_canceling)
{
  if (config_.mock_execution) {
    return execute_mock_trajectory(traj, is_canceling);
  }
  return execute_controller_trajectory(traj, is_canceling);
}

std::pair<bool, std::string> ArmTrajectoryExecutor::execute_mock_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  const std::function<bool()> & is_canceling)
{
  if (traj.points.empty()) {
    return {true, "Empty trajectory"};
  }

  const double effective_duration = compute_trajectory_duration_sec(traj);
  const auto & target_positions = traj.points.back().positions;
  const auto start_time = std::chrono::steady_clock::now();
  constexpr auto kPollInterval = std::chrono::milliseconds(50);

  while (!is_shutting_down_.load(std::memory_order_acquire)) {
    if (is_canceling && is_canceling()) {
      return {false, "Preempted and canceled"};
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    if (elapsed >= effective_duration) {
      break;
    }
    std::this_thread::sleep_for(kPollInterval);
  }

  if (is_shutting_down_.load(std::memory_order_acquire)) {
    return {false, "Server shutting down"};
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < config_.arm_joints.size() && i < target_positions.size(); ++i) {
      current_joints_[config_.arm_joints[i]] = target_positions[i];
    }
  }
  return {true, "Success"};
}

std::pair<bool, std::string> ArmTrajectoryExecutor::execute_controller_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  const std::function<bool()> & is_canceling)
{
  if (!controller_client_) {
    return {false, "Controller client uninitialized"};
  }
  if (!controller_client_->wait_for_action_server(std::chrono::seconds(2))) {
    return {false, "Controller action server unavailable: " + config_.controller_action_name};
  }

  FollowJointTrajectory::Goal goal_msg;
  goal_msg.trajectory = traj;

  auto send_future = controller_client_->async_send_goal(goal_msg);
  if (send_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return {false, "Send goal to controller timed out"};
  }

  auto handle = send_future.get();
  if (!handle) {
    return {false, "Trajectory rejected by controller"};
  }

  return wait_for_controller_result(handle, compute_trajectory_duration_sec(traj), is_canceling);
}

std::pair<bool, std::string> ArmTrajectoryExecutor::wait_for_controller_result(
  const GoalHandleFJT::SharedPtr & handle,
  double expected_sec,
  const std::function<bool()> & is_canceling)
{
  auto result_future = controller_client_->async_get_result(handle);
  const double timeout_sec = expected_sec + 5.0;
  const auto start_wait = std::chrono::steady_clock::now();

  while (!is_shutting_down_.load(std::memory_order_acquire)) {
    if (is_canceling && is_canceling()) {
      controller_client_->async_cancel_goal(handle);
      return {false, "Preempted and canceled by client"};
    }
    if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      break;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_wait).count();
    if (elapsed > timeout_sec) {
      controller_client_->async_cancel_goal(handle);
      return {false, "Trajectory execution timed out"};
    }
  }

  if (is_shutting_down_.load(std::memory_order_acquire)) {
    return {false, "Server shutting down"};
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return {
      false,
      "Trajectory controller failed with code: " +
        std::to_string(static_cast<int>(wrapped_result.code))
    };
  }
  return {true, "Success"};
}

}  // namespace lekiwi_motion::arm

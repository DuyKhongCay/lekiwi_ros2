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

#include <algorithm>
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
  config_(std::move(config)),
  motion_planner_(
    config_.arm_joints,
    config_.gripper_cfg,
    config_.named_poses,
    config_.sampling_rate_hz)
{
  init_parameters();
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

void ArmTrajectoryExecutor::init_parameters()
{
  if (!node_) {
    return;
  }

  config_.arm_joints = declare_or_get_param<std::vector<std::string>>(
    node_, "arm_joints", config_.arm_joints);
  config_.default_velocity = declare_or_get_param<double>(
    node_, "default_velocity", config_.default_velocity);
  config_.sampling_rate_hz = declare_or_get_param<double>(
    node_, "sampling_rate_hz", config_.sampling_rate_hz);
  config_.controller_action_name = declare_or_get_param<std::string>(
    node_, "controller_action_name", config_.controller_action_name);
  config_.joint_states_topic = declare_or_get_param<std::string>(
    node_, "joint_states_topic", config_.joint_states_topic);
  config_.robot_description_topic = declare_or_get_param<std::string>(
    node_, "robot_description_topic", config_.robot_description_topic);
  config_.board_frame = declare_or_get_param<std::string>(
    node_, "board_frame", config_.board_frame);
  config_.base_frame = declare_or_get_param<std::string>(
    node_, "base_frame", config_.base_frame);
  config_.tip_frame = declare_or_get_param<std::string>(
    node_, "tip_frame", config_.tip_frame);
  config_.default_pitch = declare_or_get_param<double>(
    node_, "planning.default_pitch", config_.default_pitch);
  config_.default_roll = declare_or_get_param<double>(
    node_, "planning.default_roll", config_.default_roll);
  config_.mock_execution = declare_or_get_param<bool>(
    node_, "mock_execution", config_.mock_execution);

  config_.gripper_cfg.open_rad = declare_or_get_param<double>(
    node_, "gripper.open", config_.gripper_cfg.open_rad);
  config_.gripper_cfg.closed_rad = declare_or_get_param<double>(
    node_, "gripper.closed", config_.gripper_cfg.closed_rad);

  config_.named_poses.approach = declare_or_get_param<std::vector<double>>(
    node_, "named_poses.approach", config_.named_poses.approach);
  config_.named_poses.stow = declare_or_get_param<std::vector<double>>(
    node_, "named_poses.stow", config_.named_poses.stow);
  config_.named_poses.clear_left = declare_or_get_param<std::vector<double>>(
    node_, "named_poses.clear_left", config_.named_poses.clear_left);
  config_.named_poses.clear_right = declare_or_get_param<std::vector<double>>(
    node_, "named_poses.clear_right", config_.named_poses.clear_right);

  std::lock_guard<std::mutex> lock(planner_mutex_);
  motion_planner_ = arm::ArmMotionPlanner(
    config_.arm_joints,
    config_.gripper_cfg,
    config_.named_poses,
    config_.sampling_rate_hz);
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
    rclcpp::SensorDataQoS(),
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
    config_.robot_description_topic,
    qos_profile,
    std::bind(&ArmTrajectoryExecutor::on_robot_description, this, std::placeholders::_1));

  const std::string urdf_param = declare_or_get_param<std::string>(
    node_, "robot_description", "");
  if (!urdf_param.empty()) {
    if (!accept_robot_description(urdf_param)) {
      throw std::runtime_error(
        "ArmTrajectoryExecutor: Failed to initialize kinematics from 'robot_description' parameter: " +
        kinematics_error_msg());
    }
  }
}

void ArmTrajectoryExecutor::on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg && !msg->data.empty()) {
    accept_robot_description(msg->data);
  }
}

bool ArmTrajectoryExecutor::accept_robot_description(const std::string & urdf_xml)
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  if (is_kinematics_initialized_.load(std::memory_order_relaxed)) {
    return true;
  }
  try {
    auto urdf_model = urdf::parseURDF(urdf_xml);
    if (!urdf_model) {
      has_kinematics_failed_.store(true, std::memory_order_release);
      kinematics_error_msg_ = "Failed to parse robot_description XML";
      RCLCPP_WARN(node_->get_logger(), "%s", kinematics_error_msg_.c_str());
      return false;
    }
    if (config_.arm_joints.size() < 5) {
      has_kinematics_failed_.store(true, std::memory_order_release);
      kinematics_error_msg_ = "Expected at least 5 arm joints, got " +
        std::to_string(config_.arm_joints.size());
      RCLCPP_ERROR(node_->get_logger(), "%s", kinematics_error_msg_.c_str());
      return false;
    }

    workspace::KinematicsModel new_model;
    std::string err;
    const std::vector<std::string> arm_5_joints(
      config_.arm_joints.begin(), config_.arm_joints.begin() + 5);

    if (!workspace::extract_kinematics_from_urdf(
        *urdf_model, config_.base_frame, config_.tip_frame,
        arm_5_joints, new_model, err, 0.05))
    {
      has_kinematics_failed_.store(true, std::memory_order_release);
      kinematics_error_msg_ = "Kinematics extraction failed: " + err;
      RCLCPP_WARN(node_->get_logger(), "%s", kinematics_error_msg_.c_str());
      return false;
    }

    kinematics_model_ = new_model;
    analytical_solver_ = std::make_shared<workspace::SO101AnalyticalSolver>(kinematics_model_);
    is_kinematics_initialized_.store(true, std::memory_order_release);
    has_kinematics_failed_.store(false, std::memory_order_release);
    kinematics_error_msg_.clear();

    const double reach = analytical_solver_->reach_bound();
    RCLCPP_INFO(
      node_->get_logger(),
      "Kinematics & Analytical IK initialized successfully (reach envelope: %.3f m)",
      reach);

    if (kinematics_callback_) {
      kinematics_callback_(analytical_solver_, kinematics_model_);
    }
    return true;
  } catch (const std::exception & e) {
    has_kinematics_failed_.store(true, std::memory_order_release);
    kinematics_error_msg_ = "Exception during kinematics initialization: " + std::string(e.what());
    RCLCPP_WARN(node_->get_logger(), "%s", kinematics_error_msg_.c_str());
    return false;
  }
}

void ArmTrajectoryExecutor::set_kinematics_callback(KinematicsCallback cb)
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  kinematics_callback_ = std::move(cb);
  if (is_kinematics_initialized_.load(std::memory_order_acquire) && kinematics_callback_) {
    kinematics_callback_(analytical_solver_, kinematics_model_);
  }
}

std::string ArmTrajectoryExecutor::kinematics_error_msg() const
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  return kinematics_error_msg_;
}

std::shared_ptr<workspace::SO101AnalyticalSolver> ArmTrajectoryExecutor::analytical_solver() const
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  return analytical_solver_;
}

workspace::KinematicsModel ArmTrajectoryExecutor::kinematics_model() const
{
  std::lock_guard<std::mutex> lock(kinematics_mutex_);
  return kinematics_model_;
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

  if (!has_joint_states_.load(std::memory_order_relaxed)) {
    if (config_.arm_joints.size() >= 5) {
      bool all_found = true;
      for (size_t i = 0; i < 5; ++i) {
        bool found = false;
        for (const auto & name : msg->name) {
          if (name == config_.arm_joints[i]) {
            found = true;
            break;
          }
        }
        if (!found) {
          all_found = false;
          break;
        }
      }
      if (all_found) {
        has_joint_states_.store(true, std::memory_order_release);
      }
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

std::optional<std::array<double, 5>> ArmTrajectoryExecutor::get_active_arm_positions() const
{
  if (!has_joint_states()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (config_.arm_joints.size() < 5) {
    return std::nullopt;
  }
  std::array<double, 5> pos{};
  for (size_t i = 0; i < 5; ++i) {
    auto it = current_joints_.find(config_.arm_joints[i]);
    if (it == current_joints_.end()) {
      return std::nullopt;
    }
    pos[i] = it->second;
  }
  return pos;
}

std::optional<std::vector<double>> ArmTrajectoryExecutor::get_named_pose(
  const std::string & name) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.get_named_pose(name);
}

GripperConfig ArmTrajectoryExecutor::gripper_config() const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.gripper_config();
}

double ArmTrajectoryExecutor::clamp_gripper(double target_rad) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.clamp_gripper(target_rad);
}

double ArmTrajectoryExecutor::gripper_open_rad() const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.gripper_open_rad();
}

double ArmTrajectoryExecutor::gripper_closed_rad() const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.gripper_closed_rad();
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

std::unordered_map<std::string, std::vector<double>> ArmTrajectoryExecutor::build_phase_targets(
  const std::vector<double> & pick_approach,
  const std::vector<double> & pick_descend,
  const std::vector<double> & place_approach,
  const std::vector<double> & place_descend) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.build_phase_targets(
    pick_approach, pick_descend, place_approach, place_descend);
}

std::unordered_map<std::string, std::vector<double>> ArmTrajectoryExecutor::build_clear_targets(
  const std::vector<double> & pick_approach,
  const std::vector<double> & pick_descend,
  arm::ClearBinSide side) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.build_clear_targets(pick_approach, pick_descend, side);
}

std::unordered_map<std::string, std::vector<double>> ArmTrajectoryExecutor::build_pick_targets(
  const std::vector<double> & pick_approach,
  const std::vector<double> & pick_descend) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.build_pick_targets(pick_approach, pick_descend);
}

std::unordered_map<std::string, std::vector<double>> ArmTrajectoryExecutor::build_place_targets(
  const std::vector<double> & place_approach,
  const std::vector<double> & place_descend) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return motion_planner_.build_place_targets(place_approach, place_descend);
}

trajectory_msgs::msg::JointTrajectory ArmTrajectoryExecutor::plan_trajectory_at_velocity(
  const std::vector<double> & start_joints,
  const std::vector<double> & target_joints,
  double target_velocity_rad_s) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  const double vel = (target_velocity_rad_s > 0.0) ? target_velocity_rad_s : config_.default_velocity;
  return motion_planner_.plan_trajectory_at_velocity(
    start_joints, target_joints, vel, config_.sampling_rate_hz);
}

std::pair<bool, std::string> ArmTrajectoryExecutor::execute_joint_target(
  const std::vector<double> & target_positions,
  const std::string & label,
  double velocity,
  std::function<bool()> is_canceling)
{
  if (is_shutting_down_.load(std::memory_order_acquire)) {
    return {false, "Executor shutting down"};
  }

  if (target_positions.size() != config_.arm_joints.size()) {
    return {false, "Target joints dimension mismatch"};
  }

  const auto current_pos = get_current_positions_or_default();
  trajectory_msgs::msg::JointTrajectory traj;
  try {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    const double vel = (velocity > 0.0) ? velocity : config_.default_velocity;
    traj = motion_planner_.plan_trajectory_at_velocity(
      current_pos, target_positions, vel, config_.sampling_rate_hz);
  } catch (const std::exception & exc) {
    return {false, "Trajectory planning failed for '" + label + "': " + exc.what()};
  }

  auto [ok, msg] = execute_trajectory(traj, is_canceling);
  if (!ok) {
    return {false, "Execution failed for '" + label + "': " + msg};
  }
  return {true, "Successfully executed target: " + label};
}

std::pair<bool, std::string> ArmTrajectoryExecutor::execute_named_pose(
  const std::string & name,
  double velocity,
  std::function<bool()> is_canceling)
{
  const auto target_opt = get_named_pose(name);
  if (!target_opt.has_value()) {
    return {false, "Unknown named pose: " + name};
  }

  auto [ok, msg] = execute_joint_target(*target_opt, name, velocity, is_canceling);
  if (!ok) {
    return {false, msg};
  }
  return {true, "Successfully moved to pose: " + name};
}

std::pair<bool, std::string> ArmTrajectoryExecutor::execute_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::function<bool()> is_canceling)
{
  if (is_shutting_down_.load(std::memory_order_acquire)) {
    return {false, "Executor shutting down"};
  }

  TrajectoryExecutionGuard guard(is_trajectory_busy_);
  if (!guard.acquired()) {
    return {false, "Arm is currently busy moving a trajectory"};
  }

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

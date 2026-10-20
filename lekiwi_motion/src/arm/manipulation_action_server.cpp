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

#include "lekiwi_motion/arm/manipulation_action_server.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace lekiwi_motion
{

namespace
{

template <typename T>
T get_or_declare_param(rclcpp::Node * node, const std::string & name, const T & default_val)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_val);
  }
  return node->get_parameter(name).get_value<T>();
}

[[nodiscard]] float compute_elapsed_sec(std::chrono::steady_clock::time_point start_time) noexcept
{
  return static_cast<float>(
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());
}

}  // namespace

ManipulationActionServer::ManipulationActionServer(const rclcpp::NodeOptions & options)
: Node("manipulation_action_server", options),
  diagnostic_updater_(this)
{
  declare_and_init_parameters();

  cb_group_action_server_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_recorder_coordination_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  diagnostic_updater_.setHardwareID("lekiwi_motion_manipulation");
  diagnostic_updater_.add(
    "manipulation_status", this, &ManipulationActionServer::produce_diagnostics);

  action_server_ = rclcpp_action::create_server<ExecuteChessMove>(
    this,
    action_name_,
    std::bind(
      &ManipulationActionServer::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&ManipulationActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ManipulationActionServer::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    cb_group_action_server_);

  named_pose_srv_ = create_service<GoToNamedPose>(
    "/arm/go_to_named_pose",
    std::bind(
      &ManipulationActionServer::handle_named_pose_request, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_action_server_);

  toggle_gripper_srv_ = create_service<Trigger>(
    "/arm/toggle_gripper",
    std::bind(
      &ManipulationActionServer::handle_toggle_gripper, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_action_server_);

  save_episode_srv_ = create_service<Trigger>(
    "/arm/save_episode",
    std::bind(
      &ManipulationActionServer::handle_save_episode, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_recorder_coordination_);

  abort_episode_srv_ = create_service<Trigger>(
    "/arm/abort_episode",
    std::bind(
      &ManipulationActionServer::handle_abort_episode, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_recorder_coordination_);

  save_client_ = create_client<Trigger>(
    "/episode_recorder_node/save_episode",
    rclcpp::ServicesQoS(),
    cb_group_recorder_coordination_);

  discard_client_ = create_client<Trigger>(
    "/episode_recorder_node/discard_episode",
    rclcpp::ServicesQoS(),
    cb_group_recorder_coordination_);

  rclcpp::QoS busy_qos(1);
  busy_qos.transient_local();
  busy_qos.reliable();
  pub_arm_busy_ = create_publisher<std_msgs::msg::Bool>(arm_busy_topic_, busy_qos);
  publish_arm_busy(false);
}

ManipulationActionServer::~ManipulationActionServer()
{
  is_shutting_down_.store(true, std::memory_order_release);
  if (executor_) {
    executor_->stop();
  }
  record_cv_.notify_all();
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

void ManipulationActionServer::declare_and_init_parameters()
{
  action_name_ = get_or_declare_param<std::string>(
    this, "action_name", "/manipulation/execute_chess_move");
  approach_z_offset_ = get_or_declare_param<double>(this, "approach_z_offset", 0.05);
  controller_action_name_ = get_or_declare_param<std::string>(
    this, "controller_action_name", "/arm_trajectory_controller/follow_joint_trajectory");
  arm_busy_topic_ = get_or_declare_param<std::string>(
    this, "arm_busy_topic", "/arm/is_busy");
  const auto joint_states_topic = get_or_declare_param<std::string>(
    this, "joint_states_topic", "/joint_states");

  default_velocity_ = get_or_declare_param<double>(this, "default_velocity", 1.5);
  if (default_velocity_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Parameter 'default_velocity' must be positive, got %.2f. Defaulting to 1.5 rad/s",
      default_velocity_);
    default_velocity_ = 1.5;
  }
  sampling_rate_hz_ = get_or_declare_param<double>(this, "sampling_rate_hz", 50.0);
  mock_execution_ = get_or_declare_param<bool>(this, "mock_execution", false);
  record_ = get_or_declare_param<bool>(this, "record", false);
  if (record_) {
    RCLCPP_INFO(
      get_logger(),
      "ManipulationActionServer operating in RECORD mode (stopping at APPROACH phase)");
  }

  arm_joints_ = get_or_declare_param<std::vector<std::string>>(
    this, "arm_joints", arm_joints_);

  const double gripper_closed = get_or_declare_param<double>(this, "gripper.closed", 0.00);
  const double gripper_open = get_or_declare_param<double>(this, "gripper.open", 1.50);

  arm::NamedPosesConfig named_poses;
  named_poses.approach = get_or_declare_param<std::vector<double>>(
    this, "named_poses.approach", {0.0, 0.0, 0.0, 0.0, 0.0, gripper_closed});
  named_poses.stow = get_or_declare_param<std::vector<double>>(
    this, "named_poses.stow", {0.0, -1.57, 1.57, 0.75, 0.0, gripper_closed});
  named_poses.clear_left = get_or_declare_param<std::vector<double>>(
    this, "named_poses.clear_left", {1.20, -0.60, 0.80, 0.0, 0.0, gripper_closed});
  named_poses.clear_right = get_or_declare_param<std::vector<double>>(
    this, "named_poses.clear_right", {-1.20, -0.60, 0.80, 0.0, 0.0, gripper_closed});
  default_approach_pose_ = named_poses.approach;

  arm_motion_planner_ = arm::ArmMotionPlanner(
    arm_joints_,
    arm::GripperConfig{gripper_open, gripper_closed},
    std::move(named_poses),
    sampling_rate_hz_);

  arm::ArmTrajectoryExecutorConfig exec_cfg;
  exec_cfg.arm_joints = arm_joints_;
  exec_cfg.controller_action_name = controller_action_name_;
  exec_cfg.joint_states_topic = joint_states_topic;
  exec_cfg.board_frame = get_or_declare_param<std::string>(this, "board_frame", "chessboard_frame");
  exec_cfg.base_frame = get_or_declare_param<std::string>(this, "base_frame", "base_footprint");
  exec_cfg.tip_frame = get_or_declare_param<std::string>(this, "tip_frame", "gripperframe");
  exec_cfg.default_pitch = get_or_declare_param<double>(this, "planning.default_pitch", -M_PI_2);
  exec_cfg.default_roll = get_or_declare_param<double>(this, "planning.default_roll", 0.0);
  exec_cfg.mock_execution = mock_execution_;

  executor_ = std::make_unique<arm::ArmTrajectoryExecutor>(this, std::move(exec_cfg));
}

void ManipulationActionServer::accept_robot_description(const std::string & urdf_xml)
{
  executor_->accept_robot_description(urdf_xml);
}

std::optional<std::vector<double>> ManipulationActionServer::get_named_pose(
  const std::string & pose_name) const
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  return arm_motion_planner_.get_named_pose(pose_name);
}

std::vector<double> ManipulationActionServer::get_current_positions_or_default() const
{
  return executor_->get_current_positions_or_default();
}

std::optional<std::vector<double>> ManipulationActionServer::resolve_point_ik(
  const geometry_msgs::msg::Point & point,
  const std::string & target_frame,
  double gripper_state) const
{
  double clamped_gripper = 0.0;
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    clamped_gripper = arm_motion_planner_.clamp_gripper(gripper_state);
  }
  return executor_->resolve_point_ik(point, target_frame, clamped_gripper);
}

void ManipulationActionServer::set_runtime_approach_pose(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  auto poses = arm_motion_planner_.named_poses();
  poses.approach = joints;
  arm_motion_planner_.set_named_poses(std::move(poses));
}

void ManipulationActionServer::publish_arm_busy(bool is_busy)
{
  if (pub_arm_busy_) {
    std_msgs::msg::Bool msg;
    msg.data = is_busy;
    pub_arm_busy_->publish(msg);
  }
}

arm::TrajectoryExecutionGuard ManipulationActionServer::acquire_trajectory_guard()
{
  return arm::TrajectoryExecutionGuard(
    executor_->busy_flag(),
    [this](bool busy) { publish_arm_busy(busy); });
}

std::optional<std::vector<double>> ManipulationActionServer::validate_named_pose_request(
  const std::shared_ptr<GoToNamedPose::Request> & request,
  const std::shared_ptr<GoToNamedPose::Response> & response) const
{
  if (!request || request->pose_name.empty()) {
    response->success = false;
    response->message = "Invalid empty pose name";
    return std::nullopt;
  }

  const auto target_opt = get_named_pose(request->pose_name);
  if (!target_opt.has_value()) {
    response->success = false;
    response->message = "Unknown named pose: '" + request->pose_name + "'";
    RCLCPP_WARN(
      get_logger(), "Rejected named pose request: Unknown pose '%s'",
      request->pose_name.c_str());
    return std::nullopt;
  }

  const bool in_record_wait = is_waiting_record_.load(std::memory_order_acquire);
  if (is_goal_active_.load(std::memory_order_acquire) && !in_record_wait) {
    response->success = false;
    response->message = "Arm is currently busy executing an automated chess move";
    RCLCPP_WARN(
      get_logger(), "Rejected named pose request '%s': Goal is active",
      request->pose_name.c_str());
    return std::nullopt;
  }

  return target_opt;
}

bool ManipulationActionServer::execute_manual_joint_target(
  const std::vector<double> & start_positions,
  const std::vector<double> & target_positions,
  const std::string & label,
  std::string & out_message)
{
  trajectory_msgs::msg::JointTrajectory traj;
  try {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    traj = arm_motion_planner_.plan_trajectory_at_velocity(
      start_positions, target_positions, default_velocity_, sampling_rate_hz_);
  } catch (const std::exception & exc) {
    out_message = "Trajectory planning failed for '" + label + "': " + exc.what();
    RCLCPP_ERROR(get_logger(), "%s", out_message.c_str());
    return false;
  }

  const auto [ok, err_msg] = executor_->execute_trajectory(traj);
  out_message = ok ?
    ("Successfully moved to '" + label + "'") :
    ("Execution failed for '" + label + "': " + err_msg);
  return ok;
}

void ManipulationActionServer::handle_named_pose_request(
  const std::shared_ptr<GoToNamedPose::Request> request,
  std::shared_ptr<GoToNamedPose::Response> response)
{
  const auto target_opt = validate_named_pose_request(request, response);
  if (!target_opt.has_value()) {
    return;
  }

  auto traj_guard = acquire_trajectory_guard();
  if (!traj_guard.acquired()) {
    response->success = false;
    response->message = "Arm is currently busy moving a trajectory";
    RCLCPP_WARN(
      get_logger(), "Rejected named pose request '%s': Physical trajectory in progress",
      request->pose_name.c_str());
    return;
  }

  const std::string & pose_name = request->pose_name;
  RCLCPP_INFO(get_logger(), "Executing named pose request: '%s'", pose_name.c_str());

  const auto start_positions = executor_->get_current_positions_or_default();
  response->success = execute_manual_joint_target(
    start_positions, *target_opt, pose_name, response->message);
}

void ManipulationActionServer::handle_toggle_gripper(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  const bool in_record_wait = is_waiting_record_.load(std::memory_order_acquire);
  if (is_goal_active_.load(std::memory_order_acquire) && !in_record_wait) {
    response->success = false;
    response->message = "Arm is currently busy executing an automated chess move";
    RCLCPP_WARN(get_logger(), "Rejected toggle_gripper request: Goal is active");
    return;
  }

  auto traj_guard = acquire_trajectory_guard();
  if (!traj_guard.acquired()) {
    response->success = false;
    response->message = "Arm is currently busy moving a trajectory";
    RCLCPP_WARN(get_logger(), "Rejected toggle_gripper request: Physical trajectory in progress");
    return;
  }

  const auto start_positions = executor_->get_current_positions_or_default();
  const double current_gripper_rad = start_positions.back();

  bool was_open = false;
  double target_gripper_rad = 0.0;
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    const auto & gripper_cfg = arm_motion_planner_.gripper_config();
    was_open = gripper_cfg.is_open(current_gripper_rad);
    target_gripper_rad = gripper_cfg.toggled_target(current_gripper_rad);
  }

  auto target_positions = start_positions;
  target_positions.back() = target_gripper_rad;

  const std::string action_label = was_open ? "Gripper closed" : "Gripper opened";
  RCLCPP_INFO(
    get_logger(), "Toggling gripper from %.3f rad (%s) -> %.3f rad (%s)",
    current_gripper_rad, was_open ? "OPEN" : "CLOSED",
    target_gripper_rad, action_label.c_str());

  std::string exec_msg;
  response->success = execute_manual_joint_target(
    start_positions, target_positions, action_label, exec_msg);
  response->message = response->success ? action_label : exec_msg;
}

bool ManipulationActionServer::forward_trigger_request(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const std::string & service_label,
  const std::shared_ptr<Trigger::Response> & response)
{
  if (!client->service_is_ready() && !client->wait_for_service(std::chrono::seconds(2))) {
    response->success = false;
    response->message = "Service " + service_label + " is not available";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return false;
  }

  auto future = client->async_send_request(std::make_shared<Trigger::Request>());
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    response->success = false;
    response->message = "Timed out waiting for " + service_label;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return false;
  }

  const auto resp = future.get();
  response->success = resp->success;
  response->message = resp->message;
  return resp->success;
}

void ManipulationActionServer::notify_record_signal(RecordCoordinationSignal signal) noexcept
{
  std::lock_guard<std::mutex> lock(record_mutex_);
  record_signal_ = signal;
  record_cv_.notify_all();
}

void ManipulationActionServer::discard_active_recording_async() noexcept
{
  if (!discard_client_) {
    return;
  }
  try {
    if (discard_client_->service_is_ready()) {
      discard_client_->async_send_request(std::make_shared<Trigger::Request>());
    } else {
      RCLCPP_WARN(get_logger(), "Discard episode service not ready when cancelling record wait.");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to async call discard_episode: %s", e.what());
  }
}

void ManipulationActionServer::handle_save_episode(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (!is_waiting_record_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "No active manipulation goal in RECORDING_WAIT state";
    RCLCPP_WARN(get_logger(), "Rejecting /arm/save_episode: %s", response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(), "Received /arm/save_episode request. Forwarding to episode_recorder_node...");
  if (forward_trigger_request(save_client_, "/episode_recorder_node/save_episode", response)) {
    notify_record_signal(RecordCoordinationSignal::SAVE);
  }
}

void ManipulationActionServer::handle_abort_episode(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (!is_waiting_record_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "No active manipulation goal in RECORDING_WAIT state";
    RCLCPP_WARN(get_logger(), "Rejecting /arm/abort_episode: %s", response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "Received /arm/abort_episode request. Forwarding to episode_recorder_node/discard_episode...");
  forward_trigger_request(discard_client_, "/episode_recorder_node/discard_episode", response);
  notify_record_signal(RecordCoordinationSignal::ABORT);
}

void ManipulationActionServer::set_snapshot(
  const std::string & phase,
  double progress_percent,
  const std::string & instruction)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.phase = phase;
  snapshot_.progress_percent = progress_percent;
  if (!instruction.empty()) {
    snapshot_.instruction = instruction;
  }
}

rclcpp_action::GoalResponse ManipulationActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExecuteChessMove::Goal> goal)
{
  bool expected = false;
  if (!is_goal_active_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: manipulation server is busy with an active goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(
    get_logger(), "Received manipulation goal: '%s' (%s -> %s)",
    goal->instruction.c_str(), goal->from_square.c_str(), goal->to_square.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleChessMove> /*goal_handle*/)
{
  RCLCPP_WARN(get_logger(), "Manipulation cancel request received.");
  if (is_waiting_record_.load(std::memory_order_acquire)) {
    discard_active_recording_async();
    notify_record_signal(RecordCoordinationSignal::CANCEL);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleChessMove> goal_handle)
{
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
  execution_thread_ = std::thread(&ManipulationActionServer::execute_move, this, goal_handle);
}

void ManipulationActionServer::execute_move(const std::shared_ptr<GoalHandleChessMove> goal_handle)
{
  struct GoalLifecycleGuard
  {
    std::atomic<bool> & goal_flag;
    std::atomic<bool> & wait_flag;
    ~GoalLifecycleGuard()
    {
      wait_flag.store(false, std::memory_order_release);
      goal_flag.store(false, std::memory_order_release);
    }
  } goal_guard{is_goal_active_, is_waiting_record_};

  MoveExecutionContext ctx{
    goal_handle,
    goal_handle->get_goal(),
    std::make_shared<ExecuteChessMove::Feedback>(),
    std::make_shared<ExecuteChessMove::Result>(),
    std::chrono::steady_clock::now()
  };
  ctx.feedback->instruction = ctx.goal->instruction;

  set_snapshot("STARTING", 0.0, ctx.goal->instruction);
  RCLCPP_INFO(get_logger(), "Executing chess move: '%s'", ctx.goal->instruction.c_str());

  const bool is_pure_pick = ctx.goal->to_square.empty();
  const bool is_pure_place = ctx.goal->from_square.empty();

  if (record_) {
    execute_record_mode(ctx, is_pure_place);
    return;
  }

  const auto waypoints = resolve_move_waypoints(ctx, is_pure_pick, is_pure_place);
  if (!waypoints.has_value()) {
    return;
  }

  const auto [phase_names, phase_targets] = build_active_phase_plan(
    ctx.goal, is_pure_pick, is_pure_place, waypoints->pick, waypoints->place);

  const double total_phases = static_cast<double>(phase_names.size());
  for (size_t i = 0; i < phase_names.size(); ++i) {
    const auto & name = phase_names[i];
    if (!execute_phase_step(i, name, phase_targets.at(name), total_phases, ctx)) {
      return;
    }
  }

  succeed_move(ctx, "Move completed successfully.");
}

ManipulationActionServer::RecordCoordinationSignal
ManipulationActionServer::wait_for_record_signal(
  const std::shared_ptr<GoalHandleChessMove> & goal_handle)
{
  std::unique_lock<std::mutex> lock(record_mutex_);
  is_waiting_record_.store(true, std::memory_order_release);
  record_signal_ = RecordCoordinationSignal::NONE;

  record_cv_.wait(lock, [this, &goal_handle]() {
    return record_signal_ != RecordCoordinationSignal::NONE ||
           goal_handle->is_canceling() ||
           is_shutting_down_.load(std::memory_order_acquire);
  });

  is_waiting_record_.store(false, std::memory_order_release);
  const auto sig = record_signal_;
  record_signal_ = RecordCoordinationSignal::NONE;
  return sig;
}

void ManipulationActionServer::execute_record_mode(
  const MoveExecutionContext & ctx,
  bool is_pure_place)
{
  const std::string approach_phase = is_pure_place ? "APPROACH_PLACE" : "APPROACH_PICK";
  const auto & target_pt = is_pure_place ? ctx.goal->place_point : ctx.goal->pick_point;
  const double gripper_rad = is_pure_place ?
    arm_motion_planner_.gripper_closed_rad() : arm_motion_planner_.gripper_open_rad();

  geometry_msgs::msg::Point approach_pt = target_pt;
  approach_pt.z += approach_z_offset_;

  const auto approach_joints = resolve_point_ik(approach_pt, ctx.goal->target_frame, gripper_rad);
  if (!approach_joints.has_value()) {
    abort_move(ctx, "IK failed for record " + approach_phase, 0.0);
    return;
  }

  set_runtime_approach_pose(*approach_joints);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  publish_phase_feedback(ctx, approach_phase, 50.0);
  set_snapshot("RECORDING_WAIT", 50.0, ctx.goal->instruction);
  diagnostic_updater_.force_update();

  const auto received_sig = wait_for_record_signal(ctx.goal_handle);
  set_runtime_approach_pose(default_approach_pose_);

  if (is_shutting_down_.load(std::memory_order_acquire)) {
    abort_move(ctx, "Node shutting down", 50.0);
    return;
  }
  if (received_sig == RecordCoordinationSignal::CANCEL || ctx.goal_handle->is_canceling()) {
    for (int i = 0; i < 100 && !ctx.goal_handle->is_canceling(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (ctx.goal_handle->is_canceling()) {
      cancel_move(ctx, "Execution canceled by user during record wait", 0.0);
    } else {
      abort_move(ctx, "Cancel signal received but goal handle failed to transition to CANCELING", 50.0);
    }
    return;
  }
  if (received_sig == RecordCoordinationSignal::SAVE) {
    succeed_move(ctx, "Episode saved: record move completed successfully.");
    return;
  }
  abort_move(ctx, "Episode aborted by user.", 50.0);
}

std::optional<ManipulationActionServer::MoveWaypoints>
ManipulationActionServer::resolve_move_waypoints(
  const MoveExecutionContext & ctx,
  bool is_pure_pick,
  bool is_pure_place)
{
  MoveWaypoints waypoints;
  if (!is_pure_place) {
    waypoints.pick = resolve_approach_and_descend(
      ctx, ctx.goal->pick_point, arm_motion_planner_.gripper_open_rad(), "PICK");
    if (!waypoints.pick.has_value()) {
      return std::nullopt;
    }
  }

  if (!is_pure_pick && !ctx.goal->is_capture) {
    waypoints.place = resolve_approach_and_descend(
      ctx, ctx.goal->place_point, arm_motion_planner_.gripper_closed_rad(), "PLACE");
    if (!waypoints.place.has_value()) {
      return std::nullopt;
    }
  }
  return waypoints;
}

std::optional<ManipulationActionServer::ApproachDescendPair>
ManipulationActionServer::resolve_approach_and_descend(
  const MoveExecutionContext & ctx,
  const geometry_msgs::msg::Point & target_pt,
  double gripper_rad,
  const std::string & stage_name)
{
  geometry_msgs::msg::Point approach_pt = target_pt;
  approach_pt.z += approach_z_offset_;

  const auto approach_joints = resolve_point_ik(approach_pt, ctx.goal->target_frame, gripper_rad);
  if (!approach_joints.has_value()) {
    abort_move(ctx, "IK failed for " + stage_name + " approach", 0.0);
    return std::nullopt;
  }

  const auto descend_joints = resolve_point_ik(target_pt, ctx.goal->target_frame, gripper_rad);
  if (!descend_joints.has_value()) {
    abort_move(ctx, "IK failed for " + stage_name + " descend", 0.0);
    return std::nullopt;
  }

  return ApproachDescendPair{*approach_joints, *descend_joints};
}

std::pair<std::vector<std::string>, std::unordered_map<std::string, std::vector<double>>>
ManipulationActionServer::build_active_phase_plan(
  const std::shared_ptr<const ExecuteChessMove::Goal> & goal,
  bool is_pure_pick,
  bool is_pure_place,
  const std::optional<ApproachDescendPair> & pick_pair,
  const std::optional<ApproachDescendPair> & place_pair)
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (goal->is_capture) {
    const arm::ClearBinSide bin_side = (goal->pick_point.y >= 0.0) ?
      arm::ClearBinSide::LEFT : arm::ClearBinSide::RIGHT;
    return {
      arm::ArmMotionPlanner::kClearPhases,
      arm_motion_planner_.build_clear_targets(pick_pair->approach, pick_pair->descend, bin_side)
    };
  }
  if (is_pure_pick) {
    return {
      arm::ArmMotionPlanner::kPickPhases,
      arm_motion_planner_.build_pick_targets(pick_pair->approach, pick_pair->descend)
    };
  }
  if (is_pure_place) {
    return {
      arm::ArmMotionPlanner::kPlacePhases,
      arm_motion_planner_.build_place_targets(place_pair->approach, place_pair->descend)
    };
  }
  return {
    arm::ArmMotionPlanner::kPickAndPlacePhases,
    arm_motion_planner_.build_phase_targets(
      pick_pair->approach, pick_pair->descend, place_pair->approach, place_pair->descend)
  };
}

bool ManipulationActionServer::check_and_handle_preemption(
  const MoveExecutionContext & ctx,
  const std::string & phase_name,
  double progress)
{
  if (ctx.goal_handle->is_canceling()) {
    cancel_move(ctx, "Canceled during phase " + phase_name, progress);
    return true;
  }
  if (is_shutting_down_.load(std::memory_order_acquire)) {
    abort_move(ctx, "Aborted (server shutting down) during phase " + phase_name, progress);
    return true;
  }
  return false;
}

void ManipulationActionServer::publish_phase_feedback(
  const MoveExecutionContext & ctx,
  const std::string & phase_name,
  double progress)
{
  set_snapshot(phase_name, progress);
  ctx.feedback->current_phase = phase_name;
  ctx.feedback->progress_percent = static_cast<float>(progress);
  ctx.feedback->instruction = ctx.goal->instruction;
  ctx.goal_handle->publish_feedback(ctx.feedback);
  diagnostic_updater_.force_update();
}

bool ManipulationActionServer::execute_phase_step(
  size_t phase_idx,
  const std::string & phase_name,
  const std::vector<double> & target_pos,
  double total_phases,
  const MoveExecutionContext & ctx)
{
  const double progress = (static_cast<double>(phase_idx) / total_phases) * 100.0;
  if (check_and_handle_preemption(ctx, phase_name, progress)) {
    return false;
  }

  publish_phase_feedback(ctx, phase_name, progress);

  trajectory_msgs::msg::JointTrajectory traj;
  try {
    const auto current_pos = executor_->get_current_positions_or_default();
    std::lock_guard<std::mutex> lock(planner_mutex_);
    traj = arm_motion_planner_.plan_trajectory_at_velocity(
      current_pos, target_pos, default_velocity_, sampling_rate_hz_);
  } catch (const std::exception & exc) {
    abort_move(ctx, "Planning failed in " + phase_name + ": " + exc.what(), progress);
    return false;
  }

  auto traj_guard = acquire_trajectory_guard();
  if (!traj_guard.acquired()) {
    abort_move(ctx, "Physical trajectory lock already held in phase " + phase_name, progress);
    return false;
  }

  const auto [ok, err_msg] = executor_->execute_trajectory(
    traj,
    [handle = ctx.goal_handle]() { return handle && handle->is_canceling(); });

  if (!ok) {
    if (ctx.goal_handle && ctx.goal_handle->is_canceling()) {
      cancel_move(ctx, "Canceled during phase " + phase_name, progress);
      return false;
    }
    abort_move(ctx, "Execution failed in " + phase_name + ": " + err_msg, progress);
    return false;
  }

  return true;
}

void ManipulationActionServer::succeed_move(
  const MoveExecutionContext & ctx,
  const std::string & message)
{
  set_snapshot("IDLE", 100.0);
  ctx.result->success = true;
  ctx.result->message = message;
  ctx.result->execution_time_sec = compute_elapsed_sec(ctx.start_time);
  ctx.goal_handle->succeed(ctx.result);
  RCLCPP_INFO(
    get_logger(), "%s (duration: %.2fs)", message.c_str(), ctx.result->execution_time_sec);
  diagnostic_updater_.force_update();
}

void ManipulationActionServer::cancel_move(
  const MoveExecutionContext & ctx,
  const std::string & message,
  double progress)
{
  set_snapshot("CANCELED", progress);
  ctx.result->success = false;
  ctx.result->message = message;
  ctx.result->execution_time_sec = compute_elapsed_sec(ctx.start_time);
  ctx.goal_handle->canceled(ctx.result);
  RCLCPP_INFO(get_logger(), "Move canceled: %s", message.c_str());
  diagnostic_updater_.force_update();
}

void ManipulationActionServer::abort_move(
  const MoveExecutionContext & ctx,
  const std::string & message,
  double progress)
{
  set_snapshot("ABORTED", progress);
  ctx.result->success = false;
  ctx.result->message = message;
  ctx.result->execution_time_sec = compute_elapsed_sec(ctx.start_time);
  ctx.goal_handle->abort(ctx.result);
  RCLCPP_WARN(get_logger(), "Move aborted: %s", message.c_str());
  diagnostic_updater_.force_update();
}

void ManipulationActionServer::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ExecutionSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snap = snapshot_;
  }
  const bool active = is_goal_active_.load(std::memory_order_relaxed) ||
                      executor_->is_trajectory_busy();
  stat.summary(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    active ? "Executing manipulation" : "Idle");
  stat.add("active", active);
  stat.add("current_phase", snap.phase);
  stat.add("progress_percent", snap.progress_percent);
  stat.add("instruction", snap.instruction);
  stat.add("controller_action", controller_action_name_);
  stat.add("mock_execution", mock_execution_ ? "true" : "false");
  stat.add("record_mode", record_ ? "true" : "false");
  stat.add("arm_busy", executor_->is_trajectory_busy());
}

}  // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::ManipulationActionServer)

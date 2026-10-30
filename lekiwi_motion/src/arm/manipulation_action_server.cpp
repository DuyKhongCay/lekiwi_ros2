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
 * @file manipulation_action_server.cpp
 * @brief Implementation of autonomous manipulation action server and recorder coordination.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/arm/manipulation_action_server.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <utility>

namespace lekiwi_motion
{

ManipulationActionServer::ManipulationActionServer(const rclcpp::NodeOptions & options)
: Node("manipulation_action_server", options),
  diagnostic_updater_(this)
{
  declare_and_init_parameters();

  cb_group_action_server_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_recorder_coordination_ = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  action_server_ = rclcpp_action::create_server<ExecuteChessMove>(
    this,
    action_name_,
    std::bind(&ManipulationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ManipulationActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ManipulationActionServer::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    cb_group_action_server_);

  save_episode_srv_ = create_service<Trigger>(
    "/arm/save_episode",
    std::bind(&ManipulationActionServer::handle_save_episode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_recorder_coordination_);

  skip_episode_srv_ = create_service<Trigger>(
    "/arm/skip_episode",
    std::bind(&ManipulationActionServer::handle_skip_episode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_recorder_coordination_);

  named_pose_srv_ = create_service<GoToNamedPose>(
    "~/go_to_named_pose",
    std::bind(&ManipulationActionServer::handle_named_pose_request, this, std::placeholders::_1, std::placeholders::_2),
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

  diagnostic_updater_.setHardwareID("lekiwi_arm_manipulator");
  diagnostic_updater_.add("Manipulation Status", this, &ManipulationActionServer::produce_diagnostics);

  RCLCPP_INFO(
    get_logger(),
    "ManipulationActionServer initialized. Action: '%s'. Record mode: %s",
    action_name_.c_str(), record_ ? "ENABLED" : "DISABLED");
}

ManipulationActionServer::~ManipulationActionServer()
{
  is_shutting_down_.store(true, std::memory_order_release);
  if (executor_) {
    executor_->stop();
  }
  {
    std::lock_guard<std::mutex> lock(record_mutex_);
    record_signal_ = RecordCoordinationSignal::CANCEL;
  }
  record_cv_.notify_all();
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

namespace
{
template <typename T>
T get_or_declare_server_param(
  rclcpp::Node * node,
  const std::string & name,
  const T & default_val,
  const std::string & description = "")
{
  if (!node->has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    desc.read_only = true;
    return node->declare_parameter<T>(name, default_val, desc);
  }
  return node->get_parameter(name).get_value<T>();
}
}  // namespace

void ManipulationActionServer::declare_and_init_parameters()
{
  action_name_ = get_or_declare_server_param<std::string>(
    this, "action_name", "/manipulation/execute_chess_move", "Manipulation action server endpoint name");
  approach_z_offset_ = get_or_declare_server_param<double>(
    this, "approach_z_offset", 0.05, "Vertical Z offset for pre-grasp approach phase (meters)");
  record_ = get_or_declare_server_param<bool>(
    this, "record", false, "Record mode flag: stops at approach pick/place without descending");
  if (record_) {
    RCLCPP_INFO(
      get_logger(),
      "ManipulationActionServer operating in RECORD mode (stopping at pre-grasp approach)");
  }

  arm::ArmTrajectoryExecutorConfig exec_cfg;
  executor_ = std::make_unique<arm::ArmTrajectoryExecutor>(this, std::move(exec_cfg));
}

void ManipulationActionServer::notify_record_signal(RecordCoordinationSignal signal) noexcept
{
  {
    std::lock_guard<std::mutex> lock(record_mutex_);
    record_signal_ = signal;
  }
  record_cv_.notify_all();
}

bool ManipulationActionServer::forward_trigger_request(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const std::string & service_label,
  const std::shared_ptr<Trigger::Response> & response)
{
  if (!client) {
    response->success = false;
    response->message = service_label + " client is uninitialized";
    return false;
  }
  if (!client->wait_for_service(std::chrono::milliseconds(500))) {
    response->success = false;
    response->message = service_label + " service unavailable";
    return false;
  }

  auto req = std::make_shared<Trigger::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    response->success = false;
    response->message = service_label + " call timed out";
    return false;
  }

  auto res = future.get();
  response->success = res->success;
  response->message = res->message;
  return res->success;
}

void ManipulationActionServer::discard_active_recording_async() noexcept
{
  if (discard_client_ && discard_client_->service_is_ready()) {
    auto req = std::make_shared<Trigger::Request>();
    discard_client_->async_send_request(req);
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
  RCLCPP_INFO(get_logger(), "Received service request: /arm/save_episode");
  if (forward_trigger_request(save_client_, "/episode_recorder_node/save_episode", response)) {
    notify_record_signal(RecordCoordinationSignal::SAVE);
  }
}

void ManipulationActionServer::handle_skip_episode(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (!is_waiting_record_.load(std::memory_order_acquire)) {
    response->success = false;
    response->message = "No active manipulation goal in RECORDING_WAIT state";
    RCLCPP_WARN(get_logger(), "Rejecting /arm/skip_episode: %s", response->message.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Received service request: /arm/skip_episode");
  if (forward_trigger_request(discard_client_, "/episode_recorder_node/discard_episode", response)) {
    notify_record_signal(RecordCoordinationSignal::SKIP);
  }
}

void ManipulationActionServer::handle_named_pose_request(
  const std::shared_ptr<GoToNamedPose::Request> request,
  std::shared_ptr<GoToNamedPose::Response> response)
{
  if (!request || request->pose_name.empty()) {
    response->success = false;
    response->message = "Invalid empty pose name";
    return;
  }

  const std::string key = arm::to_lower_trimmed(request->pose_name);
  if (key != "approach") {
    response->success = false;
    response->message =
      "ManipulationActionServer only manages dynamic 'approach'. For other poses (stow, clear_left, clear_right), use /cartesian_jog_server_node/go_to_named_pose";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!executor_) {
    response->success = false;
    response->message = "ArmTrajectoryExecutor not available";
    return;
  }

  if (!executor_->has_named_pose("approach")) {
    response->success = false;
    response->message = "No dynamic approach pose has been calculated yet in record mode.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Executing dynamic approach pose transition...");
  auto [ok, msg] = executor_->execute_named_pose("approach");
  response->success = ok;
  response->message = msg;
  if (!ok) {
    RCLCPP_WARN(get_logger(), "Failed moving arm to approach pose: %s", msg.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Successfully moved arm to dynamic approach pose.");
  }
}

rclcpp_action::GoalResponse ManipulationActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExecuteChessMove::Goal> goal)
{
  if (goal->operation > ExecuteChessMove::Goal::OP_PLACE) {
    RCLCPP_WARN(
      get_logger(), "Rejecting goal: Unknown operation enum %d", goal->operation);
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool expected = false;
  if (!is_goal_active_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    RCLCPP_WARN(
      get_logger(), "Rejecting goal: Arm is currently busy executing a trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (executor_ && executor_->is_trajectory_busy()) {
    is_goal_active_.store(false, std::memory_order_release);
    RCLCPP_WARN(
      get_logger(), "Rejecting goal: Arm is currently busy executing a trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleChessMove> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancel requested for active manipulation goal");
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

std::string ManipulationActionServer::resolve_instruction_label(
  const ExecuteChessMove::Goal & goal) const
{
  if (!goal.instruction.empty()) {
    return goal.instruction;
  }
  switch (goal.operation) {
    case ExecuteChessMove::Goal::OP_MOVE:
      return "OP_MOVE";
    case ExecuteChessMove::Goal::OP_CLEAR_LEFT:
      return "OP_CLEAR_LEFT";
    case ExecuteChessMove::Goal::OP_CLEAR_RIGHT:
      return "OP_CLEAR_RIGHT";
    case ExecuteChessMove::Goal::OP_PICK:
      return "OP_PICK";
    case ExecuteChessMove::Goal::OP_PLACE:
      return "OP_PLACE";
    default:
      return "OP_UNKNOWN";
  }
}

void ManipulationActionServer::execute_move(
  const std::shared_ptr<GoalHandleChessMove> goal_handle)
{
  try {
    struct GoalActiveScopeGuard
    {
      std::atomic<bool> & flag;
      ~GoalActiveScopeGuard() { flag.store(false, std::memory_order_release); }
    } scope_guard{is_goal_active_};

    MoveExecutionContext ctx{
      goal_handle,
      goal_handle->get_goal(),
      std::make_shared<ExecuteChessMove::Feedback>(),
      std::make_shared<ExecuteChessMove::Result>(),
      std::chrono::steady_clock::now()
    };

    const std::string instruction_label = resolve_instruction_label(*ctx.goal);

    if (record_) {
      execute_record_mode(ctx, instruction_label);
      return;
    }

    const auto waypoints_opt = resolve_move_waypoints(ctx);
    if (!waypoints_opt.has_value()) {
      return;
    }

    std::pair<std::vector<std::string>, std::unordered_map<std::string, std::vector<double>>> plan;
    try {
      plan = build_active_phase_plan(ctx.goal, waypoints_opt->pick, waypoints_opt->place);
    } catch (const std::exception & exc) {
      abort_move(ctx, std::string("Phase plan generation failed: ") + exc.what(), 0.0);
      return;
    }

    if (execute_phase_pipeline(ctx, plan.first, plan.second, instruction_label)) {
      succeed_move(ctx, "Chess move completed successfully: " + instruction_label);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Unhandled exception in execute_move: %s", e.what());
    if (goal_handle && goal_handle->is_active()) {
      auto res = std::make_shared<ExecuteChessMove::Result>();
      res->success = false;
      res->message = std::string("Internal error: ") + e.what();
      goal_handle->abort(res);
    }
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Unknown unhandled exception in execute_move");
    if (goal_handle && goal_handle->is_active()) {
      auto res = std::make_shared<ExecuteChessMove::Result>();
      res->success = false;
      res->message = "Unknown internal error";
      goal_handle->abort(res);
    }
  }
}

bool ManipulationActionServer::execute_phase_pipeline(
  const MoveExecutionContext & ctx,
  const std::vector<std::string> & phase_names,
  const std::unordered_map<std::string, std::vector<double>> & phase_targets,
  const std::string & instruction_label)
{
  const double total_phases = static_cast<double>(phase_names.size());
  for (size_t i = 0; i < phase_names.size(); ++i) {
    const auto & phase_name = phase_names[i];
    auto it = phase_targets.find(phase_name);
    if (it == phase_targets.end()) {
      abort_move(ctx, "Missing target for phase: " + phase_name, 0.0);
      return false;
    }
    if (!execute_phase_step(i, phase_name, it->second, total_phases, ctx, instruction_label)) {
      return false;
    }
  }
  return true;
}

std::optional<std::vector<double>> ManipulationActionServer::compute_approach_joints(
  const MoveExecutionContext & ctx,
  bool is_pure_place)
{
  if (!executor_) {
    return std::nullopt;
  }
  const auto & target_pt = is_pure_place ? ctx.goal->place_point : ctx.goal->pick_point;
  const double gripper_rad = is_pure_place ?
    executor_->gripper_closed_rad() : executor_->gripper_open_rad();

  geometry_msgs::msg::Point approach_pt = target_pt;
  approach_pt.z += approach_z_offset_;

  return executor_->resolve_point_ik(approach_pt, ctx.goal->target_frame, gripper_rad);
}

void ManipulationActionServer::execute_record_mode(
  const MoveExecutionContext & ctx,
  const std::string & instruction_label)
{
  const bool is_pure_place = (ctx.goal->operation == ExecuteChessMove::Goal::OP_PLACE);
  const std::string approach_phase = is_pure_place ? "APPROACH_PLACE" : "APPROACH_PICK";

  const auto approach_joints = compute_approach_joints(ctx, is_pure_place);
  if (!approach_joints.has_value()) {
    abort_move(ctx, "IK resolution failed for " + approach_phase + " in RECORD mode", 0.0);
    return;
  }

  executor_->set_named_pose("approach", *approach_joints);

  publish_phase_feedback(ctx, "WAITING_RECORD_TRIGGER", 25.0, instruction_label);
  RCLCPP_INFO(
    get_logger(),
    "RECORD mode: Dynamic approach pose calculated and registered to ~/go_to_named_pose ('approach'). "
    "Arm stationary. Waiting for operator trigger (/arm/save_episode or /arm/skip_episode)...");

  is_waiting_record_.store(true, std::memory_order_release);
  const auto signal = wait_for_record_signal(ctx);
  is_waiting_record_.store(false, std::memory_order_release);

  process_record_signal(ctx, signal);
}

void ManipulationActionServer::process_record_signal(
  const MoveExecutionContext & ctx,
  RecordCoordinationSignal sig)
{
  switch (sig) {
    case RecordCoordinationSignal::SAVE:
      succeed_move(ctx, "Episode saved: " + resolve_instruction_label(*ctx.goal));
      break;
    case RecordCoordinationSignal::SKIP:
      succeed_move(ctx, "Episode skipped: " + resolve_instruction_label(*ctx.goal));
      break;
    case RecordCoordinationSignal::CANCEL:
      cancel_move(ctx, "Record move canceled by client request", 25.0);
      break;
    case RecordCoordinationSignal::NONE:
    default:
      abort_move(ctx, "Record mode timed out waiting for signal", 25.0);
      break;
  }
}

ManipulationActionServer::RecordCoordinationSignal ManipulationActionServer::wait_for_record_signal(
  const MoveExecutionContext & ctx)
{
  std::unique_lock<std::mutex> lock(record_mutex_);
  record_signal_ = RecordCoordinationSignal::NONE;

  while (!is_shutting_down_.load(std::memory_order_acquire)) {
    if (ctx.goal_handle->is_canceling()) {
      discard_active_recording_async();
      return RecordCoordinationSignal::CANCEL;
    }
    if (record_signal_ != RecordCoordinationSignal::NONE) {
      return record_signal_;
    }
    publish_phase_feedback(ctx, "WAITING_RECORD_TRIGGER", 25.0, resolve_instruction_label(*ctx.goal));
    record_cv_.wait_for(lock, std::chrono::milliseconds(50));
  }
  return RecordCoordinationSignal::CANCEL;
}

std::optional<ManipulationActionServer::MoveWaypoints>
ManipulationActionServer::resolve_move_waypoints(const MoveExecutionContext & ctx)
{
  MoveWaypoints waypoints;
  const uint8_t op = ctx.goal->operation;

  if (op == ExecuteChessMove::Goal::OP_MOVE ||
      op == ExecuteChessMove::Goal::OP_CLEAR_LEFT ||
      op == ExecuteChessMove::Goal::OP_CLEAR_RIGHT ||
      op == ExecuteChessMove::Goal::OP_PICK)
  {
    waypoints.pick = resolve_approach_and_descend(
      ctx, ctx.goal->pick_point, executor_->gripper_open_rad(), "PICK");
    if (!waypoints.pick.has_value()) {
      return std::nullopt;
    }
  }

  if (op == ExecuteChessMove::Goal::OP_MOVE ||
      op == ExecuteChessMove::Goal::OP_PLACE)
  {
    waypoints.place = resolve_approach_and_descend(
      ctx, ctx.goal->place_point, executor_->gripper_closed_rad(), "PLACE");
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
  if (!executor_) {
    return std::nullopt;
  }
  geometry_msgs::msg::Point approach_pt = target_pt;
  approach_pt.z += approach_z_offset_;

  const auto approach_opt = executor_->resolve_point_ik(approach_pt, ctx.goal->target_frame, gripper_rad);
  if (!approach_opt.has_value()) {
    abort_move(ctx, "IK resolution failed for APPROACH_" + stage_name, 0.0);
    return std::nullopt;
  }

  const auto descend_opt = executor_->resolve_point_ik(target_pt, ctx.goal->target_frame, gripper_rad);
  if (!descend_opt.has_value()) {
    abort_move(ctx, "IK resolution failed for DESCEND_" + stage_name, 0.0);
    return std::nullopt;
  }

  return ApproachDescendPair{*approach_opt, *descend_opt};
}

std::pair<std::vector<std::string>, std::unordered_map<std::string, std::vector<double>>>
ManipulationActionServer::build_active_phase_plan(
  const std::shared_ptr<const ExecuteChessMove::Goal> & goal,
  const std::optional<ApproachDescendPair> & pick_pair,
  const std::optional<ApproachDescendPair> & place_pair)
{
  switch (goal->operation) {
    case ExecuteChessMove::Goal::OP_MOVE:
      if (!pick_pair.has_value() || !place_pair.has_value()) {
        throw std::invalid_argument("OP_MOVE requires both pick and place waypoints.");
      }
      return {
        arm::ArmMotionPlanner::kPickAndPlacePhases,
        executor_->build_phase_targets(
          pick_pair->approach, pick_pair->descend, place_pair->approach, place_pair->descend)
      };

    case ExecuteChessMove::Goal::OP_CLEAR_LEFT:
      if (!pick_pair.has_value()) {
        throw std::invalid_argument("OP_CLEAR_LEFT requires pick waypoint.");
      }
      return {
        arm::ArmMotionPlanner::kClearPhases,
        executor_->build_clear_targets(
          pick_pair->approach, pick_pair->descend, arm::ClearBinSide::LEFT)
      };

    case ExecuteChessMove::Goal::OP_CLEAR_RIGHT:
      if (!pick_pair.has_value()) {
        throw std::invalid_argument("OP_CLEAR_RIGHT requires pick waypoint.");
      }
      return {
        arm::ArmMotionPlanner::kClearPhases,
        executor_->build_clear_targets(
          pick_pair->approach, pick_pair->descend, arm::ClearBinSide::RIGHT)
      };

    case ExecuteChessMove::Goal::OP_PICK:
      if (!pick_pair.has_value()) {
        throw std::invalid_argument("OP_PICK requires pick waypoint.");
      }
      return {
        arm::ArmMotionPlanner::kPickPhases,
        executor_->build_pick_targets(pick_pair->approach, pick_pair->descend)
      };

    case ExecuteChessMove::Goal::OP_PLACE:
      if (!place_pair.has_value()) {
        throw std::invalid_argument("OP_PLACE requires place waypoint.");
      }
      return {
        arm::ArmMotionPlanner::kPlacePhases,
        executor_->build_place_targets(place_pair->approach, place_pair->descend)
      };

    default:
      throw std::invalid_argument(
        "Unknown manipulation operation enum: " + std::to_string(goal->operation));
  }
}

bool ManipulationActionServer::execute_phase_step(
  size_t phase_idx,
  const std::string & phase_name,
  const std::vector<double> & target_pos,
  double total_phases,
  const MoveExecutionContext & ctx,
  const std::string & instruction_label)
{
  const double base_progress = (static_cast<double>(phase_idx) / total_phases) * 100.0;
  publish_phase_feedback(ctx, phase_name, base_progress, instruction_label);

  if (check_and_handle_preemption(ctx, phase_name, base_progress)) {
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Executing phase [%zu/%.0f]: '%s'",
    phase_idx + 1, total_phases, phase_name.c_str());

  auto [ok, msg] = executor_->execute_joint_target(
    target_pos, phase_name, -1.0,
    [&ctx]() { return ctx.goal_handle->is_canceling(); });

  if (!ok) {
    if (ctx.goal_handle->is_canceling()) {
      cancel_move(ctx, "Preempted and canceled at phase: " + phase_name, base_progress);
    } else {
      abort_move(ctx, "Execution failed for phase " + phase_name + ": " + msg, base_progress);
    }
    return false;
  }
  return true;
}

bool ManipulationActionServer::check_and_handle_preemption(
  const MoveExecutionContext & ctx,
  const std::string & phase_name,
  double progress)
{
  if (ctx.goal_handle->is_canceling()) {
    cancel_move(ctx, "Goal canceled before executing phase: " + phase_name, progress);
    return true;
  }
  if (is_shutting_down_.load(std::memory_order_acquire)) {
    abort_move(ctx, "Server shutting down", progress);
    return true;
  }
  return false;
}

void ManipulationActionServer::publish_phase_feedback(
  const MoveExecutionContext & ctx,
  const std::string & phase_name,
  double progress,
  const std::string & instruction_label)
{
  set_snapshot(phase_name, progress, instruction_label);
  ctx.feedback->current_phase = phase_name;
  ctx.feedback->progress_percent = static_cast<float>(progress);
  ctx.feedback->instruction = instruction_label;
  ctx.goal_handle->publish_feedback(ctx.feedback);
}

void ManipulationActionServer::finalize_move(
  const MoveExecutionContext & ctx,
  ActionTerminalState term_state,
  const std::string & message,
  double progress)
{
  const double duration = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - ctx.start_time).count();
  const std::string instruction = resolve_instruction_label(*ctx.goal);
  ctx.result->message = message;
  ctx.result->execution_time_sec = static_cast<float>(duration);

  switch (term_state) {
    case ActionTerminalState::SUCCEEDED:
      set_snapshot("COMPLETED", 100.0, instruction);
      ctx.result->success = true;
      ctx.goal_handle->succeed(ctx.result);
      RCLCPP_INFO(get_logger(), "%s (Duration: %.2fs)", message.c_str(), duration);
      break;
    case ActionTerminalState::CANCELED:
      set_snapshot("CANCELED", progress, instruction);
      ctx.result->success = false;
      if (ctx.goal_handle->is_canceling()) {
        ctx.goal_handle->canceled(ctx.result);
      } else {
        ctx.goal_handle->abort(ctx.result);
      }
      RCLCPP_WARN(get_logger(), "%s (Duration: %.2fs)", message.c_str(), duration);
      break;
    case ActionTerminalState::ABORTED:
      set_snapshot("ABORTED", progress, instruction);
      ctx.result->success = false;
      ctx.goal_handle->abort(ctx.result);
      RCLCPP_ERROR(get_logger(), "%s (Duration: %.2fs)", message.c_str(), duration);
      break;
  }
}

void ManipulationActionServer::succeed_move(
  const MoveExecutionContext & ctx,
  const std::string & message)
{
  finalize_move(ctx, ActionTerminalState::SUCCEEDED, message, 100.0);
}

void ManipulationActionServer::cancel_move(
  const MoveExecutionContext & ctx,
  const std::string & message,
  double progress)
{
  finalize_move(ctx, ActionTerminalState::CANCELED, message, progress);
}

void ManipulationActionServer::abort_move(
  const MoveExecutionContext & ctx,
  const std::string & message,
  double progress)
{
  finalize_move(ctx, ActionTerminalState::ABORTED, message, progress);
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

void ManipulationActionServer::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Arm manipulation nominal");
  stat.add("Active Phase", snapshot_.phase);
  stat.add("Progress (%)", snapshot_.progress_percent);
  stat.add("Instruction", snapshot_.instruction);
  stat.add("Goal Active", is_goal_active_.load(std::memory_order_relaxed));
  stat.add("Record Mode Waiting", is_waiting_record_.load(std::memory_order_relaxed));
  stat.add(
    "Trajectory Busy",
    executor_ ? executor_->is_trajectory_busy() : false);
  stat.add(
    "Kinematics Ready",
    executor_ ? executor_->is_kinematics_initialized() : false);
}

}  // namespace lekiwi_motion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::ManipulationActionServer)

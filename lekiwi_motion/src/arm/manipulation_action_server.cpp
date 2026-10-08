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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

namespace lekiwi_motion
{

  ManipulationActionServer::ManipulationActionServer(const rclcpp::NodeOptions &options)
      : Node("manipulation_action_server", options),
        diagnostic_updater_(this)
  {
    declare_and_load_parameters();
    init_kinematics_and_tf();

    cb_group_action_server_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cb_group_controller_client_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    diagnostic_updater_.setHardwareID("lekiwi_motion_manipulation");
    diagnostic_updater_.add("manipulation_status", this, &ManipulationActionServer::produce_diagnostics);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_action_server_;
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ManipulationActionServer::on_joint_state, this, std::placeholders::_1),
        sub_options);

    named_pose_sub_ = create_subscription<std_msgs::msg::String>(
        "~/command_named_pose",
        10,
        std::bind(&ManipulationActionServer::on_command_named_pose, this, std::placeholders::_1),
        sub_options);

    if (mock_manipulation_)
    {
      mock_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
          joint_states_topic_, 10);
      RCLCPP_INFO(
          get_logger(),
          "ManipulationActionServer online in MOCK mode at '%s' (simulated trajectory & /joint_states enabled)",
          action_name_.c_str());
    }
    else
    {
      controller_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
          this,
          controller_action_name_,
          cb_group_controller_client_);
      RCLCPP_INFO(
          get_logger(),
          "ManipulationActionServer online at '%s' -> controller '%s'",
          action_name_.c_str(),
          controller_action_name_.c_str());
    }

    action_server_ = rclcpp_action::create_server<ExecuteChessMove>(
        this,
        action_name_,
        std::bind(&ManipulationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ManipulationActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ManipulationActionServer::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        cb_group_action_server_);
  }

  ManipulationActionServer::~ManipulationActionServer()
  {
    is_shutting_down_ = true;
    if (execution_thread_.joinable())
    {
      execution_thread_.join();
    }
  }

  void ManipulationActionServer::declare_and_load_parameters()
  {
    action_name_ = declare_parameter<std::string>(
        "action_name",
        "/manipulation/execute_chess_move");
    controller_action_name_ = declare_parameter<std::string>(
        "trajectory_controller_action",
        "/arm_trajectory_controller/follow_joint_trajectory");
    joint_states_topic_ = declare_parameter<std::string>(
        "joint_states_topic",
        "/joint_states");
    max_velocity_ = declare_parameter<double>(
        "max_velocity",
        1.5);
    if (max_velocity_ <= 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "Parameter 'max_velocity' must be positive, got %.2f. Defaulting to 1.5 rad/s",
          max_velocity_);
      max_velocity_ = 1.5;
    }
    approach_z_offset_ = declare_parameter<double>(
        "approach_z_offset",
        0.05);
    sampling_rate_hz_ = declare_parameter<double>(
        "sampling_rate_hz",
        50.0);
    mock_manipulation_ = declare_parameter<bool>(
        "mock_manipulation",
        false);

    arm_joints_ = declare_parameter<std::vector<std::string>>(
        "arm_joints",
        std::vector<std::string>{
            "arm_shoulder_pan",
            "arm_shoulder_lift",
            "arm_elbow_flex",
            "arm_wrist_flex",
            "arm_wrist_roll",
            "arm_gripper"});

    const double gripper_open = declare_parameter<double>("gripper.open", 1.50);
    const double gripper_closed = declare_parameter<double>("gripper.closed", 0.00);

    NamedPosesConfig named_poses;
    named_poses.home = declare_parameter<std::vector<double>>(
        "named_poses.home",
        {0.0, 0.0, 0.0, 0.0, 0.0, gripper_closed});
    named_poses.stow = declare_parameter<std::vector<double>>(
        "named_poses.stow",
        {0.0, -1.57, 1.57, 0.75, 0.0, gripper_closed});
    named_poses.clear_left = declare_parameter<std::vector<double>>(
        "named_poses.clear_left",
        {1.20, -0.60, 0.80, 0.0, 0.0, gripper_closed});
    named_poses.clear_right = declare_parameter<std::vector<double>>(
        "named_poses.clear_right",
        {-1.20, -0.60, 0.80, 0.0, 0.0, gripper_closed});

    arm_motion_planner_.set_named_poses(std::move(named_poses));
    arm_motion_planner_.set_arm_joints(arm_joints_);
    arm_motion_planner_.set_default_rate_hz(sampling_rate_hz_);
    arm_motion_planner_.set_gripper_config({gripper_open, gripper_closed});

    board_frame_ = declare_parameter<std::string>("board_frame", "chessboard_frame");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    tip_frame_ = declare_parameter<std::string>("tip_frame", "gripperframe");
    grasp_z_ = declare_parameter<double>("planning.grasp_z", 0.035);
    default_pitch_ = declare_parameter<double>("planning.default_pitch", -M_PI_2);
    default_roll_ = declare_parameter<double>("planning.default_roll", 0.0);
  }

  void ManipulationActionServer::init_kinematics_and_tf()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string urdf_xml;
    if (has_parameter("robot_description"))
    {
      urdf_xml = get_parameter("robot_description").as_string();
    }
    if (!urdf_xml.empty())
    {
      accept_robot_description(urdf_xml);
    }
    else
    {
      rclcpp::QoS qos_profile(1);
      qos_profile.transient_local();
      qos_profile.reliable();
      robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
          "robot_description", qos_profile,
          std::bind(&ManipulationActionServer::on_robot_description, this, std::placeholders::_1));
    }
  }

  void ManipulationActionServer::on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (msg)
    {
      accept_robot_description(msg->data);
    }
  }

  void ManipulationActionServer::accept_robot_description(const std::string &xml)
  {
    try
    {
      auto urdf_model = urdf::parseURDF(xml);
      if (!urdf_model)
      {
        RCLCPP_WARN(get_logger(), "Failed to parse robot_description XML");
        return;
      }
      workspace::KinematicsModel new_model;
      std::string err;
      std::vector<std::string> arm_5_joints;
      if (arm_joints_.size() >= 5)
      {
        arm_5_joints.assign(arm_joints_.begin(), arm_joints_.begin() + 5);
      }
      else
      {
        arm_5_joints = {"arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll"};
      }

      if (!workspace::extract_kinematics_from_urdf(*urdf_model, base_frame_, tip_frame_,
                                                   arm_5_joints, new_model, err, 0.05))
      {
        RCLCPP_WARN(get_logger(), "Kinematics extraction failed: %s", err.c_str());
        return;
      }
      kinematics_model_ = new_model;
      analytical_solver_ = std::make_unique<workspace::SO101AnalyticalSolver>(new_model);
      RCLCPP_INFO(get_logger(), "ManipulationActionServer IK solver ready (reach: %.3f m)", analytical_solver_->reach_bound());
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Exception during kinematics initialization: %s", e.what());
    }
  }

  geometry_msgs::msg::Point ManipulationActionServer::sanitize_target_point(
      const geometry_msgs::msg::Point &point,
      const std::string &target_frame) const
  {
    geometry_msgs::msg::Point pt = point;
    const std::string actual_frame = target_frame.empty() ? board_frame_ : target_frame;
    if (pt.z == 0.0 && actual_frame == board_frame_ && grasp_z_ > 0.0)
    {
      pt.z = grasp_z_;
    }
    return pt;
  }

  std::optional<geometry_msgs::msg::PointStamped> ManipulationActionServer::transform_to_base(
      const geometry_msgs::msg::PointStamped &in_pt) const
  {
    if (in_pt.header.frame_id == base_frame_)
    {
      return in_pt;
    }
    if (!tf_buffer_)
    {
      RCLCPP_WARN(get_logger(), "TF buffer unavailable for JIT IK resolution");
      return std::nullopt;
    }
    try
    {
      return tf_buffer_->transform(in_pt, base_frame_, tf2::durationFromSec(0.2));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(
          get_logger(), "TF transform failed (%s -> %s): %s",
          in_pt.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  std::optional<std::vector<double>> ManipulationActionServer::resolve_point_ik(
      const geometry_msgs::msg::Point &point,
      const std::string &target_frame,
      double gripper_state)
  {
    if (!analytical_solver_)
    {
      RCLCPP_WARN(get_logger(), "SO101AnalyticalSolver not initialized");
      return std::nullopt;
    }

    const std::string actual_frame = target_frame.empty() ? board_frame_ : target_frame;
    geometry_msgs::msg::PointStamped in_pt;
    in_pt.header.frame_id = actual_frame;
    in_pt.point = sanitize_target_point(point, target_frame);

    const auto out_pt = transform_to_base(in_pt);
    if (!out_pt.has_value())
    {
      return std::nullopt;
    }

    auto ik_res = analytical_solver_->solve(
        out_pt->point.x, out_pt->point.y, out_pt->point.z, default_pitch_, default_roll_);
    if (!ik_res.success)
    {
      RCLCPP_WARN(
          get_logger(), "JIT IK unsolvable for target (%.3f, %.3f, %.3f) in frame '%s'",
          out_pt->point.x, out_pt->point.y, out_pt->point.z, base_frame_.c_str());
      return std::nullopt;
    }

    std::vector<double> joint_positions(arm_joints_.size(), 0.0);
    for (size_t i = 0; i < 5 && i < arm_joints_.size(); ++i)
    {
      joint_positions[i] = ik_res.joints[i];
    }
    joint_positions.back() = arm_motion_planner_.clamp_gripper(gripper_state);
    return joint_positions;
  }

  void ManipulationActionServer::on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (i < msg->position.size())
      {
        current_joints_[msg->name[i]] = msg->position[i];
      }
    }
  }

  std::vector<double> ManipulationActionServer::get_current_positions_or_default()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<double> positions;
    positions.reserve(arm_joints_.size());
    for (const auto &j : arm_joints_)
    {
      auto it = current_joints_.find(j);
      positions.push_back(it != current_joints_.end() ? it->second : 0.0);
    }
    return positions;
  }

  void ManipulationActionServer::set_snapshot(
      const std::string &phase,
      double progress_percent,
      const std::string &instruction)
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.phase = phase;
    snapshot_.progress_percent = progress_percent;
    if (!instruction.empty())
    {
      snapshot_.instruction = instruction;
    }
  }

  rclcpp_action::GoalResponse ManipulationActionServer::handle_goal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const ExecuteChessMove::Goal> goal)
  {
    bool expected = false;
    if (!is_executing_.compare_exchange_strong(expected, true))
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal: manipulation server is busy.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
        get_logger(),
        "Received manipulation goal: '%s' (%s -> %s)",
        goal->instruction.c_str(),
        goal->from_square.c_str(),
        goal->to_square.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ManipulationActionServer::handle_cancel(
      const std::shared_ptr<GoalHandleChessMove> /*goal_handle*/)
  {
    RCLCPP_WARN(get_logger(), "Manipulation cancel request received.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ManipulationActionServer::handle_accepted(const std::shared_ptr<GoalHandleChessMove> goal_handle)
  {
    if (execution_thread_.joinable())
    {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&ManipulationActionServer::execute_move, this, goal_handle);
  }

  std::pair<bool, std::string> ManipulationActionServer::wait_for_controller_result(
      rclcpp_action::Client<FollowJointTrajectory>::GoalHandle::SharedPtr handle,
      double expected_sec,
      const std::shared_ptr<GoalHandleChessMove> &goal_handle)
  {
    auto result_future = controller_client_->async_get_result(handle);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(expected_sec + 3.0);

    while (std::chrono::steady_clock::now() < deadline)
    {
      if ((goal_handle && goal_handle->is_canceling()) || is_shutting_down_)
      {
        controller_client_->async_cancel_goal(handle);
        return {false, "Preempted and canceled"};
      }
      if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready)
      {
        break;
      }
    }

    if (result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      controller_client_->async_cancel_goal(handle);
      return {false, "Controller execution timed out"};
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL)
    {
      return {true, "Success"};
    }

    return {false, "Controller failed with code " + std::to_string(wrapped_result.result->error_code)};
  }

  std::pair<bool, std::string> ManipulationActionServer::send_trajectory_and_wait(
      const trajectory_msgs::msg::JointTrajectory &traj,
      const std::shared_ptr<GoalHandleChessMove> &goal_handle)
  {
    if (mock_manipulation_)
    {
      return execute_mock_trajectory(traj, goal_handle);
    }

    if (!controller_client_ || !controller_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      return {false, "Controller action server unavailable"};
    }

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = traj;

    auto send_future = controller_client_->async_send_goal(goal_msg);
    if (send_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      return {false, "Send goal to controller timed out"};
    }

    auto handle = send_future.get();
    if (!handle)
    {
      return {false, "Trajectory rejected by controller"};
    }

    const double expected_sec = traj.points.empty() ? 1.0 : (traj.points.back().time_from_start.sec + traj.points.back().time_from_start.nanosec * 1e-9);
    return wait_for_controller_result(handle, expected_sec, goal_handle);
  }

  void ManipulationActionServer::publish_simulated_joint_state(const std::vector<double> & positions)
  {
    if (!mock_joint_state_pub_)
    {
      return;
    }
    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now();
    js_msg.name = arm_joints_;
    js_msg.position = positions;
    mock_joint_state_pub_->publish(js_msg);
  }

  std::pair<bool, std::string> ManipulationActionServer::execute_mock_trajectory(
      const trajectory_msgs::msg::JointTrajectory &traj,
      const std::shared_ptr<GoalHandleChessMove> &goal_handle)
  {
    if (traj.points.empty())
    {
      return {true, "Empty trajectory"};
    }

    const double expected_sec = traj.points.back().time_from_start.sec +
                                traj.points.back().time_from_start.nanosec * 1e-9;
    const double effective_duration = (expected_sec > 0.0) ? expected_sec : 1.0;
    const auto & target_positions = traj.points.back().positions;

    const double dt = 0.05;  // 50ms ticks
    const auto start_time = std::chrono::steady_clock::now();

    while (!is_shutting_down_)
    {
      if (goal_handle && goal_handle->is_canceling())
      {
        return {false, "Preempted and canceled"};
      }

      auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      if (elapsed >= effective_duration)
      {
        break;
      }

      const size_t pt_idx = std::min(
          traj.points.size() - 1,
          static_cast<size_t>((elapsed / effective_duration) * (traj.points.size() - 1)));
      const auto & step_pos = traj.points[pt_idx].positions;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < arm_joints_.size() && i < step_pos.size(); ++i)
        {
          current_joints_[arm_joints_[i]] = step_pos[i];
        }
      }

      publish_simulated_joint_state(step_pos);
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    if (is_shutting_down_)
    {
      return {false, "Server shutting down"};
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (size_t i = 0; i < arm_joints_.size() && i < target_positions.size(); ++i)
      {
        current_joints_[arm_joints_[i]] = target_positions[i];
      }
    }
    publish_simulated_joint_state(target_positions);

    return {true, "Success"};
  }

  std::pair<bool, std::string> ManipulationActionServer::execute_named_pose(
      const std::string &pose_name,
      std::optional<double> velocity_rad_s)
  {
    auto target_opt = arm_motion_planner_.get_named_pose(pose_name);
    if (!target_opt.has_value())
    {
      return {false, "Unknown named pose '" + pose_name + "'."};
    }

    const auto start_positions = get_current_positions_or_default();
    const auto &target_positions = *target_opt;
    const double target_velocity = velocity_rad_s.value_or(max_velocity_);

    try
    {
      const auto traj = arm_motion_planner_.plan_trajectory_at_velocity(
          start_positions,
          target_positions,
          target_velocity,
          sampling_rate_hz_);
      return send_trajectory_and_wait(traj, nullptr);
    }
    catch (const std::exception &exc)
    {
      return {false, "Failed to plan trajectory for '" + pose_name + "': " + exc.what()};
    }
  }

  void ManipulationActionServer::on_command_named_pose(std_msgs::msg::String::ConstSharedPtr msg)
  {
    const std::string & pose_name = msg->data;

    bool expected = false;
    if (!is_executing_.compare_exchange_strong(expected, true))
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected named pose command '%s': manipulation server is busy.",
          pose_name.c_str());
      return;
    }

    set_snapshot("NAMED_POSE_" + pose_name, 0.0, "Named pose: " + pose_name);
    RCLCPP_INFO(get_logger(), "Executing named pose command: '%s'", pose_name.c_str());

    auto [ok, res_msg] = execute_named_pose(pose_name);
    if (!ok)
    {
      set_snapshot("FAILED", 0.0);
      RCLCPP_WARN(get_logger(), "Command named pose '%s' failed: %s", pose_name.c_str(), res_msg.c_str());
    }
    else
    {
      set_snapshot("IDLE", 100.0);
      RCLCPP_INFO(get_logger(), "Command named pose '%s' succeeded: %s", pose_name.c_str(), res_msg.c_str());
    }

    is_executing_ = false;
    diagnostic_updater_.force_update();
  }

  void ManipulationActionServer::execute_move(const std::shared_ptr<GoalHandleChessMove> goal_handle)
  {
    is_executing_ = true;
    const auto start_time = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteChessMove::Feedback>();
    auto result = std::make_shared<ExecuteChessMove::Result>();

    set_snapshot("STARTING", 0.0, goal->instruction);
    RCLCPP_INFO(get_logger(), "Executing chess move: '%s'", goal->instruction.c_str());

    const double gripper_open = arm_motion_planner_.gripper_open_rad();
    const double gripper_closed = arm_motion_planner_.gripper_closed_rad();

    const bool has_from = !goal->from_square.empty() ||
                          (goal->pick_point.x != 0.0 || goal->pick_point.y != 0.0 || goal->pick_point.z != 0.0);
    const bool has_to = !goal->to_square.empty() ||
                        (goal->place_point.x != 0.0 || goal->place_point.y != 0.0 || goal->place_point.z != 0.0);

    const bool is_pure_pick = !goal->is_capture && has_from && !has_to;
    const bool is_pure_place = !goal->is_capture && !has_from && has_to;

    std::optional<ApproachDescendPair> pick_pair;
    if (goal->is_capture || is_pure_pick || (!is_pure_place))
    {
      pick_pair = resolve_approach_and_descend(
          goal->pick_point, goal->target_frame, gripper_open, "pick", goal_handle, result, start_time);
      if (!pick_pair.has_value())
      {
        return;
      }
    }

    std::optional<ApproachDescendPair> place_pair;
    if (!goal->is_capture && (is_pure_place || (!is_pure_pick)))
    {
      place_pair = resolve_approach_and_descend(
          goal->place_point, goal->target_frame, gripper_closed, "place", goal_handle, result, start_time);
      if (!place_pair.has_value())
      {
        return;
      }
    }

    auto [phases, phase_targets] = build_active_phase_plan(
        goal, is_pure_pick, is_pure_place, pick_pair, place_pair);

    const double total_phases = static_cast<double>(phases.size());
    for (size_t idx = 0; idx < phases.size(); ++idx)
    {
      if (!execute_phase_step(
              idx, phases[idx], phase_targets.at(phases[idx]),
              total_phases, goal_handle, feedback, result, start_time))
      {
        return;
      }
    }

    set_snapshot("COMPLETED", 100.0);
    is_executing_ = false;

    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    result->success = true;
    result->message = "Completed move: " + goal->instruction;
    result->execution_time_sec = static_cast<float>(elapsed);
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "Move finished successfully in %.2fs", elapsed);
    diagnostic_updater_.force_update();
  }

  void ManipulationActionServer::abort_move(
      const std::shared_ptr<GoalHandleChessMove> &goal_handle,
      const std::shared_ptr<ExecuteChessMove::Result> &result,
      const std::string &message,
      double progress,
      std::chrono::steady_clock::time_point start_time)
  {
    set_snapshot("FAILED", progress);
    is_executing_ = false;
    result->success = false;
    result->message = message;
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    result->execution_time_sec = static_cast<float>(elapsed);
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    diagnostic_updater_.force_update();
  }

  std::optional<ManipulationActionServer::ApproachDescendPair>
  ManipulationActionServer::resolve_approach_and_descend(
      const geometry_msgs::msg::Point &target_pt,
      const std::string &target_frame,
      double gripper_rad,
      const std::string &stage_name,
      const std::shared_ptr<GoalHandleChessMove> &goal_handle,
      const std::shared_ptr<ExecuteChessMove::Result> &result,
      std::chrono::steady_clock::time_point start_time)
  {
    const geometry_msgs::msg::Point descend_pt = sanitize_target_point(target_pt, target_frame);
    geometry_msgs::msg::Point approach_pt = descend_pt;
    approach_pt.z += approach_z_offset_;

    auto approach_sol = resolve_point_ik(approach_pt, target_frame, gripper_rad);
    if (!approach_sol.has_value())
    {
      abort_move(
          goal_handle, result,
          "Target " + stage_name + " approach pose unreachable or IK failed for: (" +
              std::to_string(approach_pt.x) + ", " +
              std::to_string(approach_pt.y) + ", " +
              std::to_string(approach_pt.z) + ")",
          0.0, start_time);
      return std::nullopt;
    }

    auto descend_sol = resolve_point_ik(descend_pt, target_frame, gripper_rad);
    if (!descend_sol.has_value())
    {
      abort_move(
          goal_handle, result,
          "Target " + stage_name + " descend pose unreachable or IK failed for: (" +
              std::to_string(descend_pt.x) + ", " +
              std::to_string(descend_pt.y) + ", " +
              std::to_string(descend_pt.z) + ")",
          0.0, start_time);
      return std::nullopt;
    }

    return ApproachDescendPair{std::move(*approach_sol), std::move(*descend_sol)};
  }

  std::pair<std::vector<std::string>, std::unordered_map<std::string, std::vector<double>>>
  ManipulationActionServer::build_active_phase_plan(
      const std::shared_ptr<const ExecuteChessMove::Goal> &goal,
      bool is_pure_pick,
      bool is_pure_place,
      const std::optional<ApproachDescendPair> &pick_pair,
      const std::optional<ApproachDescendPair> &place_pair)
  {
    if (goal->is_capture)
    {
      const ClearBinSide bin_side = (goal->pick_point.y >= 0.0) ? ClearBinSide::LEFT : ClearBinSide::RIGHT;
      RCLCPP_INFO(
          get_logger(),
          "Executing capture clear move to bin: %s",
          (bin_side == ClearBinSide::LEFT) ? "CLEAR_LEFT" : "CLEAR_RIGHT");
      return {ArmMotionPlanner::kClearPhases,
              arm_motion_planner_.build_clear_targets(pick_pair->approach, pick_pair->descend, bin_side)};
    }
    if (is_pure_pick)
    {
      RCLCPP_INFO(
          get_logger(),
          "Executing atomic PICK move for square '%s' (concluding in transit stow)",
          goal->from_square.c_str());
      return {ArmMotionPlanner::kPickPhases,
              arm_motion_planner_.build_pick_targets(pick_pair->approach, pick_pair->descend)};
    }
    if (is_pure_place)
    {
      RCLCPP_INFO(
          get_logger(),
          "Executing atomic PLACE move for square '%s' (starting from transit stow)",
          goal->to_square.c_str());
      return {ArmMotionPlanner::kPlacePhases,
              arm_motion_planner_.build_place_targets(place_pair->approach, place_pair->descend)};
    }
    RCLCPP_INFO(
        get_logger(),
        "Executing full PICK_AND_PLACE move (%s -> %s)",
        goal->from_square.c_str(), goal->to_square.c_str());
    return {ArmMotionPlanner::kPickAndPlacePhases,
            arm_motion_planner_.build_phase_targets(
                pick_pair->approach, pick_pair->descend, place_pair->approach, place_pair->descend)};
  }

  bool ManipulationActionServer::execute_phase_step(
      size_t phase_idx,
      const std::string &phase_name,
      const std::vector<double> &target_pos,
      double total_phases,
      const std::shared_ptr<GoalHandleChessMove> &goal_handle,
      const std::shared_ptr<ExecuteChessMove::Feedback> &feedback,
      const std::shared_ptr<ExecuteChessMove::Result> &result,
      std::chrono::steady_clock::time_point start_time)
  {
    const double progress = (static_cast<double>(phase_idx) / total_phases) * 100.0;

    if (goal_handle->is_canceling() || is_shutting_down_)
    {
      set_snapshot("CANCELED", progress);
      is_executing_ = false;
      result->success = false;
      result->message = "Canceled during phase " + phase_name;
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      result->execution_time_sec = static_cast<float>(elapsed);
      goal_handle->canceled(result);
      RCLCPP_WARN(get_logger(), "Move canceled in %s", phase_name.c_str());
      diagnostic_updater_.force_update();
      return false;
    }

    set_snapshot(phase_name, progress);
    feedback->current_phase = phase_name;
    feedback->progress_percent = static_cast<float>(progress);
    goal_handle->publish_feedback(feedback);
    diagnostic_updater_.force_update();

    const auto current_pos = get_current_positions_or_default();
    trajectory_msgs::msg::JointTrajectory traj;
    try
    {
      traj = arm_motion_planner_.plan_trajectory_at_velocity(
          current_pos,
          target_pos,
          max_velocity_,
          sampling_rate_hz_);
    }
    catch (const std::exception &exc)
    {
      abort_move(
          goal_handle, result,
          "Planning failed in " + phase_name + ": " + exc.what(),
          progress, start_time);
      return false;
    }

    auto [ok, err_msg] = send_trajectory_and_wait(traj, goal_handle);
    if (!ok)
    {
      abort_move(
          goal_handle, result,
          "Execution failed in " + phase_name + ": " + err_msg,
          progress, start_time);
      return false;
    }

    return true;
  }

  void ManipulationActionServer::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    ExecutionSnapshot snap;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      snap = snapshot_;
    }
    const bool active = is_executing_.load();
    stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        active ? "Executing manipulation" : "Idle");
    stat.add("active", active);
    stat.add("current_phase", snap.phase);
    stat.add("progress_percent", snap.progress_percent);
    stat.add("instruction", snap.instruction);
    stat.add("controller_action", controller_action_name_);
    stat.add("mock_manipulation", mock_manipulation_ ? "true" : "false");
  }

} // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::ManipulationActionServer)

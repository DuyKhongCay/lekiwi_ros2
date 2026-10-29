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
 * @file manipulation_action_server.hpp
 * @brief High-level autonomous ROS 2 action server executing chess piece pick & place,
 *        bin clearing, and episode recorder lifecycle coordination.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__MANIPULATION_ACTION_SERVER_HPP_
#define LEKIWI_MOTION__MANIPULATION_ACTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <lekiwi_interfaces/srv/go_to_named_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lekiwi_motion/arm/arm_trajectory_executor.hpp"

namespace lekiwi_motion
{

/**
 * @class ManipulationActionServer
 * @brief Autonomous action server executing ExecuteChessMove operations (OP_MOVE,
 *        OP_CLEAR_*, OP_PICK, OP_PLACE) with live JIT analytical IK, and coordinating
 *        recording lifecycle with chess_episode_recorder.
 */
class ManipulationActionServer : public rclcpp::Node
{
public:
  using ExecuteChessMove = lekiwi_interfaces::action::ExecuteChessMove;
  using GoalHandleChessMove = rclcpp_action::ServerGoalHandle<ExecuteChessMove>;
  using Trigger = std_srvs::srv::Trigger;
  using GoToNamedPose = lekiwi_interfaces::srv::GoToNamedPose;

  explicit ManipulationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManipulationActionServer() override;

  [[nodiscard]] bool is_waiting_record() const noexcept
  {
    return is_waiting_record_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool is_goal_active() const noexcept
  {
    return is_goal_active_.load(std::memory_order_acquire);
  }

private:
  struct ApproachDescendPair
  {
    std::vector<double> approach;
    std::vector<double> descend;
  };

  struct MoveWaypoints
  {
    std::optional<ApproachDescendPair> pick;
    std::optional<ApproachDescendPair> place;
  };

  struct MoveExecutionContext
  {
    std::shared_ptr<GoalHandleChessMove> goal_handle;
    std::shared_ptr<const ExecuteChessMove::Goal> goal;
    std::shared_ptr<ExecuteChessMove::Feedback> feedback;
    std::shared_ptr<ExecuteChessMove::Result> result;
    std::chrono::steady_clock::time_point start_time;
  };

  enum class RecordCoordinationSignal
  {
    NONE,
    SAVE,
    SKIP,
    CANCEL
  };

  void declare_and_init_parameters();

  // Record Coordination Services
  void handle_save_episode(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  void handle_skip_episode(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  void handle_named_pose_request(
    const std::shared_ptr<GoToNamedPose::Request> request,
    std::shared_ptr<GoToNamedPose::Response> response);

  bool forward_trigger_request(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & service_label,
    const std::shared_ptr<Trigger::Response> & response);

  void notify_record_signal(RecordCoordinationSignal signal) noexcept;
  void discard_active_recording_async() noexcept;

  // Action Server Callbacks & Move Orchestration
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteChessMove::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleChessMove> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleChessMove> goal_handle);
  void execute_move(const std::shared_ptr<GoalHandleChessMove> goal_handle);

  // Helper Methods for execute_move (Cognitive Complexity Reduction)
  [[nodiscard]] std::string resolve_instruction_label(
    const ExecuteChessMove::Goal & goal) const;

  void execute_record_mode(
    const MoveExecutionContext & ctx,
    const std::string & instruction_label);

  [[nodiscard]] std::optional<std::vector<double>> compute_approach_joints(
    const MoveExecutionContext & ctx,
    bool is_pure_place);

  void process_record_signal(
    const MoveExecutionContext & ctx,
    RecordCoordinationSignal sig);

  [[nodiscard]] RecordCoordinationSignal wait_for_record_signal(
    const MoveExecutionContext & ctx);

  bool execute_phase_pipeline(
    const MoveExecutionContext & ctx,
    const std::vector<std::string> & phase_names,
    const std::unordered_map<std::string, std::vector<double>> & phase_targets,
    const std::string & instruction_label);

  [[nodiscard]] std::optional<MoveWaypoints> resolve_move_waypoints(
    const MoveExecutionContext & ctx);

  [[nodiscard]] std::optional<ApproachDescendPair> resolve_approach_and_descend(
    const MoveExecutionContext & ctx,
    const geometry_msgs::msg::Point & target_pt,
    double gripper_rad,
    const std::string & stage_name);

  [[nodiscard]] std::pair<
    std::vector<std::string>,
    std::unordered_map<std::string, std::vector<double>>>
  build_active_phase_plan(
    const std::shared_ptr<const ExecuteChessMove::Goal> & goal,
    const std::optional<ApproachDescendPair> & pick_pair,
    const std::optional<ApproachDescendPair> & place_pair);

  bool execute_phase_step(
    size_t phase_idx,
    const std::string & phase_name,
    const std::vector<double> & target_pos,
    double total_phases,
    const MoveExecutionContext & ctx,
    const std::string & instruction_label);

  bool check_and_handle_preemption(
    const MoveExecutionContext & ctx,
    const std::string & phase_name,
    double progress);

  void publish_phase_feedback(
    const MoveExecutionContext & ctx,
    const std::string & phase_name,
    double progress,
    const std::string & instruction_label);

  enum class ActionTerminalState
  {
    SUCCEEDED,
    CANCELED,
    ABORTED
  };

  void finalize_move(
    const MoveExecutionContext & ctx,
    ActionTerminalState term_state,
    const std::string & message,
    double progress = 100.0);

  void succeed_move(const MoveExecutionContext & ctx, const std::string & message);
  void cancel_move(const MoveExecutionContext & ctx, const std::string & message, double progress);
  void abort_move(const MoveExecutionContext & ctx, const std::string & message, double progress);

  void set_snapshot(
    const std::string & phase,
    double progress_percent,
    const std::string & instruction = "");
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Configuration Parameters
  std::string action_name_{"/manipulation/execute_chess_move"};
  double approach_z_offset_{0.05};
  bool record_{false};

  // Trajectory Execution Engine (Shared Core)
  std::unique_ptr<arm::ArmTrajectoryExecutor> executor_;

  // Record Mode Coordination
  std::condition_variable record_cv_;
  std::mutex record_mutex_;
  RecordCoordinationSignal record_signal_{RecordCoordinationSignal::NONE};
  std::atomic<bool> is_waiting_record_{false};

  // Callback Groups & ROS 2 Interfaces
  rclcpp::CallbackGroup::SharedPtr cb_group_action_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_recorder_coordination_;
  rclcpp_action::Server<ExecuteChessMove>::SharedPtr action_server_;
  rclcpp::Service<Trigger>::SharedPtr save_episode_srv_;
  rclcpp::Service<Trigger>::SharedPtr skip_episode_srv_;
  rclcpp::Service<GoToNamedPose>::SharedPtr named_pose_srv_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<Trigger>::SharedPtr discard_client_;

  // Diagnostics & Execution State
  diagnostic_updater::Updater diagnostic_updater_;
  mutable std::mutex snapshot_mutex_;
  struct ExecutionSnapshot
  {
    std::string phase{"IDLE"};
    double progress_percent{0.0};
    std::string instruction{""};
  } snapshot_;

  std::atomic<bool> is_goal_active_{false};
  std::atomic<bool> is_shutting_down_{false};
  std::thread execution_thread_;
};

}  // namespace lekiwi_motion

#endif  // LEKIWI_MOTION__MANIPULATION_ACTION_SERVER_HPP_

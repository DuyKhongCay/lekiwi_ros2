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
 * @brief Composable ROS 2 Node orchestrating multi-phase chess manipulation and named poses.
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
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lekiwi_motion/arm/arm_motion_planner.hpp"
#include "lekiwi_motion/arm/arm_trajectory_executor.hpp"

namespace lekiwi_motion
{

class ManipulationActionServer : public rclcpp::Node
{
public:
  using ExecuteChessMove = lekiwi_interfaces::action::ExecuteChessMove;
  using GoalHandleChessMove = rclcpp_action::ServerGoalHandle<ExecuteChessMove>;
  using GoToNamedPose = lekiwi_interfaces::srv::GoToNamedPose;
  using Trigger = std_srvs::srv::Trigger;

  explicit ManipulationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManipulationActionServer() override;

  void accept_robot_description(const std::string & urdf_xml);

  [[nodiscard]] const std::vector<std::string> & arm_joints() const noexcept
  {
    return arm_joints_;
  }

  [[nodiscard]] std::optional<std::vector<double>> get_named_pose(
    const std::string & pose_name) const;

  [[nodiscard]] std::vector<double> get_current_positions_or_default() const;

  [[nodiscard]] std::optional<std::vector<double>> resolve_point_ik(
    const geometry_msgs::msg::Point & point,
    const std::string & target_frame,
    double gripper_state) const;

  [[nodiscard]] bool is_waiting_record() const noexcept { return is_waiting_record_.load(std::memory_order_acquire); }

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
    ABORT,
    CANCEL
  };

  void declare_and_init_parameters();
  void set_runtime_approach_pose(const std::vector<double> & joints);

  // Service Callbacks & Helpers
  void handle_named_pose_request(
    const std::shared_ptr<GoToNamedPose::Request> request,
    std::shared_ptr<GoToNamedPose::Response> response);

  [[nodiscard]] std::optional<std::vector<double>> validate_named_pose_request(
    const std::shared_ptr<GoToNamedPose::Request> & request,
    const std::shared_ptr<GoToNamedPose::Response> & response) const;

  void handle_toggle_gripper(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  [[nodiscard]] bool execute_manual_joint_target(
    const std::vector<double> & start_positions,
    const std::vector<double> & target_positions,
    const std::string & label,
    std::string & out_message);

  [[nodiscard]] arm::TrajectoryExecutionGuard acquire_trajectory_guard();
  void publish_arm_busy(bool is_busy);

  void handle_save_episode(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  void handle_abort_episode(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

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
  void execute_record_mode(const MoveExecutionContext & ctx, bool is_pure_place);

  [[nodiscard]] RecordCoordinationSignal wait_for_record_signal(
    const std::shared_ptr<GoalHandleChessMove> & goal_handle);

  [[nodiscard]] std::optional<MoveWaypoints> resolve_move_waypoints(
    const MoveExecutionContext & ctx,
    bool is_pure_pick,
    bool is_pure_place);

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
    bool is_pure_pick,
    bool is_pure_place,
    const std::optional<ApproachDescendPair> & pick_pair,
    const std::optional<ApproachDescendPair> & place_pair);

  bool execute_phase_step(
    size_t phase_idx,
    const std::string & phase_name,
    const std::vector<double> & target_pos,
    double total_phases,
    const MoveExecutionContext & ctx);

  bool check_and_handle_preemption(
    const MoveExecutionContext & ctx,
    const std::string & phase_name,
    double progress);

  void publish_phase_feedback(
    const MoveExecutionContext & ctx,
    const std::string & phase_name,
    double progress);

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
  std::string controller_action_name_{"/arm_trajectory_controller/follow_joint_trajectory"};
  std::string arm_busy_topic_{"/arm/is_busy"};
  double approach_z_offset_{0.05};
  double default_velocity_{1.5};
  double sampling_rate_hz_{50.0};
  bool mock_execution_{false};
  bool record_{false};
  std::vector<std::string> arm_joints_{
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"
  };
  std::vector<double> default_approach_pose_{};

  // Domain Planner & Trajectory Executor
  mutable std::mutex planner_mutex_;
  arm::ArmMotionPlanner arm_motion_planner_;
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
  rclcpp::Service<GoToNamedPose>::SharedPtr named_pose_srv_;
  rclcpp::Service<Trigger>::SharedPtr toggle_gripper_srv_;
  rclcpp::Service<Trigger>::SharedPtr save_episode_srv_;
  rclcpp::Service<Trigger>::SharedPtr abort_episode_srv_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<Trigger>::SharedPtr discard_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arm_busy_;

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

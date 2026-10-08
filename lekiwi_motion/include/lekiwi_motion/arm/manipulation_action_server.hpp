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
 * @brief Composable ROS 2 Action Server executing multi-phase pick-and-place manipulation.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__MANIPULATION_ACTION_SERVER_HPP_
#define LEKIWI_MOTION__MANIPULATION_ACTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <lekiwi_interfaces/action/execute_chess_move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <optional>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "lekiwi_motion/arm/arm_motion_planner.hpp"
#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace lekiwi_motion
{

class ManipulationActionServer : public rclcpp::Node
{
public:
  using ExecuteChessMove = lekiwi_interfaces::action::ExecuteChessMove;
  using GoalHandleChessMove = rclcpp_action::ServerGoalHandle<ExecuteChessMove>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit ManipulationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManipulationActionServer() override;

  [[nodiscard]] std::vector<double> get_current_positions_or_default();
  [[nodiscard]] const std::vector<std::string> & arm_joints() const noexcept { return arm_joints_; }

  void init_kinematics_and_tf();
  void accept_robot_description(const std::string & xml);

  [[nodiscard]] std::optional<std::vector<double>> resolve_point_ik(
    const geometry_msgs::msg::Point & point,
    const std::string & target_frame,
    double gripper_state);

  std::pair<bool, std::string> execute_named_pose(
    const std::string & pose_name,
    std::optional<double> velocity_rad_s = std::nullopt);
  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> named_poses() const
  {
    return arm_motion_planner_.named_poses().to_map();
  }
  [[nodiscard]] const ArmMotionPlanner & planner() const noexcept { return arm_motion_planner_; }
  [[nodiscard]] bool is_mock_manipulation() const noexcept { return mock_manipulation_; }
  [[nodiscard]] double max_velocity() const noexcept { return max_velocity_; }

private:
  void declare_and_load_parameters();
  void on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg);

  void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void on_command_named_pose(std_msgs::msg::String::ConstSharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteChessMove::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleChessMove> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleChessMove> goal_handle);
  void execute_move(const std::shared_ptr<GoalHandleChessMove> goal_handle);

  std::pair<bool, std::string> send_trajectory_and_wait(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::shared_ptr<GoalHandleChessMove> & goal_handle = nullptr);

  std::pair<bool, std::string> execute_mock_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::shared_ptr<GoalHandleChessMove> & goal_handle = nullptr);

  void publish_simulated_joint_state(const std::vector<double> & positions);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  [[nodiscard]] geometry_msgs::msg::Point sanitize_target_point(
    const geometry_msgs::msg::Point & point,
    const std::string & target_frame) const;

  [[nodiscard]] std::optional<geometry_msgs::msg::PointStamped> transform_to_base(
    const geometry_msgs::msg::PointStamped & in_pt) const;

  struct ApproachDescendPair
  {
    std::vector<double> approach;
    std::vector<double> descend;
  };

  void abort_move(
    const std::shared_ptr<GoalHandleChessMove> & goal_handle,
    const std::shared_ptr<ExecuteChessMove::Result> & result,
    const std::string & message,
    double progress,
    std::chrono::steady_clock::time_point start_time);

  [[nodiscard]] std::optional<ApproachDescendPair> resolve_approach_and_descend(
    const geometry_msgs::msg::Point & target_pt,
    const std::string & target_frame,
    double gripper_rad,
    const std::string & stage_name,
    const std::shared_ptr<GoalHandleChessMove> & goal_handle,
    const std::shared_ptr<ExecuteChessMove::Result> & result,
    std::chrono::steady_clock::time_point start_time);

  [[nodiscard]] std::pair<std::vector<std::string>, std::unordered_map<std::string, std::vector<double>>>
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
    const std::shared_ptr<GoalHandleChessMove> & goal_handle,
    const std::shared_ptr<ExecuteChessMove::Feedback> & feedback,
    const std::shared_ptr<ExecuteChessMove::Result> & result,
    std::chrono::steady_clock::time_point start_time);

  std::pair<bool, std::string> wait_for_controller_result(
    rclcpp_action::Client<FollowJointTrajectory>::GoalHandle::SharedPtr handle,
    double expected_sec,
    const std::shared_ptr<GoalHandleChessMove> & goal_handle);

  // Parameters
  std::string action_name_;
  std::string controller_action_name_;
  std::string joint_states_topic_;
  double max_velocity_{1.5};
  double approach_z_offset_{0.05};
  double sampling_rate_hz_{50.0};
  std::vector<std::string> arm_joints_;
  bool mock_manipulation_{false};

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cb_group_action_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_controller_client_;

  // ROS 2 communication entities
  rclcpp_action::Server<ExecuteChessMove>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr controller_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mock_joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_pose_sub_;

  // Diagnostic updater and thread-safe snapshot
  diagnostic_updater::Updater diagnostic_updater_;
  mutable std::mutex snapshot_mutex_;
  struct ExecutionSnapshot
  {
    std::string phase{"IDLE"};
    double progress_percent{0.0};
    std::string instruction{""};
  } snapshot_;

  void set_snapshot(const std::string & phase, double progress_percent, const std::string & instruction = "");

  // State & Threading
  std::mutex state_mutex_;
  std::unordered_map<std::string, double> current_joints_;
  std::atomic<bool> is_executing_{false};
  std::atomic<bool> is_shutting_down_{false};
  std::thread execution_thread_;

  // Arm Motion Planner (Phase sequencing & Quintic trajectory generator)
  ArmMotionPlanner arm_motion_planner_;

  // TF & Kinematics entities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<workspace::SO101AnalyticalSolver> analytical_solver_;
  workspace::KinematicsModel kinematics_model_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  // Frame parameters
  std::string board_frame_{"chessboard_frame"};
  std::string base_frame_{"base_footprint"};
  std::string tip_frame_{"gripperframe"};
  double grasp_z_{0.035};
  double default_pitch_{-1.57079632679};
  double default_roll_{0.0};
};

}  // namespace lekiwi_motion

#endif  // LEKIWI_MOTION__MANIPULATION_ACTION_SERVER_HPP_

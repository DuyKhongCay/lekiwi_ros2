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
 * @file arm_trajectory_executor.hpp
 * @brief Thread-safe arm trajectory actuation, kinematics ownership, landmark poses,
 *        and controller action execution for LeKiwi arm.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__ARM__ARM_TRAJECTORY_EXECUTOR_HPP_
#define LEKIWI_MOTION__ARM__ARM_TRAJECTORY_EXECUTOR_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "lekiwi_motion/arm/arm_motion_planner.hpp"
#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace lekiwi_motion::arm
{

/**
 * @struct ArmTrajectoryExecutorConfig
 * @brief Configuration parameters for ArmTrajectoryExecutor initialization.
 */
struct ArmTrajectoryExecutorConfig
{
  std::vector<std::string> arm_joints{
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"
  };
  std::string controller_action_name{"/arm_trajectory_controller/follow_joint_trajectory"};
  std::string joint_states_topic{"/joint_states"};
  std::string robot_description_topic{"/robot_description"};
  std::string board_frame{"chessboard_frame"};
  std::string base_frame{"base_footprint"};
  std::string tip_frame{"gripperframe"};
  double default_pitch{-1.57079632679};
  double default_roll{0.0};
  double default_velocity{1.5};
  double sampling_rate_hz{50.0};
  NamedPosesConfig named_poses{
    {},                                                         // approach (dynamically computed)
    {1.5616, -1.7411, 1.5570, 0.7747, -2.7888, 0.0813},       // stow
    {-1.9804, -0.3375, 0.3927, 1.4972, -1.5631, 0.0828},     // clear_left
    {2.0632, -0.2301, 0.3405, 1.5079, -1.5723, 0.0828}       // clear_right
  };
  GripperConfig gripper_cfg{1.50, 0.00};
  bool mock_execution{false};
};

/**
 * @class TrajectoryExecutionGuard
 * @brief Lock-free RAII guard enforcing mutually exclusive physical trajectory execution.
 */
class TrajectoryExecutionGuard
{
public:
  explicit TrajectoryExecutionGuard(std::atomic<bool> & busy_flag) noexcept
  : busy_flag_(&busy_flag),
    acquired_(false)
  {
    bool expected = false;
    acquired_ = busy_flag_->compare_exchange_strong(expected, true, std::memory_order_acq_rel);
  }

  ~TrajectoryExecutionGuard()
  {
    if (acquired_ && busy_flag_) {
      busy_flag_->store(false, std::memory_order_release);
    }
  }

  TrajectoryExecutionGuard(TrajectoryExecutionGuard && other) noexcept
  : busy_flag_(other.busy_flag_),
    acquired_(other.acquired_)
  {
    other.acquired_ = false;
    other.busy_flag_ = nullptr;
  }

  TrajectoryExecutionGuard & operator=(TrajectoryExecutionGuard && other) noexcept
  {
    if (this != &other) {
      if (acquired_ && busy_flag_) {
        busy_flag_->store(false, std::memory_order_release);
      }
      busy_flag_ = other.busy_flag_;
      acquired_ = other.acquired_;
      other.acquired_ = false;
      other.busy_flag_ = nullptr;
    }
    return *this;
  }

  [[nodiscard]] bool acquired() const noexcept { return acquired_; }

  TrajectoryExecutionGuard(const TrajectoryExecutionGuard &) = delete;
  TrajectoryExecutionGuard & operator=(const TrajectoryExecutionGuard &) = delete;

private:
  std::atomic<bool> * busy_flag_{nullptr};
  bool acquired_{false};
};

/**
 * @class ArmTrajectoryExecutor
 * @brief Centralized execution engine managing kinematics solver, landmark poses,
 *        joint telemetry, trajectory interpolation, and action client execution.
 */
class ArmTrajectoryExecutor
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using KinematicsCallback = std::function<void(
    const std::shared_ptr<workspace::SO101AnalyticalSolver> &,
    const workspace::KinematicsModel &)>;

  ArmTrajectoryExecutor(rclcpp::Node * node, ArmTrajectoryExecutorConfig config);
  ~ArmTrajectoryExecutor();

  ArmTrajectoryExecutor(const ArmTrajectoryExecutor &) = delete;
  ArmTrajectoryExecutor & operator=(const ArmTrajectoryExecutor &) = delete;

  void stop() noexcept;

  // URDF & Kinematics Lifecycle
  bool accept_robot_description(const std::string & urdf_xml);
  void set_kinematics_callback(KinematicsCallback cb);

  [[nodiscard]] bool is_kinematics_initialized() const noexcept
  {
    return is_kinematics_initialized_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool has_kinematics_failed() const noexcept
  {
    return has_kinematics_failed_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::string kinematics_error_msg() const;
  [[nodiscard]] std::shared_ptr<workspace::SO101AnalyticalSolver> analytical_solver() const;
  [[nodiscard]] workspace::KinematicsModel kinematics_model() const;

  // Joint Telemetry
  [[nodiscard]] const std::vector<std::string> & arm_joints() const noexcept
  {
    return config_.arm_joints;
  }

  [[nodiscard]] bool has_joint_states() const noexcept
  {
    return has_joint_states_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::vector<double> get_current_positions_or_default() const;
  [[nodiscard]] std::optional<std::array<double, 5>> get_active_arm_positions() const;

  [[nodiscard]] bool is_trajectory_busy() const noexcept
  {
    return is_trajectory_busy_.load(std::memory_order_relaxed);
  }

  // Landmark & Gripper Configuration
  [[nodiscard]] std::optional<std::vector<double>> get_named_pose(const std::string & name) const;
  [[nodiscard]] bool has_named_pose(const std::string & name) const;
  bool set_named_pose(const std::string & name, std::vector<double> joints);
  [[nodiscard]] GripperConfig gripper_config() const;
  [[nodiscard]] double clamp_gripper(double target_rad) const;
  [[nodiscard]] double gripper_open_rad() const;
  [[nodiscard]] double gripper_closed_rad() const;

  // JIT Inverse Kinematics
  [[nodiscard]] std::optional<std::vector<double>> resolve_point_ik(
    const geometry_msgs::msg::Point & point,
    const std::string & target_frame,
    double clamped_gripper_rad) const;

  // Phase Plan Building & Trajectory Interpolation
  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_phase_targets(
    const std::vector<double> & pick_approach,
    const std::vector<double> & pick_descend,
    const std::vector<double> & place_approach,
    const std::vector<double> & place_descend) const;

  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_clear_targets(
    const std::vector<double> & pick_approach,
    const std::vector<double> & pick_descend,
    arm::ClearBinSide side) const;

  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_pick_targets(
    const std::vector<double> & pick_approach,
    const std::vector<double> & pick_descend) const;

  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_place_targets(
    const std::vector<double> & place_approach,
    const std::vector<double> & place_descend) const;

  [[nodiscard]] trajectory_msgs::msg::JointTrajectory plan_trajectory_at_velocity(
    const std::vector<double> & start_joints,
    const std::vector<double> & target_joints,
    double target_velocity_rad_s = -1.0) const;

  // Synchronous High-Level Execution
  [[nodiscard]] std::pair<bool, std::string> execute_joint_target(
    const std::vector<double> & target_positions,
    const std::string & label = "",
    double velocity = -1.0,
    std::function<bool()> is_canceling = nullptr);

  [[nodiscard]] std::pair<bool, std::string> execute_named_pose(
    const std::string & name,
    double velocity = -1.0,
    std::function<bool()> is_canceling = nullptr);

  [[nodiscard]] std::pair<bool, std::string> execute_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::function<bool()> is_canceling = nullptr);

private:
  void init_parameters();
  void init_trajectory_client_and_subscribers();
  void init_kinematics_and_tf();
  void on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg);
  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

  [[nodiscard]] std::optional<geometry_msgs::msg::PointStamped> transform_to_base(
    const geometry_msgs::msg::PointStamped & in_pt) const;

  [[nodiscard]] std::pair<bool, std::string> execute_mock_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::function<bool()> & is_canceling);

  [[nodiscard]] std::pair<bool, std::string> execute_controller_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::function<bool()> & is_canceling);

  [[nodiscard]] std::pair<bool, std::string> wait_for_controller_result(
    const GoalHandleFJT::SharedPtr & handle,
    double expected_sec,
    const std::function<bool()> & is_canceling);

  rclcpp::Node * node_;
  ArmTrajectoryExecutorConfig config_;

  // Motion Planning & Landmark Poses
  mutable std::mutex planner_mutex_;
  arm::ArmMotionPlanner motion_planner_;

  // Controller Client & Subscriptions
  rclcpp::CallbackGroup::SharedPtr cb_group_controller_client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_joint_state_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr controller_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  // Telemetry Cache
  mutable std::mutex state_mutex_;
  std::unordered_map<std::string, double> current_joints_;
  std::atomic<bool> has_joint_states_{false};

  // Kinematics & TF
  mutable std::mutex kinematics_mutex_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<workspace::SO101AnalyticalSolver> analytical_solver_;
  workspace::KinematicsModel kinematics_model_{};
  std::atomic<bool> is_kinematics_initialized_{false};
  std::atomic<bool> has_kinematics_failed_{false};
  std::string kinematics_error_msg_{};
  KinematicsCallback kinematics_callback_{nullptr};

  // State
  std::atomic<bool> is_trajectory_busy_{false};
  std::atomic<bool> is_shutting_down_{false};
};

}  // namespace lekiwi_motion::arm

#endif  // LEKIWI_MOTION__ARM__ARM_TRAJECTORY_EXECUTOR_HPP_

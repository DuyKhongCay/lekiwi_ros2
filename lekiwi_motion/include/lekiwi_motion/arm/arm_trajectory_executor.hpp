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
 * @brief Thread-safe arm trajectory actuation, joint telemetry, and TF-backed analytical IK resolver.
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
  std::string board_frame{"chessboard_frame"};
  std::string base_frame{"base_footprint"};
  std::string tip_frame{"gripperframe"};
  double default_pitch{-1.57079632679};
  double default_roll{0.0};
  bool mock_execution{false};
};

/**
 * @class TrajectoryExecutionGuard
 * @brief Lock-free RAII guard enforcing mutually exclusive physical trajectory execution.
 */
class TrajectoryExecutionGuard
{
public:
  using TransitionCallback = std::function<void(bool /*is_busy*/)>;

  explicit TrajectoryExecutionGuard(
    std::atomic<bool> & busy_flag,
    TransitionCallback on_transition = nullptr) noexcept
  : busy_flag_(&busy_flag),
    on_transition_(std::move(on_transition)),
    acquired_(false)
  {
    bool expected = false;
    acquired_ = busy_flag_->compare_exchange_strong(expected, true, std::memory_order_acq_rel);
    if (acquired_ && on_transition_) {
      try {
        on_transition_(true);
      } catch (...) {}
    }
  }

  ~TrajectoryExecutionGuard()
  {
    if (acquired_ && busy_flag_) {
      busy_flag_->store(false, std::memory_order_release);
      if (on_transition_) {
        try {
          on_transition_(false);
        } catch (...) {}
      }
    }
  }

  TrajectoryExecutionGuard(TrajectoryExecutionGuard && other) noexcept
  : busy_flag_(other.busy_flag_),
    on_transition_(std::move(other.on_transition_)),
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
        if (on_transition_) {
          try {
            on_transition_(false);
          } catch (...) {}
        }
      }
      busy_flag_ = other.busy_flag_;
      on_transition_ = std::move(other.on_transition_);
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
  TransitionCallback on_transition_{nullptr};
  bool acquired_{false};
};

/**
 * @class ArmTrajectoryExecutor
 * @brief Manages joint state telemetry, TF-transformed analytical IK, and trajectory execution.
 */
class ArmTrajectoryExecutor
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  ArmTrajectoryExecutor(rclcpp::Node * node, ArmTrajectoryExecutorConfig config);
  ~ArmTrajectoryExecutor();

  ArmTrajectoryExecutor(const ArmTrajectoryExecutor &) = delete;
  ArmTrajectoryExecutor & operator=(const ArmTrajectoryExecutor &) = delete;

  void stop() noexcept;
  void accept_robot_description(const std::string & urdf_xml);

  [[nodiscard]] const std::vector<std::string> & arm_joints() const noexcept
  {
    return config_.arm_joints;
  }

  [[nodiscard]] std::vector<double> get_current_positions_or_default() const;

  [[nodiscard]] bool is_trajectory_busy() const noexcept
  {
    return is_trajectory_busy_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::atomic<bool> & busy_flag() noexcept
  {
    return is_trajectory_busy_;
  }

  [[nodiscard]] std::optional<std::vector<double>> resolve_point_ik(
    const geometry_msgs::msg::Point & point,
    const std::string & target_frame,
    double clamped_gripper_rad) const;

  [[nodiscard]] std::pair<bool, std::string> execute_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::function<bool()> is_canceling = nullptr);

private:
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

  rclcpp::CallbackGroup::SharedPtr cb_group_controller_client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_joint_state_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr controller_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  mutable std::mutex state_mutex_;
  std::unordered_map<std::string, double> current_joints_;

  mutable std::mutex kinematics_mutex_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<workspace::SO101AnalyticalSolver> analytical_solver_;

  std::atomic<bool> is_trajectory_busy_{false};
  std::atomic<bool> is_shutting_down_{false};
};

}  // namespace lekiwi_motion::arm

#endif  // LEKIWI_MOTION__ARM__ARM_TRAJECTORY_EXECUTOR_HPP_

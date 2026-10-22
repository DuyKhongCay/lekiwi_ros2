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
 * @file cartesian_jog_server_node.hpp
 * @brief Standalone ROS 2 node executing analytical IK Cartesian jogging,
 *        landmark named poses, and gripper toggling via ArmTrajectoryExecutor.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__ARM__CARTESIAN_JOG_SERVER_NODE_HPP_
#define LEKIWI_MOTION__ARM__CARTESIAN_JOG_SERVER_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lekiwi_interfaces/srv/go_to_named_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "lekiwi_motion/arm/arm_trajectory_executor.hpp"
#include "lekiwi_motion/arm/cartesian_jogger.hpp"
#include "lekiwi_motion/arm/joint_motion_filter.hpp"
#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace lekiwi_motion::arm
{

/**
 * @enum ControllerMode
 * @brief Target ros2_control controller type for streaming arm commands.
 */
enum class ControllerMode : uint8_t
{
  TRAJECTORY,  ///< Streams trajectory_msgs/msg/JointTrajectory to arm_trajectory_controller
  FORWARD      ///< Streams std_msgs/msg/Float64MultiArray to arm_forward_controller
};

[[nodiscard]] constexpr std::string_view to_string(ControllerMode mode) noexcept
{
  switch (mode) {
    case ControllerMode::TRAJECTORY:
      return "trajectory";
    case ControllerMode::FORWARD:
      return "forward";
  }
  return "unknown";
}

/**
 * @class CartesianJogServerNode
 * @brief Unified ROS 2 interactive arm server node translating Cartesian velocity twists to joint commands,
 *        and delegating landmark named poses and gripper toggling to ArmTrajectoryExecutor.
 *
 * Subscribes to:
 * - ~/cmd_vel (geometry_msgs::msg::TwistStamped): Cartesian TCP linear & angular velocity.
 * - /joint_states (sensor_msgs::msg::JointState): Telemetry used for bumpless FK sync.
 *
 * Publishes to:
 * - /arm_trajectory_controller/joint_trajectory (trajectory_msgs::msg::JointTrajectory): 50 Hz streaming (if TRAJECTORY mode).
 * - /arm_forward_controller/commands (std_msgs::msg::Float64MultiArray): 50 Hz streaming (if FORWARD mode).
 *
 * Services:
 * - ~/go_to_named_pose (lekiwi_interfaces::srv::GoToNamedPose): Landmark pose transition.
 * - ~/toggle_gripper (std_srvs::srv::Trigger): Reversible gripper open/close actuation.
 */
class CartesianJogServerNode : public rclcpp::Node
{
public:
  using GoToNamedPose = lekiwi_interfaces::srv::GoToNamedPose;
  using Trigger = std_srvs::srv::Trigger;

  explicit CartesianJogServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CartesianJogServerNode() override;

  [[nodiscard]] bool is_kinematics_initialized() const noexcept;
  [[nodiscard]] bool has_kinematics_failed() const noexcept;
  [[nodiscard]] std::string kinematics_error_msg() const;
  [[nodiscard]] bool is_active() const noexcept;
  [[nodiscard]] bool is_executing_trajectory() const noexcept;
  [[nodiscard]] ControllerMode controller_mode() const noexcept { return controller_mode_; }

private:
  void declare_all_parameters();
  void init_executor_and_kinematics();

  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  void control_loop();
  [[nodiscard]] bool check_jog_prerequisites();
  void handle_suppressed_or_idle(
    const std::array<double, 5> & physical_joints, bool is_idle_timeout);
  void execute_active_jog(
    const geometry_msgs::msg::Twist & twist, const std::array<double, 5> & physical_joints);
  void dispatch_command(const std::array<double, 5> & target_joints, double dt);

  // Named Pose & Gripper Services
  void handle_named_pose_request(
    const std::shared_ptr<GoToNamedPose::Request> request,
    std::shared_ptr<GoToNamedPose::Response> response);

  void handle_toggle_gripper(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  // Jog Parameters
  double publish_rate_hz_{50.0};
  double point_dt_s_{0.08};
  double stale_timeout_s_{0.25};
  double lpf_alpha_{0.85};
  double max_joint_vel_rad_s_{2.0};
  std::string base_frame_{"base_footprint"};
  ControllerMode controller_mode_{ControllerMode::TRAJECTORY};
  std::string arm_trajectory_topic_{"/arm_trajectory_controller/joint_trajectory"};
  std::string arm_forward_topic_{"/arm_forward_controller/commands"};

  std::vector<std::string> jog_joint_names_{
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll"
  };

  // Callback Groups & ROS Interfaces
  rclcpp::CallbackGroup::SharedPtr cb_group_services_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_forward_cmd_;
  rclcpp::Service<GoToNamedPose>::SharedPtr named_pose_srv_;
  rclcpp::Service<Trigger>::SharedPtr toggle_gripper_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Shared Execution Engine
  std::unique_ptr<arm::ArmTrajectoryExecutor> executor_;

  // Kinematic & Control Components (Jogger)
  mutable std::mutex kinematics_mutex_;
  std::shared_ptr<workspace::SO101AnalyticalSolver> ik_solver_{nullptr};
  workspace::KinematicsModel kinematics_model_{};
  std::unique_ptr<CartesianJogger> jogger_{nullptr};
  JointMotionFilter<5> motion_filter_{0.85};

  // State Tracking
  mutable std::mutex state_mutex_;
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist latest_twist_{};
  bool active_jogging_{false};
};

}  // namespace lekiwi_motion::arm

#endif  // LEKIWI_MOTION__ARM__CARTESIAN_JOG_SERVER_NODE_HPP_

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
 * @brief Standalone ROS 2 node executing analytical IK Cartesian jogging with bumpless transfer.
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
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "lekiwi_motion/arm/cartesian_jogger.hpp"
#include "lekiwi_motion/arm/joint_motion_filter.hpp"
#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace lekiwi_motion::arm
{

/**
 * @class CartesianJogServerNode
 * @brief Independent 5-DoF ROS 2 server node translating Cartesian velocity twists to arm joint commands.
 *
 * Subscribes to:
 * - ~/cmd_vel (geometry_msgs::msg::TwistStamped): Cartesian TCP linear & angular velocity.
 * - /joint_states (sensor_msgs::msg::JointState): Telemetry used for bumpless FK sync.
 * - /robot_description (std_msgs::msg::String): Robot URDF string for kinematics initialization.
 *
 * Publishes to:
 * - /arm_forward_controller/commands (std_msgs::msg::Float64MultiArray): In forward_position mode.
 * - /arm_trajectory_controller/joint_trajectory (trajectory_msgs::msg::JointTrajectory): In joint_trajectory mode.
 */
class CartesianJogServerNode : public rclcpp::Node
{
public:
  enum class KinematicsState
  {
    WAITING_FOR_URDF,
    INITIALIZED,
    FAILED
  };

  explicit CartesianJogServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CartesianJogServerNode() override = default;

  [[nodiscard]] bool is_kinematics_initialized() const noexcept;
  [[nodiscard]] bool has_kinematics_failed() const noexcept;
  [[nodiscard]] std::string kinematics_error_msg() const;
  [[nodiscard]] bool is_active() const noexcept;
  [[nodiscard]] bool is_arm_busy() const noexcept;

private:
  void declare_all_parameters();
  void load_parameters();
  bool init_kinematics_from_urdf(const std::string & urdf_xml);

  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);
  void arm_busy_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void control_loop();
  void dispatch_command(const std::array<double, 5> & target_joints, double dt);

  // Parameters
  double publish_rate_hz_{50.0};
  double point_dt_s_{0.08};
  double stale_timeout_s_{0.25};
  double lpf_alpha_{0.85};
  std::string controller_type_{"joint_trajectory"};

  std::string base_frame_{"base_footprint"};
  std::string tip_frame_{"gripperframe"};
  double safety_margin_rad_{0.0};
  std::vector<std::string> arm_joint_names_{
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll"
  };

  std::string arm_forward_topic_{"/arm_forward_controller/commands"};
  std::string arm_trajectory_topic_{"/arm_trajectory_controller/joint_trajectory"};
  std::string arm_busy_topic_{"/arm/is_busy"};
  std::string joint_states_topic_{"/joint_states"};
  std::string robot_description_topic_{"/robot_description"};

  // ROS Interfaces
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_description_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_arm_busy_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_forward_commands_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Kinematic & Control Components
  mutable std::mutex kinematics_mutex_;
  KinematicsState kinematics_state_{KinematicsState::WAITING_FOR_URDF};
  std::string kinematics_error_msg_{};
  workspace::KinematicsModel kinematics_model_{};
  std::shared_ptr<workspace::SO101AnalyticalSolver> ik_solver_{nullptr};
  std::unique_ptr<CartesianJogger> jogger_{nullptr};
  JointMotionFilter<5> motion_filter_{0.85};

  // State Tracking
  mutable std::mutex state_mutex_;
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist latest_twist_{};

  std::vector<int> joint_state_indices_{};
  std::array<double, 5> current_physical_joints_{};
  bool have_joint_states_{false};
  bool active_jogging_{false};
  std::atomic<bool> is_arm_busy_{false};
};

}  // namespace lekiwi_motion::arm

#endif  // LEKIWI_MOTION__ARM__CARTESIAN_JOG_SERVER_NODE_HPP_


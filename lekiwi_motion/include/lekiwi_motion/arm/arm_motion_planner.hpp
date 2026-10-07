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
 * @file arm_motion_planner.hpp
 * @brief Unified arm motion planning: gripper models, landmark poses, phase sequencing,
 *        and C2-continuous quintic polynomial trajectory interpolation.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__ARM_MOTION_PLANNER_HPP_
#define LEKIWI_MOTION__ARM_MOTION_PLANNER_HPP_

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace lekiwi_motion
{

/**
 * @enum ClearBinSide
 * @brief Directional designation for captured chess piece graveyard bins.
 */
enum class ClearBinSide
{
  LEFT,
  RIGHT
};

/**
 * @struct GripperConfig
 * @brief Configuration parameters for robot end-effector / gripper bounds.
 */
struct GripperConfig
{
  double open_rad{1.50};
  double closed_rad{0.00};

  [[nodiscard]] double clamp(double position) const noexcept
  {
    const double min_val = std::min(closed_rad, open_rad);
    const double max_val = std::max(closed_rad, open_rad);
    return std::clamp(position, min_val, max_val);
  }
};

/**
 * @struct NamedPosesConfig
 * @brief Landmark joint target configurations loaded from YAML.
 *
 * - home: Standby / neutral waypoint pose (safety transition or calibration fallback).
 * - stow: Dual-purpose compact transit pose during Nav2 and board observation pose.
 * - clear_left: Drop location into left onboard chess piece bin.
 * - clear_right: Drop location into right onboard chess piece bin.
 */
struct NamedPosesConfig
{
  std::vector<double> home{};
  std::vector<double> stow{};
  std::vector<double> clear_left{};
  std::vector<double> clear_right{};

  [[nodiscard]] std::optional<std::vector<double>> get(const std::string & name) const
  {
    const size_t first = name.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return std::nullopt;
    }
    const size_t last = name.find_last_not_of(" \t\n\r");
    std::string key = name.substr(first, last - first + 1);
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (key == "home") return home;
    if (key == "stow") return stow;
    if (key == "clear_left") return clear_left;
    if (key == "clear_right") return clear_right;
    return std::nullopt;
  }

  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> to_map() const
  {
    return {
      {"home", home},
      {"stow", stow},
      {"clear_left", clear_left},
      {"clear_right", clear_right}
    };
  }
};

/**
 * @class ArmMotionPlanner
 * @brief High-cohesion domain planner for robotic arm manipulation:
 *        combines discrete phase waypoint sequencing, gripper/landmark management,
 *        and C2-continuous quintic polynomial trajectory interpolation.
 */
class ArmMotionPlanner
{
public:
  // Dynamic Phase Sequences
  inline static const std::vector<std::string> kPickPhases{
    "APPROACH_PICK", "DESCEND_PICK", "GRASP", "LIFT", "RETRACT_STOW"};
  inline static const std::vector<std::string> kPlacePhases{
    "APPROACH_PLACE", "DESCEND_PLACE", "RELEASE", "RETRACT", "RETRACT_STOW"};
  inline static const std::vector<std::string> kPickAndPlacePhases{
    "APPROACH_PICK", "DESCEND_PICK", "GRASP", "LIFT",
    "APPROACH_PLACE", "DESCEND_PLACE", "RELEASE", "RETRACT"};
  inline static const std::vector<std::string> kClearPhases{
    "APPROACH_PICK", "DESCEND_PICK", "GRASP", "LIFT",
    "DROP_CLEAR", "RELEASE_CLEAR", "RETRACT_CLEAR"};

  explicit ArmMotionPlanner(
    std::vector<std::string> arm_joints = {},
    GripperConfig gripper_cfg = {},
    NamedPosesConfig named_poses_cfg = {},
    double default_rate_hz = 50.0);

  [[nodiscard]] double default_rate_hz() const noexcept { return default_rate_hz_; }
  void set_default_rate_hz(double rate_hz)
  {
    if (rate_hz <= 0.0) {
      throw std::invalid_argument("rate_hz must be positive.");
    }
    default_rate_hz_ = rate_hz;
  }

  [[nodiscard]] const std::vector<std::string> & arm_joints() const noexcept { return arm_joints_; }
  void set_arm_joints(std::vector<std::string> joints) noexcept { arm_joints_ = std::move(joints); }

  [[nodiscard]] const GripperConfig & gripper_config() const noexcept { return gripper_cfg_; }
  void set_gripper_config(GripperConfig cfg) noexcept { gripper_cfg_ = cfg; }
  [[nodiscard]] double gripper_open_rad() const noexcept { return gripper_cfg_.open_rad; }
  [[nodiscard]] double gripper_closed_rad() const noexcept { return gripper_cfg_.closed_rad; }

  [[nodiscard]] const NamedPosesConfig & named_poses() const noexcept { return named_poses_cfg_; }
  void set_named_poses(NamedPosesConfig cfg) noexcept { named_poses_cfg_ = std::move(cfg); }

  /**
   * @brief Clamps a gripper position using the internal gripper bounds.
   */
  [[nodiscard]] double clamp_gripper(double position) const noexcept
  {
    return gripper_cfg_.clamp(position);
  }

  /**
   * @brief Retrieves joint targets for a named landmark pose ("home", "stow", "clear_left", "clear_right").
   */
  [[nodiscard]] std::optional<std::vector<double>> get_named_pose(const std::string & pose_name) const
  {
    return named_poses_cfg_.get(pose_name);
  }

  /**
   * @brief Builds 5-phase joint targets for an atomic Pick operation.
   * Ends with RETRACT_STOW: safely tucked arm holding piece with closed gripper for mobile base transit.
   * @param[in] pick_approach Approach joint angles for pick position (elevated in Cartesian Z).
   * @param[in] pick_descend Descend joint angles for pick position (at piece grasp height).
   * @return Map of phase name to target joint angles vector.
   */
  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_pick_targets(
    const std::vector<double> & pick_approach,
    const std::vector<double> & pick_descend) const;

  /**
   * @brief Builds 5-phase joint targets for an atomic Place operation.
   * Starts with closed gripper, descends to place, releases, retracts in Z, and concludes in RETRACT_STOW with open gripper.
   * @param[in] place_approach Approach joint angles for place position (elevated in Cartesian Z).
   * @param[in] place_descend Descend joint angles for place position (at target placement height).
   * @return Map of phase name to target joint angles vector.
   */
  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_place_targets(
    const std::vector<double> & place_approach,
    const std::vector<double> & place_descend) const;

  /**
   * @brief Builds 8-phase dynamic joint targets for pick-and-place sequence.
   * Directly transits between pick LIFT and place APPROACH_PLACE without mid-transit stow.
   * @param[in] pick_approach Approach joint angles for pick position (elevated in Cartesian Z).
   * @param[in] pick_descend Descend joint angles for pick position (at piece grasp height).
   * @param[in] place_approach Approach joint angles for place position (elevated in Cartesian Z).
   * @param[in] place_descend Descend joint angles for place position (at target placement height).
   * @return Map of phase name to target joint angles vector.
   */
  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_phase_targets(
    const std::vector<double> & pick_approach,
    const std::vector<double> & pick_descend,
    const std::vector<double> & place_approach,
    const std::vector<double> & place_descend) const;

  /**
   * @brief Builds 7-phase joint targets for capture move (Pick from board -> Drop to left/right bin).
   * @param[in] pick_approach Approach joint angles for pick position (elevated in Cartesian Z).
   * @param[in] pick_descend Descend joint angles for pick position (at piece grasp height).
   * @param[in] bin_side Destination graveyard bin (LEFT or RIGHT).
   * @return Map of phase name to target joint angles vector.
   */
  [[nodiscard]] std::unordered_map<std::string, std::vector<double>> build_clear_targets(
    const std::vector<double> & pick_approach,
    const std::vector<double> & pick_descend,
    ClearBinSide bin_side) const;

  /**
   * @brief Calculates the minimum synchronized trajectory duration (seconds) such that no joint
   *        exceeds max_velocity_rad_s under quintic polynomial interpolation.
   * @param[in] start_positions Starting joint angles (rad).
   * @param[in] target_positions Target joint angles (rad).
   * @param[in] max_velocity_rad_s Maximum permissible angular velocity (rad/s) across joints (> 0.0).
   * @param[in] min_duration_sec Minimum clamp duration to avoid division by zero / singularity (default 0.1s).
   * @return Synchronized duration in seconds.
   * @throws std::invalid_argument if max_velocity_rad_s <= 0, min_duration_sec <= 0, or dimensions mismatch.
   */
  [[nodiscard]] double calculate_trajectory_duration(
    const std::vector<double> & start_positions,
    const std::vector<double> & target_positions,
    double max_velocity_rad_s,
    double min_duration_sec = 0.1) const;

  /**
   * @brief Plans a quintic joint trajectory from start positions to target positions bounded by max joint velocity.
   * @param[in] start_positions Starting joint angles (rad), size must match joint count.
   * @param[in] target_positions Target joint angles (rad), size must match joint count.
   * @param[in] max_velocity_rad_s Maximum permissible angular velocity (rad/s) across joints (> 0.0).
   * @param[in] rate_hz Optional sampling frequency in Hz (uses default_rate_hz if unset).
   * @return trajectory_msgs::msg::JointTrajectory message with positions, velocities, and accelerations.
   * @throws std::invalid_argument if max_velocity_rad_s <= 0, rate <= 0, or vector dimensions mismatch.
   */
  [[nodiscard]] trajectory_msgs::msg::JointTrajectory plan_trajectory_at_velocity(
    const std::vector<double> & start_positions,
    const std::vector<double> & target_positions,
    double max_velocity_rad_s,
    std::optional<double> rate_hz = std::nullopt) const;

  /**
   * @brief Plans a quintic joint trajectory from start positions to target positions.
   * @param[in] start_positions Starting joint angles (rad), size must match joint count.
   * @param[in] target_positions Target joint angles (rad), size must match joint count.
   * @param[in] duration_sec Execution duration in seconds (> 0.0).
   * @param[in] rate_hz Optional sampling frequency in Hz (uses default_rate_hz if unset).
   * @return trajectory_msgs::msg::JointTrajectory message with positions, velocities, and accelerations.
   * @throws std::invalid_argument if duration <= 0, rate <= 0, or vector dimensions mismatch.
   */
  [[nodiscard]] trajectory_msgs::msg::JointTrajectory plan_trajectory(
    const std::vector<double> & start_positions,
    const std::vector<double> & target_positions,
    double duration_sec,
    std::optional<double> rate_hz = std::nullopt) const;

private:
  [[nodiscard]] std::vector<double> with_gripper(
    std::vector<double> joints, double gripper_rad) const
  {
    if (!joints.empty()) {
      joints.back() = gripper_cfg_.clamp(gripper_rad);
    }
    return joints;
  }

  [[nodiscard]] std::vector<double> get_stow_pose_internal() const;

  std::vector<std::string> arm_joints_;
  GripperConfig gripper_cfg_{};
  NamedPosesConfig named_poses_cfg_;
  double default_rate_hz_{50.0};
};

}  // namespace lekiwi_motion

#endif  // LEKIWI_MOTION__ARM_MOTION_PLANNER_HPP_


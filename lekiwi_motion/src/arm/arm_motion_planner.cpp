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

#include "lekiwi_motion/arm/arm_motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace lekiwi_motion
{

ArmMotionPlanner::ArmMotionPlanner(
  std::vector<std::string> arm_joints,
  GripperConfig gripper_cfg,
  NamedPosesConfig named_poses_cfg,
  double default_rate_hz)
: arm_joints_(std::move(arm_joints)),
  gripper_cfg_(gripper_cfg),
  named_poses_cfg_(std::move(named_poses_cfg)),
  default_rate_hz_(default_rate_hz)
{
  if (default_rate_hz_ <= 0.0) {
    throw std::invalid_argument("default_rate_hz must be positive.");
  }
}

std::vector<double> ArmMotionPlanner::get_stow_pose_internal() const
{
  auto opt_stow = named_poses_cfg_.get("stow");
  if (opt_stow.has_value() && !opt_stow->empty()) {
    return *opt_stow;
  }
  // Safe default fallback matching system orchestrator config for SO-101 6-DoF arm
  return {0.0, -1.57, 1.57, 0.75, 0.0, gripper_closed_rad()};
}

std::unordered_map<std::string, std::vector<double>> ArmMotionPlanner::build_pick_targets(
  const std::vector<double> & pick_approach,
  const std::vector<double> & pick_descend) const
{
  const double open_g = gripper_open_rad();
  const double closed_g = gripper_closed_rad();

  return {
    {"APPROACH_PICK", with_gripper(pick_approach, open_g)},
    {"DESCEND_PICK",  with_gripper(pick_descend, open_g)},
    {"GRASP",         with_gripper(pick_descend, closed_g)},
    {"LIFT",          with_gripper(pick_approach, closed_g)},
    {"RETRACT_STOW",  with_gripper(get_stow_pose_internal(), closed_g)},
  };
}

std::unordered_map<std::string, std::vector<double>> ArmMotionPlanner::build_place_targets(
  const std::vector<double> & place_approach,
  const std::vector<double> & place_descend) const
{
  const double open_g = gripper_open_rad();
  const double closed_g = gripper_closed_rad();

  return {
    {"APPROACH_PLACE", with_gripper(place_approach, closed_g)},
    {"DESCEND_PLACE",  with_gripper(place_descend, closed_g)},
    {"RELEASE",        with_gripper(place_descend, open_g)},
    {"RETRACT",        with_gripper(place_approach, open_g)},
    {"RETRACT_STOW",   with_gripper(get_stow_pose_internal(), open_g)},
  };
}

std::unordered_map<std::string, std::vector<double>> ArmMotionPlanner::build_phase_targets(
  const std::vector<double> & pick_approach,
  const std::vector<double> & pick_descend,
  const std::vector<double> & place_approach,
  const std::vector<double> & place_descend) const
{
  const double open_g = gripper_open_rad();
  const double closed_g = gripper_closed_rad();

  return {
    {"APPROACH_PICK",  with_gripper(pick_approach, open_g)},
    {"DESCEND_PICK",   with_gripper(pick_descend, open_g)},
    {"GRASP",          with_gripper(pick_descend, closed_g)},
    {"LIFT",           with_gripper(pick_approach, closed_g)},
    {"APPROACH_PLACE", with_gripper(place_approach, closed_g)},
    {"DESCEND_PLACE",  with_gripper(place_descend, closed_g)},
    {"RELEASE",        with_gripper(place_descend, open_g)},
    {"RETRACT",        with_gripper(get_stow_pose_internal(), open_g)},
  };
}

std::unordered_map<std::string, std::vector<double>> ArmMotionPlanner::build_clear_targets(
  const std::vector<double> & pick_approach,
  const std::vector<double> & pick_descend,
  ClearBinSide bin_side) const
{
  const double open_g = gripper_open_rad();
  const double closed_g = gripper_closed_rad();

  const std::string bin_pose_name = (bin_side == ClearBinSide::LEFT) ? "clear_left" : "clear_right";
  const std::vector<double> bin_pose = named_poses_cfg_.get(bin_pose_name).value_or(std::vector<double>{});

  return {
    {"APPROACH_PICK",  with_gripper(pick_approach, open_g)},
    {"DESCEND_PICK",   with_gripper(pick_descend, open_g)},
    {"GRASP",          with_gripper(pick_descend, closed_g)},
    {"LIFT",           with_gripper(pick_approach, closed_g)},
    {"DROP_CLEAR",     with_gripper(bin_pose, closed_g)},
    {"RELEASE_CLEAR",  with_gripper(bin_pose, open_g)},
    {"RETRACT_CLEAR",  with_gripper(get_stow_pose_internal(), open_g)},
  };
}

double ArmMotionPlanner::calculate_trajectory_duration(
  const std::vector<double> & start_positions,
  const std::vector<double> & target_positions,
  double max_velocity_rad_s,
  double min_duration_sec) const
{
  if (max_velocity_rad_s <= 0.0) {
    throw std::invalid_argument(
      "max_velocity_rad_s must be positive, got " + std::to_string(max_velocity_rad_s) + " rad/s.");
  }
  if (min_duration_sec <= 0.0) {
    throw std::invalid_argument(
      "min_duration_sec must be positive, got " + std::to_string(min_duration_sec) + "s.");
  }

  const size_t num_joints = start_positions.size();
  if (num_joints == 0) {
    throw std::invalid_argument("Joint positions vector cannot be empty.");
  }
  if (target_positions.size() != num_joints) {
    throw std::invalid_argument(
      "Dimension mismatch: start_positions has " + std::to_string(num_joints) +
      " elements while target_positions has " + std::to_string(target_positions.size()) + ".");
  }
  if (!arm_joints_.empty() && arm_joints_.size() != num_joints) {
    throw std::invalid_argument(
      "Dimension mismatch: arm_joints has " + std::to_string(arm_joints_.size()) +
      " elements while positions have " + std::to_string(num_joints) + ".");
  }

  double required_duration = min_duration_sec;
  for (size_t i = 0; i < num_joints; ++i) {
    const double delta_q = std::abs(target_positions[i] - start_positions[i]);
    // For quintic polynomial with v0 = vf = a0 = af = 0, peak velocity is 1.875 * (delta_q / T).
    // Therefore T >= 1.875 * delta_q / max_velocity_rad_s.
    const double joint_duration = (1.875 * delta_q) / max_velocity_rad_s;
    required_duration = std::max(required_duration, joint_duration);
  }

  return required_duration;
}

trajectory_msgs::msg::JointTrajectory ArmMotionPlanner::plan_trajectory_at_velocity(
  const std::vector<double> & start_positions,
  const std::vector<double> & target_positions,
  double max_velocity_rad_s,
  std::optional<double> rate_hz) const
{
  const double duration_sec = calculate_trajectory_duration(
    start_positions, target_positions, max_velocity_rad_s);
  return plan_trajectory(start_positions, target_positions, duration_sec, rate_hz);
}

trajectory_msgs::msg::JointTrajectory ArmMotionPlanner::plan_trajectory(
  const std::vector<double> & start_positions,
  const std::vector<double> & target_positions,
  double duration_sec,
  std::optional<double> rate_hz) const
{
  if (duration_sec <= 0.0) {
    throw std::invalid_argument(
      "Trajectory duration must be positive, got " + std::to_string(duration_sec) + "s.");
  }

  const double effective_rate_hz = rate_hz.value_or(default_rate_hz_);
  if (effective_rate_hz <= 0.0) {
    throw std::invalid_argument(
      "Sampling rate must be positive, got " + std::to_string(effective_rate_hz) + "Hz.");
  }

  const size_t num_joints = start_positions.size();
  if (num_joints == 0) {
    throw std::invalid_argument("Joint positions vector cannot be empty.");
  }
  if (target_positions.size() != num_joints) {
    throw std::invalid_argument(
      "Dimension mismatch: start_positions has " + std::to_string(num_joints) +
      " elements while target_positions has " + std::to_string(target_positions.size()) + ".");
  }
  if (!arm_joints_.empty() && arm_joints_.size() != num_joints) {
    throw std::invalid_argument(
      "Dimension mismatch: arm_joints has " + std::to_string(arm_joints_.size()) +
      " elements while positions have " + std::to_string(num_joints) + ".");
  }

  const double dt = 1.0 / effective_rate_hz;
  const size_t num_points = std::max<size_t>(
    2UL,
    static_cast<size_t>(std::ceil(duration_sec / dt)) + 1UL);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = arm_joints_;
  traj_msg.points.reserve(num_points);

  std::vector<double> delta_q(num_joints, 0.0);
  for (size_t i = 0; i < num_joints; ++i) {
    delta_q[i] = target_positions[i] - start_positions[i];
  }

  const double duration_inv = 1.0 / duration_sec;
  const double duration_sq_inv = duration_inv * duration_inv;

  for (size_t step = 0; step < num_points; ++step) {
    const double t = std::min(static_cast<double>(step) * dt, duration_sec);
    const double u = t * duration_inv;

    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;

    const double s = 6.0 * u5 - 15.0 * u4 + 10.0 * u3;
    const double ds_du = 30.0 * u4 - 60.0 * u3 + 30.0 * u2;
    const double d2s_du2 = 120.0 * u3 - 180.0 * u2 + 60.0 * u;

    const double ds_dt = ds_du * duration_inv;
    const double d2s_dt2 = d2s_du2 * duration_sq_inv;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(num_joints);
    point.velocities.resize(num_joints);
    point.accelerations.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i) {
      point.positions[i] = start_positions[i] + s * delta_q[i];
      point.velocities[i] = ds_dt * delta_q[i];
      point.accelerations[i] = d2s_dt2 * delta_q[i];
    }

    const auto sec = static_cast<int32_t>(t);
    const auto nanosec = static_cast<uint32_t>((t - static_cast<double>(sec)) * 1e9);
    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = nanosec;

    traj_msg.points.push_back(std::move(point));
  }

  // Exact boundary conditions at the last point
  auto & last_point = traj_msg.points.back();
  last_point.positions = target_positions;
  std::fill(last_point.velocities.begin(), last_point.velocities.end(), 0.0);
  std::fill(last_point.accelerations.begin(), last_point.accelerations.end(), 0.0);

  const auto final_sec = static_cast<int32_t>(duration_sec);
  const auto final_nanosec = static_cast<uint32_t>(
    (duration_sec - static_cast<double>(final_sec)) * 1e9);
  last_point.time_from_start.sec = final_sec;
  last_point.time_from_start.nanosec = final_nanosec;

  return traj_msg;
}

}  // namespace lekiwi_motion

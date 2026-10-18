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
 * @file cartesian_jogger.cpp
 * @brief Implementation of CartesianJogger for tracking Cartesian pose and computing analytical IK.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/arm/cartesian_jogger.hpp"

#include <stdexcept>
#include <utility>

namespace lekiwi_motion::arm
{

CartesianJogger::CartesianJogger(
  std::shared_ptr<const workspace::SO101AnalyticalSolver> ik_solver,
  workspace::KinematicsModel kinematics_model)
: ik_solver_(std::move(ik_solver)),
  kinematics_model_(std::move(kinematics_model))
{
  if (!ik_solver_) {
    throw std::invalid_argument("CartesianJogger requires a non-null SO101AnalyticalSolver.");
  }
}

void CartesianJogger::sync_from_fk(const std::array<double, 5> & arm_joints)
{
  const Eigen::Isometry3d fk = workspace::forward_kinematics(kinematics_model_, arm_joints);
  target_pos_.x = fk.translation().x();
  target_pos_.y = fk.translation().y();
  target_pos_.z = fk.translation().z();

  const auto [computed_pitch, computed_roll] =
    ik_solver_->compute_tool_angles_from_joints(arm_joints);
  target_pitch_ = computed_pitch;
  target_roll_ = computed_roll;

  current_joints_ = arm_joints;
  initialized_ = true;
}

JogResult CartesianJogger::process_jog(const CartesianDelta & delta)
{
  JogResult result;

  if (delta.is_near_zero()) {
    result.success = true;
    result.joints = current_joints_;
    return result;
  }

  const double cand_x = target_pos_.x + delta.dx;
  const double cand_y = target_pos_.y + delta.dy;
  const double cand_z = target_pos_.z + delta.dz;
  const double cand_pitch = target_pitch_ + delta.dpitch;
  const double cand_roll = target_roll_ + delta.droll;

  const auto ik_res = ik_solver_->solve(cand_x, cand_y, cand_z, cand_pitch, cand_roll);

  if (ik_res.success) {
    target_pos_.x = cand_x;
    target_pos_.y = cand_y;
    target_pos_.z = cand_z;
    target_pitch_ = cand_pitch;
    target_roll_ = cand_roll;

    current_joints_ = ik_res.joints;

    result.success = true;
    result.joints = ik_res.joints;
  } else {
    result.success = false;
    result.joints = current_joints_;
    result.reason = ik_res.reason;
  }

  return result;
}

}  // namespace lekiwi_motion::arm


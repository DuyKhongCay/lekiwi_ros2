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
 * @file cartesian_jogger.hpp
 * @brief Pure POCO domain controller for Cartesian arm jogging, FK synchronization, and IK evaluation.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__ARM__CARTESIAN_JOGGER_HPP_
#define LEKIWI_MOTION__ARM__CARTESIAN_JOGGER_HPP_

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace lekiwi_motion::arm
{

/**
 * @struct CartesianDelta
 * @brief Incremental translation and orientation displacements over a control step dt.
 */
struct CartesianDelta
{
  double dx{0.0};
  double dy{0.0};
  double dz{0.0};
  double dpitch{0.0};
  double droll{0.0};

  /**
   * @brief Evaluates whether all delta components are within negligible noise threshold.
   */
  [[nodiscard]] bool is_near_zero(double tol = 1e-7) const noexcept
  {
    return std::hypot(dx, dy, dz) < tol &&
           std::abs(dpitch) < tol &&
           std::abs(droll) < tol;
  }
};

/**
 * @struct JogResult
 * @brief Result of a candidate Cartesian jog step.
 */
struct JogResult
{
  bool success{false};
  std::array<double, 5> joints{};
  std::string reason;
};

/**
 * @class CartesianJogger
 * @brief POCO Domain Controller tracking TCP pose in Cartesian coordinates and solving analytical IK.
 */
class CartesianJogger
{
public:
  /**
   * @brief Constructs CartesianJogger with calibrated analytical IK solver and kinematics model.
   * @param[in] ik_solver Shared pointer to calibrated SO101AnalyticalSolver.
   * @param[in] kinematics_model Kinematic model chain.
   */
  explicit CartesianJogger(
    std::shared_ptr<const workspace::SO101AnalyticalSolver> ik_solver,
    workspace::KinematicsModel kinematics_model);

  /**
   * @brief Synchronizes Cartesian target pose directly from physical joint configuration using FK.
   * Ensures bumpless transfer when initiating Cartesian control.
   * @param[in] arm_joints Current 5-DOF joint angles (rad).
   */
  void sync_from_fk(const std::array<double, 5> & arm_joints);

  /**
   * @brief Evaluates candidate step displacement and solves inverse kinematics.
   * If candidate pose is feasible, internal target pose and joints are updated.
   * If infeasible, target pose is retained without drift.
   * @param[in] delta Cartesian increment.
   * @return JogResult with validity flag, joint solutions, and diagnostics.
   */
  JogResult process_jog(const CartesianDelta & delta);

  /**
   * @brief Returns current target 3D translation.
   */
  [[nodiscard]] workspace::Point3D current_position() const noexcept { return target_pos_; }

  /**
   * @brief Returns current target orientation pair [pitch, roll] in radians.
   */
  [[nodiscard]] std::pair<double, double> current_orientation() const noexcept
  {
    return {target_pitch_, target_roll_};
  }

  /**
   * @brief Returns currently computed 5 active arm joint positions.
   */
  [[nodiscard]] const std::array<double, 5> & current_joints() const noexcept
  {
    return current_joints_;
  }

  /**
   * @brief Returns true if jogger has been synchronized at least once via sync_from_fk().
   */
  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

private:
  std::shared_ptr<const workspace::SO101AnalyticalSolver> ik_solver_;
  workspace::KinematicsModel kinematics_model_;

  workspace::Point3D target_pos_{0.20, 0.0, 0.15};
  double target_pitch_{-1.5708};
  double target_roll_{0.0};
  std::array<double, 5> current_joints_{};
  bool initialized_{false};
};

}  // namespace lekiwi_motion::arm

#endif  // LEKIWI_MOTION__ARM__CARTESIAN_JOGGER_HPP_


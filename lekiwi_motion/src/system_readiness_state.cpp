// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/system_readiness_state.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace lekiwi_motion
{

  SystemReadinessEvaluator::SystemReadinessEvaluator(const EvaluatorConfig &config)
      : config_(config)
  {
  }

  void SystemReadinessEvaluator::record_global_ekf_convergence(bool converged) noexcept
  {
    if (converged)
    {
      global_ekf_seeded_ = true;
    }
  }

  void SystemReadinessEvaluator::reset_global_ekf_seed() noexcept
  {
    global_ekf_seeded_ = false;
  }

  SystemReadinessReport SystemReadinessEvaluator::evaluate(
      double now_sec,
      const LocalOdomSnapshot &local_odom,
      const GlobalOdomSnapshot &global_odom,
      const JointsSnapshot &joints,
      const TfChainSnapshot &tf_chains)
  {
    SystemReadinessReport report;

    // 1. Local Odometry & Speed Evaluation
    bool local_odom_fresh = false;
    if (local_odom.valid)
    {
      local_odom_fresh = policy::fresh(
          now_sec, local_odom.stamp_sec, config_.nav_odom_max_age_sec);
      report.current_speed = local_odom.speed();
      report.is_stationary = local_odom_fresh &&
                             policy::stationary(local_odom.vx, local_odom.vy, local_odom.wz,
                                                config_.max_stop_velocity,
                                                config_.max_stop_angular_vel);
    }

    // 2. Global EKF Evaluation
    bool global_ekf_converged = false;
    if (global_odom.valid)
    {
      const bool global_ekf_fresh = policy::fresh(
          now_sec, global_odom.stamp_sec, config_.max_transform_age_sec);
      report.current_pos_variance = global_odom.pos_variance;
      global_ekf_converged =
          global_ekf_fresh &&
          (global_odom.pos_variance <= config_.max_pos_variance) &&
          (global_odom.yaw_variance <= config_.max_yaw_variance);

      if (global_ekf_converged)
      {
        global_ekf_seeded_ = true;
      }
    }

    // 3. Joints Completeness & Freshness
    bool joints_complete = false;
    if (joints.valid)
    {
      const bool joints_fresh = policy::fresh(
          now_sec, joints.stamp_sec, config_.max_transform_age_sec);
      joints_complete = joints_fresh;
      for (const auto &required : config_.required_arm_joints)
      {
        if (joints.joint_names.find(required) == joints.joint_names.end())
        {
          joints_complete = false;
          break;
        }
      }
    }

    // ================= TẦNG 1: NAVIGATION READINESS =================
    // Đòi hỏi: Local odom tươi, TF odom->base tươi, và Global EKF đã từng khóa vị trí ít nhất 1 lần.
    const bool seeded_ok = !config_.require_global_ekf_seed || global_ekf_seeded_;
    report.nav_ready = local_odom_fresh &&
                       tf_chains.odom_to_base_fresh &&
                       seeded_ok;

    // ================= TẦNG 2: GRASP READINESS =================
    // Đòi hỏi: Xe đứng yên hoàn toàn, đầy đủ khớp tay, Global EKF đang hội tụ và chuỗi TF bàn cờ + tay kẹp hoàn hảo.
    report.grasp_ready = report.is_stationary &&
                         joints_complete &&
                         global_ekf_converged &&
                         tf_chains.base_to_gripper_fresh &&
                         tf_chains.map_to_board_fresh;

    // Self-documenting blocker reason if grasp is not ready
    if (!report.grasp_ready)
    {
      std::ostringstream ss;
      if (!report.is_stationary)
      {
        ss << "Moving (v=" << std::round(report.current_speed * 100.0) / 100.0 << "m/s)";
      }
      else if (!joints_complete)
      {
        ss << "Arm joints incomplete or stale";
      }
      else if (!global_ekf_converged)
      {
        ss << "Global EKF unconverged or stale (var=" << report.current_pos_variance << ")";
      }
      else if (!tf_chains.map_to_board_fresh)
      {
        ss << "Chessboard tag TF not fresh";
      }
      else if (!tf_chains.base_to_gripper_fresh)
      {
        ss << "Gripper TF not fresh";
      }
      else
      {
        ss << "Unknown grasp inhibition";
      }
      report.grasp_blocker_reason = ss.str();
    }
    else
    {
      report.grasp_blocker_reason = "Ready";
    }

    return report;
  }

} // namespace lekiwi_motion

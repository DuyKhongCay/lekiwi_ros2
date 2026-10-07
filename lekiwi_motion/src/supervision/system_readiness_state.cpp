/**
 * @file system_readiness_state.cpp
 * @brief Implementation of domain readiness evaluator for LeKiwi mobile base and arm.
 *
 * Evaluates odometry velocity limits, covariance matrices, joint enumeration,
 * and transform freshness to compute discrete readiness flags.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_motion/supervision/system_readiness_state.hpp"

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
    stationary_latched_ = false;
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

    // Invalidate stationary convergence latch if chassis is in motion or local odom is stale
    if (!report.is_stationary)
    {
      stationary_latched_ = false;
    }

    // 2. Global EKF Evaluation
    bool global_ekf_converged = false;
    bool global_ekf_fresh = false;
    if (global_odom.valid)
    {
      global_ekf_fresh = policy::fresh(
          now_sec, global_odom.stamp_sec, config_.max_transform_age_sec);
      report.current_pos_variance = global_odom.pos_variance;
      global_ekf_converged =
          global_ekf_fresh &&
          (global_odom.pos_variance <= config_.max_pos_variance) &&
          (global_odom.yaw_variance <= config_.max_yaw_variance);

      if (global_ekf_converged)
      {
        global_ekf_seeded_ = true;
        if (report.is_stationary)
        {
          stationary_latched_ = true;
        }
      }
    }

    // Categorize root cause of EKF divergence or covariance elevation
    if (global_ekf_converged)
    {
      report.drift_type = DriftType::NONE;
    }
    else if (!report.is_stationary)
    {
      // Chassis in motion: kinematic slippage or transient visual track loss
      report.drift_type = DriftType::MOVING;
    }
    else if (!global_ekf_seeded_)
    {
      // Stationary chassis, but global EKF has never converged since startup
      report.drift_type = DriftType::NOT_SEEDED;
    }
    else
    {
      // Stationary chassis, but AprilTag fiducial lost or filter variance drifted
      report.drift_type = DriftType::STATIONARY_NO_TAG;
    }

    report.stationary_latched = stationary_latched_;

    // 3. Joints Completeness & Freshness
    bool joints_complete = false;
    if (joints.valid)
    {
      const bool joints_fresh = policy::fresh(
          now_sec, joints.stamp_sec, config_.max_transform_age_sec);
      joints_complete = joints_fresh;
      for (const auto &required : config_.arm_joints)
      {
        if (joints.joint_names.find(required) == joints.joint_names.end())
        {
          joints_complete = false;
          break;
        }
      }
    }

    // ================= TIER 1: NAVIGATION READINESS =================
    // Requires: fresh local odom, fresh odom->base TF, and historical EKF initialization
    const bool seeded_ok = !config_.require_global_ekf_seed || global_ekf_seeded_;
    report.nav_ready = local_odom_fresh &&
                       tf_chains.odom_to_base_fresh &&
                       seeded_ok;

    // ================= TIER 2: GRASP READINESS =================
    // Permit latched EKF precision if chassis remained stationary continuously since last convergence
    const bool ekf_precision_ready = global_ekf_converged ||
                                     (report.is_stationary && stationary_latched_);

    report.grasp_ready = report.is_stationary &&
                         joints_complete &&
                         ekf_precision_ready &&
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
      else if (!ekf_precision_ready)
      {
        if (report.drift_type == DriftType::NOT_SEEDED)
        {
          ss << "Global EKF unconverged: Not seeded";
        }
        else if (report.drift_type == DriftType::STATIONARY_NO_TAG)
        {
          if (!global_ekf_fresh)
          {
            ss << "Global EKF unconverged: Stationary without AprilTag (tag stale/missing, var="
               << report.current_pos_variance << ")";
          }
          else
          {
            ss << "Global EKF unconverged: Stationary without AprilTag (high variance var="
               << report.current_pos_variance << ")";
          }
        }
        else
        {
          ss << "Global EKF unconverged or stale (var=" << report.current_pos_variance << ")";
        }
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

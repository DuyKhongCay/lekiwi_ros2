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
    was_stationary_ = false;
    non_stationary_since_sec_ = -1.0;
  }

  bool SystemReadinessEvaluator::evaluate_stationary_state(
      double now_sec,
      const LocalOdomSnapshot &local_odom,
      bool local_odom_fresh,
      double &out_speed) noexcept
  {
    out_speed = local_odom.valid ? local_odom.speed() : 0.0;
    if (!local_odom.valid || !local_odom_fresh)
    {
      non_stationary_since_sec_ = -1.0;
      was_stationary_ = false;
      return false;
    }

    const bool raw_stationary = policy::stationary(
        local_odom.vx, local_odom.vy, local_odom.wz,
        config_.max_stop_velocity, config_.max_stop_angular_vel);

    if (raw_stationary)
    {
      non_stationary_since_sec_ = -1.0;
      was_stationary_ = true;
      return true;
    }

    // Handle unlatch debounce window
    if (was_stationary_ && config_.stationary_unlatch_debounce_sec > 0.0)
    {
      if (non_stationary_since_sec_ < 0.0)
      {
        non_stationary_since_sec_ = now_sec;
      }
      const bool within_debounce =
          (now_sec - non_stationary_since_sec_) < config_.stationary_unlatch_debounce_sec;
      was_stationary_ = within_debounce;
      return within_debounce;
    }

    non_stationary_since_sec_ = -1.0;
    was_stationary_ = false;
    return false;
  }

  void SystemReadinessEvaluator::update_stationary_latch(bool is_stationary) noexcept
  {
    if (!is_stationary)
    {
      stationary_latched_ = false;
    }
  }

  bool SystemReadinessEvaluator::evaluate_ekf_convergence(
      double now_sec,
      const GlobalOdomSnapshot &global_odom,
      bool is_stationary,
      double &out_pos_variance,
      bool &out_ekf_fresh) noexcept
  {
    if (!global_odom.valid)
    {
      out_pos_variance = std::numeric_limits<double>::infinity();
      out_ekf_fresh = false;
      return false;
    }

    out_ekf_fresh = policy::fresh(now_sec, global_odom.stamp_sec, config_.max_transform_age_sec);
    out_pos_variance = global_odom.pos_variance;

    const bool converged = out_ekf_fresh &&
        (global_odom.pos_variance <= config_.max_pos_variance) &&
        (global_odom.yaw_variance <= config_.max_yaw_variance);

    if (converged)
    {
      global_ekf_seeded_ = true;
      if (is_stationary)
      {
        stationary_latched_ = true;
      }
    }

    return converged;
  }

  DriftType SystemReadinessEvaluator::classify_drift(
      bool global_ekf_converged,
      bool is_stationary) const noexcept
  {
    if (global_ekf_converged)
    {
      return DriftType::NONE;
    }
    if (!is_stationary)
    {
      return DriftType::MOVING;
    }
    if (!global_ekf_seeded_)
    {
      return DriftType::NOT_SEEDED;
    }
    return DriftType::STATIONARY_NO_TAG;
  }

  bool SystemReadinessEvaluator::validate_joints(
      double now_sec,
      const JointsSnapshot &joints) const noexcept
  {
    if (!joints.valid)
    {
      return false;
    }

    if (!policy::fresh(now_sec, joints.stamp_sec, config_.max_transform_age_sec))
    {
      return false;
    }

    return std::all_of(
        config_.arm_joints.begin(), config_.arm_joints.end(),
        [&joints](const std::string &name)
        {
          return joints.joint_names.find(name) != joints.joint_names.end();
        });
  }

  std::string SystemReadinessEvaluator::build_nav_blocker_reason(
      bool nav2_active,
      bool local_odom_fresh,
      bool odom_to_base_fresh,
      bool seeded_ok)
  {
    if (!nav2_active)
    {
      return "Nav2 lifecycle is not active";
    }
    if (!local_odom_fresh)
    {
      return "Local odometry stale or missing";
    }
    if (!odom_to_base_fresh)
    {
      return "TF odom -> base_footprint stale or missing";
    }
    if (!seeded_ok)
    {
      return "Global EKF unconverged: Not seeded";
    }
    return "Navigation inhibited";
  }

  std::string SystemReadinessEvaluator::build_grasp_blocker_reason(
      bool is_stationary,
      double speed,
      bool joints_complete,
      bool ekf_precision_ready,
      DriftType drift_type,
      bool global_ekf_fresh,
      double pos_variance,
      bool map_to_board_fresh,
      bool base_to_gripper_fresh)
  {
    std::ostringstream ss;
    if (!is_stationary)
    {
      ss << "Moving (v=" << std::round(speed * 100.0) / 100.0 << "m/s)";
      return ss.str();
    }
    if (!joints_complete)
    {
      return "Arm joints incomplete or stale";
    }
    if (!ekf_precision_ready)
    {
      if (drift_type == DriftType::NOT_SEEDED)
      {
        return "Global EKF unconverged: Not seeded";
      }
      if (drift_type == DriftType::STATIONARY_NO_TAG)
      {
        if (!global_ekf_fresh)
        {
          ss << "Global EKF unconverged: Stationary without AprilTag (tag stale/missing, var="
             << pos_variance << ")";
        }
        else
        {
          ss << "Global EKF unconverged: Stationary without AprilTag (high variance var="
             << pos_variance << ")";
        }
        return ss.str();
      }
      ss << "Global EKF unconverged or stale (var=" << pos_variance << ")";
      return ss.str();
    }
    if (!map_to_board_fresh)
    {
      return "Chessboard tag TF not fresh";
    }
    if (!base_to_gripper_fresh)
    {
      return "Gripper TF not fresh";
    }
    return "Unknown grasp inhibition";
  }

  SystemReadinessReport SystemReadinessEvaluator::evaluate(
      double now_sec,
      const LocalOdomSnapshot &local_odom,
      const GlobalOdomSnapshot &global_odom,
      const JointsSnapshot &joints,
      const TfChainSnapshot &tf_chains,
      const Nav2LifecycleSnapshot &nav2_lifecycle)
  {
    SystemReadinessReport report;

    // 1. Local Odometry & Speed Evaluation
    const bool local_odom_fresh = policy::fresh(
        now_sec, local_odom.stamp_sec, config_.nav_odom_max_age_sec);
    report.is_stationary = evaluate_stationary_state(
        now_sec, local_odom, local_odom_fresh, report.current_speed);
    update_stationary_latch(report.is_stationary);

    // 2. Global EKF Evaluation
    bool global_ekf_fresh = false;
    const bool global_ekf_converged = evaluate_ekf_convergence(
        now_sec, global_odom, report.is_stationary,
        report.current_pos_variance, global_ekf_fresh);
    report.drift_type = classify_drift(global_ekf_converged, report.is_stationary);
    report.stationary_latched = stationary_latched_;

    // 3. Joints Completeness & Freshness
    const bool joints_complete = validate_joints(now_sec, joints);

    // 4. Tier 1: Navigation Readiness (ekf_converged & nav2_lifecycle_active)
    const bool seeded_ok = !config_.require_global_ekf_seed || global_ekf_seeded_;
    const bool ekf_converged = local_odom_fresh && tf_chains.odom_to_base_fresh && seeded_ok;
    report.nav2_lifecycle_active = nav2_lifecycle.active;
    report.nav_ready = ekf_converged && nav2_lifecycle.active;
    report.nav_blocker_reason = report.nav_ready ? "Ready" :
        build_nav_blocker_reason(
            nav2_lifecycle.active, local_odom_fresh, tf_chains.odom_to_base_fresh, seeded_ok);

    // 5. Tier 2: Grasp Precision Readiness
    const bool ekf_precision_ready = global_ekf_converged || (report.is_stationary && stationary_latched_);
    report.grasp_ready = report.is_stationary && joints_complete && ekf_precision_ready &&
                         tf_chains.base_to_gripper_fresh && tf_chains.map_to_board_fresh;
    report.grasp_blocker_reason = report.grasp_ready ? "Ready" :
        build_grasp_blocker_reason(
            report.is_stationary, report.current_speed, joints_complete,
            ekf_precision_ready, report.drift_type, global_ekf_fresh,
            report.current_pos_variance, tf_chains.map_to_board_fresh,
            tf_chains.base_to_gripper_fresh);

    return report;
  }

} // namespace lekiwi_motion

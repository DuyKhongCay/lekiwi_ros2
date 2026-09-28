// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_MOTION__SYSTEM_READINESS_STATE_HPP_
#define LEKIWI_MOTION__SYSTEM_READINESS_STATE_HPP_

#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace lekiwi_motion
{

  namespace policy
  {
    inline bool fresh(double now, double stamp, double max_age)
    {
      return std::isfinite(now) && std::isfinite(stamp) && stamp > 0.0 &&
             now >= stamp && (now - stamp) <= max_age;
    }

    inline bool stationary(double vx, double vy, double wz, double linear_limit, double angular_limit)
    {
      return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
             std::hypot(vx, vy) <= linear_limit && std::abs(wz) <= angular_limit;
    }

    inline bool converged(double var_x, double var_y, double var_yaw, double max_pos, double max_yaw)
    {
      return std::isfinite(var_x) && std::isfinite(var_y) && std::isfinite(var_yaw) &&
             var_x >= 0.0 && var_y >= 0.0 && var_yaw >= 0.0 &&
             (var_x + var_y) <= max_pos && var_yaw <= max_yaw;
    }
  } // namespace policy

  struct LocalOdomSnapshot
  {
    bool valid{false};
    double stamp_sec{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};

    [[nodiscard]] double speed() const noexcept
    {
      return std::hypot(vx, vy);
    }
  };

  struct GlobalOdomSnapshot
  {
    bool valid{false};
    double stamp_sec{0.0};
    double pos_variance{std::numeric_limits<double>::infinity()};
    double yaw_variance{std::numeric_limits<double>::infinity()};
  };

  struct JointsSnapshot
  {
    bool valid{false};
    double stamp_sec{0.0};
    std::set<std::string> joint_names;
  };

  struct TfChainSnapshot
  {
    bool odom_to_base_fresh{false};
    bool base_to_gripper_fresh{false};
    bool map_to_board_fresh{false};
  };

  struct EvaluatorConfig
  {
    double max_pos_variance{0.0012};
    double max_yaw_variance{0.0030};
    double max_transform_age_sec{0.30};
    double nav_odom_max_age_sec{0.50};
    double max_stop_velocity{0.03};
    double max_stop_angular_vel{0.08};
    bool require_global_ekf_seed{true};
    std::vector<std::string> required_arm_joints{
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"};
  };

  struct SystemReadinessReport
  {
    bool nav_ready{false};
    bool grasp_ready{false};
    bool is_stationary{false};
    double current_speed{0.0};
    double current_pos_variance{std::numeric_limits<double>::infinity()};
    std::string grasp_blocker_reason;
  };

  class SystemReadinessEvaluator
  {
  public:
    explicit SystemReadinessEvaluator(const EvaluatorConfig &config = EvaluatorConfig());

    [[nodiscard]] SystemReadinessReport evaluate(
        double now_sec,
        const LocalOdomSnapshot &local_odom,
        const GlobalOdomSnapshot &global_odom,
        const JointsSnapshot &joints,
        const TfChainSnapshot &tf_chains);

    void record_global_ekf_convergence(bool converged) noexcept;
    void reset_global_ekf_seed() noexcept;

    [[nodiscard]] bool is_global_ekf_seeded() const noexcept { return global_ekf_seeded_; }
    [[nodiscard]] const EvaluatorConfig &config() const noexcept { return config_; }

  private:
    EvaluatorConfig config_;
    bool global_ekf_seeded_{false};
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__SYSTEM_READINESS_STATE_HPP_

/**
 * @file kinematic_remapper.hpp
 * @brief Kinematic Remapping Engine for Leader-Follower Teleoperation.
 *
 * Implements 4 kinematic remapping strategies:
 * - Direct: 1:1 SI Radians pass-through with follower safety clamping.
 * - Min-Max: Linear normalization between leader and follower ranges (e.g. gripper).
 * - Centered-Scale: Home 0.0 rad anchor with independent positive and negative scaling.
 * - Scale-Offset: Generalized linear transformation (k * x + b).
 *
 * Pure C++ math library, independent of ROS 2 middleware and hardware drivers.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace teleop_zhongli_servo_hw
{

  enum class RemapMode
  {
    DIRECT,
    MIN_MAX,
    CENTERED_SCALE,
    SCALE_OFFSET
  };

  struct JointRemapConfig
  {
    std::string joint_name;
    RemapMode mode{RemapMode::DIRECT};
    double leader_min{-M_PI};
    double leader_max{M_PI};
    double follower_min{-M_PI};
    double follower_max{M_PI};
    double scale{1.0};
    double offset{0.0};

    // Pre-computed constants for zero-cost runtime execution
    double k_gain{1.0};
    double b_bias{0.0};
    double pos_scale{1.0};
    double neg_scale{1.0};
  };

  class KinematicRemapper
  {
  public:
    KinematicRemapper() = default;
    ~KinematicRemapper() = default;

    /**
     * @brief Configure joint remapping parameters and pre-calculate linear coefficients.
     * @param configs Vector of joint configuration structs.
     * @return true if configuration succeeded.
     */
    bool configure(const std::vector<JointRemapConfig> &configs);

    /**
     * @brief Remap a single joint angle from leader to follower.
     * @param index Index of the joint in the configured vector.
     * @param leader_angle_rad Input leader joint angle in radians.
     * @return Mapped and clamped follower joint angle in radians.
     */
    double remap_joint(size_t index, double leader_angle_rad) const;

    /**
     * @brief Remap all joint angles in order.
     * @param leader_angles_rad Vector of leader joint angles in radians.
     * @return Vector of mapped follower joint angles in radians.
     */
    std::vector<double> remap_all(const std::vector<double> &leader_angles_rad) const;

    [[nodiscard]] size_t size() const noexcept { return configs_.size(); }
    [[nodiscard]] bool empty() const noexcept { return configs_.empty(); }
    [[nodiscard]] const std::vector<JointRemapConfig> &get_configs() const noexcept { return configs_; }
    [[nodiscard]] const JointRemapConfig &get_config(size_t index) const;
    [[nodiscard]] int find_joint_index(const std::string &joint_name) const;

    static RemapMode string_to_mode(const std::string &mode_str);
    static std::string mode_to_string(RemapMode mode);

  private:
    static void precompute_joint(JointRemapConfig &cfg);

    std::vector<JointRemapConfig> configs_;
  };

} // namespace teleop_zhongli_servo_hw

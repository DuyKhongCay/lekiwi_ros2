/**
 * @file system_readiness_state.hpp
 * @brief Pure domain state machine and evaluator for LeKiwi system readiness.
 *
 * Implements decoupled contracts for verifying local odometry freshness, robot
 * stationary state, global EKF convergence, arm joint state completeness, and
 * critical TF chain transformations before authorizing autonomous navigation or
 * precision manipulation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__SYSTEM_READINESS_STATE_HPP_
#define LEKIWI_MOTION__SYSTEM_READINESS_STATE_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace lekiwi_motion
{

  /**
   * @brief Inline stateless evaluation policies for timestamp freshness, motion, and variance.
   */
  namespace policy
  {
    /**
     * @brief Checks if a signal timestamp is finite, non-zero, and within the max allowed age.
     * @param[in] now Current reference time in seconds.
     * @param[in] stamp Message timestamp in seconds.
     * @param[in] max_age Maximum allowed age in seconds.
     * @return True if timestamp is fresh and not in the future.
     */
    inline bool fresh(double now, double stamp, double max_age)
    {
      return std::isfinite(now) && std::isfinite(stamp) && stamp > 0.0 &&
             now >= stamp && (now - stamp) <= max_age;
    }

    /**
     * @brief Evaluates whether linear and angular velocities fall within stationary thresholds.
     * @param[in] vx Linear velocity along X (m/s).
     * @param[in] vy Linear velocity along Y (m/s).
     * @param[in] wz Angular velocity around Z (rad/s).
     * @param[in] linear_limit Upper bound on planar linear speed (m/s).
     * @param[in] angular_limit Upper bound on yaw angular speed (rad/s).
     * @return True if robot velocities are within stationary bounds.
     */
    inline bool stationary(double vx, double vy, double wz, double linear_limit, double angular_limit)
    {
      return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
             std::hypot(vx, vy) <= linear_limit && std::abs(wz) <= angular_limit;
    }

    /**
     * @brief Verifies whether position and yaw variances satisfy convergence thresholds.
     * @param[in] var_x Positional variance along X (m^2).
     * @param[in] var_y Positional variance along Y (m^2).
     * @param[in] var_yaw Orientation variance around Z (rad^2).
     * @param[in] max_pos Maximum allowed cumulative planar position variance (m^2).
     * @param[in] max_yaw Maximum allowed yaw variance (rad^2).
     * @return True if filter variances indicate spatial convergence.
     */
    inline bool converged(double var_x, double var_y, double var_yaw, double max_pos, double max_yaw)
    {
      return std::isfinite(var_x) && std::isfinite(var_y) && std::isfinite(var_yaw) &&
             var_x >= 0.0 && var_y >= 0.0 && var_yaw >= 0.0 &&
             (var_x + var_y) <= max_pos && var_yaw <= max_yaw;
    }
  } // namespace policy

  /**
   * @struct LocalOdomSnapshot
   * @brief Cached state of high-rate local wheel odometry.
   */
  struct LocalOdomSnapshot
  {
    bool valid{false};        ///< True if at least one valid message was received.
    double stamp_sec{0.0};    ///< Epoch timestamp in seconds.
    double vx{0.0};           ///< Planar linear velocity X (m/s).
    double vy{0.0};           ///< Planar linear velocity Y (m/s).
    double wz{0.0};           ///< Yaw angular velocity (rad/s).

    /**
     * @brief Computes planar Euclidean ground speed.
     * @return Resulting planar velocity magnitude (m/s).
     */
    [[nodiscard]] double speed() const noexcept
    {
      return std::hypot(vx, vy);
    }
  };

  /**
   * @struct GlobalOdomSnapshot
   * @brief Cached state of global sensor fusion (EKF/AprilTag) with covariance.
   */
  struct GlobalOdomSnapshot
  {
    bool valid{false};                                                   ///< True if message received.
    double stamp_sec{0.0};                                               ///< Epoch timestamp in seconds.
    double pos_variance{std::numeric_limits<double>::infinity()};        ///< Cumulative X-Y position variance (m^2).
    double yaw_variance{std::numeric_limits<double>::infinity()};        ///< Heading variance around Z (rad^2).
  };

  /**
   * @struct JointsSnapshot
   * @brief Cached set of active joint names from robot state publisher.
   */
  struct JointsSnapshot
  {
    bool valid{false};                    ///< True if joint state update received.
    double stamp_sec{0.0};                ///< Epoch timestamp in seconds.
    std::set<std::string> joint_names;    ///< Set of published joint identifiers.
  };

  /**
   * @struct TfChainSnapshot
   * @brief Freshness status flags for essential coordinate transform links.
   */
  struct TfChainSnapshot
  {
    bool odom_to_base_fresh{false};       ///< Odom -> base_footprint transform status.
    bool base_to_gripper_fresh{false};    ///< base_footprint -> gripperframe transform status.
    bool map_to_board_fresh{false};       ///< Map -> chessboard_frame transform status.
  };

  /**
   * @struct EvaluatorConfig
   * @brief Operational parameters and tolerance bounds for readiness evaluation.
   */
  struct EvaluatorConfig
  {
    double max_pos_variance{0.0012};         ///< Maximum acceptable positional variance for grasp readiness (m^2).
    double max_yaw_variance{0.0030};         ///< Maximum acceptable yaw variance for grasp readiness (rad^2).
    double max_transform_age_sec{0.30};      ///< Max age for precision transform chains (s).
    double nav_odom_max_age_sec{0.50};       ///< Max age for local wheel odometry (s).
    double max_stop_velocity{0.03};          ///< Linear speed threshold below which robot is stationary (m/s).
    double max_stop_angular_vel{0.08};       ///< Angular speed threshold below which robot is stationary (rad/s).
    bool require_global_ekf_seed{true};      ///< Require global EKF convergence at least once before nav ready.
    std::vector<std::string> required_arm_joints{
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"}; ///< Required joint names for arm integrity.
  };

  /**
   * @enum DriftType
   * @brief Categorization of EKF filter drift causes.
   */
  enum class DriftType : uint8_t
  {
    NONE = 0,               ///< Filter converged, zero drift detected.
    MOVING = 1,             ///< Robot in active motion; kinematic slippage expected.
    NOT_SEEDED = 2,         ///< Stationary but global filter has not yet converged once.
    STATIONARY_NO_TAG = 3   ///< Stationary but AprilTag fiducial lost or variance exceeding bounds.
  };

  /**
   * @struct SystemReadinessReport
   * @brief Diagnostic and gatekeeper report summarizing system capabilities.
   */
  struct SystemReadinessReport
  {
    bool nav_ready{false};                                                ///< True if base navigation is authorized.
    bool grasp_ready{false};                                              ///< True if precision grasping is authorized.
    bool is_stationary{false};                                            ///< True if robot velocities are below stop thresholds.
    double current_speed{0.0};                                            ///< Current planar ground speed (m/s).
    double current_pos_variance{std::numeric_limits<double>::infinity()}; ///< Current EKF position variance (m^2).
    std::string grasp_blocker_reason;                                     ///< Human-readable explanation if grasp inhibited.
    DriftType drift_type{DriftType::NONE};                                ///< Diagnostic drift classification.
    bool stationary_latched{false};                                       ///< True if stationary state maintains grasp validity.
  };

  /**
   * @class SystemReadinessEvaluator
   * @brief Pure domain evaluator determining multi-tier readiness for mobility and manipulation.
   */
  class SystemReadinessEvaluator
  {
  public:
    /**
     * @brief Constructs evaluator with configuration parameters.
     * @param[in] config Operational parameters and thresholds.
     */
    explicit SystemReadinessEvaluator(const EvaluatorConfig &config = EvaluatorConfig());

    /**
     * @brief Evaluates incoming sensor snapshots to produce a system readiness report.
     *
     * @details Executes a two-tier evaluation policy:
     * - Tier 1 (Navigation Readiness): Requires fresh local odometry, valid odom-to-base TF,
     *   and historical EKF initialization (if configured).
     * - Tier 2 (Grasp Readiness): Requires stationary chassis, complete arm joint states,
     *   converged or stationary-latched global EKF, and valid end-effector/board TF chains.
     *
     * @param[in] now_sec Current monotonic epoch time in seconds.
     * @param[in] local_odom High-rate local wheel odometry snapshot.
     * @param[in] global_odom Fused global odometry snapshot with covariance.
     * @param[in] joints Published joint state snapshot.
     * @param[in] tf_chains Status of prerequisite TF tree transforms.
     * @return Detailed SystemReadinessReport with boolean gates and diagnostic blocker reason.
     * @pre now_sec must be non-negative and finite.
     * @post Evaluator state updates stationary latching and EKF seeding status.
     * @note Pure domain logic; does not depend on ROS 2 runtime execution.
     */
    [[nodiscard]] SystemReadinessReport evaluate(
        double now_sec,
        const LocalOdomSnapshot &local_odom,
        const GlobalOdomSnapshot &global_odom,
        const JointsSnapshot &joints,
        const TfChainSnapshot &tf_chains);

    /**
     * @brief Records whether global EKF has reached convergence at least once.
     * @param[in] converged True if filter covariance satisfied convergence bounds.
     */
    void record_global_ekf_convergence(bool converged) noexcept;

    /**
     * @brief Resets EKF seed state and stationary latching upon epoch reset or clock jumps.
     */
    void reset_global_ekf_seed() noexcept;

    /**
     * @brief Checks if global EKF has been seeded since initialization.
     * @return True if filter has achieved convergence at least once.
     */
    [[nodiscard]] bool is_global_ekf_seeded() const noexcept { return global_ekf_seeded_; }

    /**
     * @brief Retrieves active evaluator configuration parameters.
     * @return Reference to immutable EvaluatorConfig struct.
     */
    [[nodiscard]] const EvaluatorConfig &config() const noexcept { return config_; }

  private:
    EvaluatorConfig config_;          ///< Active configuration thresholds.
    bool global_ekf_seeded_{false};   ///< Latched status indicating initial EKF convergence.
    bool stationary_latched_{false};  ///< Latch preserving grasp readiness while robot stays motionless.
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__SYSTEM_READINESS_STATE_HPP_


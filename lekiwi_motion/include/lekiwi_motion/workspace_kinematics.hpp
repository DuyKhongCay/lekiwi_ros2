/**
 * @file workspace_kinematics.hpp
 * @brief Kinematic representations, forward kinematics, and analytical inverse kinematics for the SO-101 5-DoF arm.
 *
 * Provides pure mathematical primitives (Point3D, BasePose), URDF kinematic chain
 * parsing, direct forward kinematics evaluation, and a closed-form analytical IK solver
 * for the SO-101 robotic arm mounted on the LeKiwi mobile platform.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__WORKSPACE_KINEMATICS_HPP_
#define LEKIWI_MOTION__WORKSPACE_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>
#include <urdf_model/model.h>

namespace lekiwi_motion::workspace
{

  /**
   * @struct Point3D
   * @brief 3D Cartesian point with finite coordinate validation.
   */
  struct Point3D
  {
    double x{0.0}; ///< X coordinate (meters).
    double y{0.0}; ///< Y coordinate (meters).
    double z{0.0}; ///< Z coordinate (meters).

    /**
     * @brief Checks whether all coordinates are finite numbers.
     * @return True if x, y, and z are not NaN or infinity.
     */
    [[nodiscard]] bool is_finite() const noexcept
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
  };

  /**
   * @struct BasePose
   * @brief 2D planar mobile-base pose with elevation and heading yaw.
   */
  struct BasePose
  {
    double x{0.0};   ///< Planar position along X (meters).
    double y{0.0};   ///< Planar position along Y (meters).
    double z{0.0};   ///< Base footprint elevation (meters).
    double yaw{0.0}; ///< Chassis heading angle in radians.

    /**
     * @brief Checks whether all pose coordinates and yaw are finite.
     * @return True if x, y, z, and yaw are finite numbers.
     */
    [[nodiscard]] bool is_finite() const noexcept
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(yaw);
    }
  };

  /**
   * @struct IkResult
   * @brief Outcome of an inverse kinematics query for the 5-DoF manipulator arm.
   */
  struct IkResult
  {
    bool success{false};               ///< True if a reachable, limit-compliant solution was found.
    std::array<double, 5> joints{};    ///< Solved joint angles [pan, shoulder, elbow, wrist_flex, wrist_roll] (rad).
    double radius{0.0};                ///< Planar radial distance from pan origin to target (meters).
    std::string reason;                ///< Diagnostic explanation of failure or success confirmation.
  };

  /**
   * @struct ChainSegment
   * @brief Cached transform and joint axis extracted from URDF parent-to-joint link.
   */
  struct ChainSegment
  {
    std::string name;                                          ///< Segment or joint name.
    Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};  ///< Fixed relative origin transform.
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};            ///< Normalized rotation axis in segment frame.
    int joint_index{-1};                                       ///< Index in joint array (0..4) or -1 for fixed joints.
  };

  /**
   * @struct KinematicsModel
   * @brief Complete kinematic chain model parsed from URDF structure.
   */
  struct KinematicsModel
  {
    std::string base_frame;                    ///< Base reference link identifier.
    std::string tip_frame;                     ///< Tool center point (TCP) link identifier.
    std::vector<ChainSegment> chain;           ///< Sequential ordered kinematic segments.
    std::array<std::string, 5> joint_names{};  ///< Canonical names of the 5 revolute joints.
    std::array<double, 5> lower_limits{};      ///< Lower joint position limits with safety margins (rad).
    std::array<double, 5> upper_limits{};      ///< Upper joint position limits with safety margins (rad).
    double reach_bound{0.0};                   ///< Theoretical maximum reach envelope bound (meters).
  };

  /**
   * @brief Evaluates forward kinematics across the structured URDF chain.
   * @param[in] model Initialized KinematicsModel.
   * @param[in] joints 5-element array of active joint positions (rad).
   * @return Resulting TCP pose as an Eigen::Isometry3d transformation in base frame.
   */
  Eigen::Isometry3d forward_kinematics(
      const KinematicsModel &model,
      const std::array<double, 5> &joints);

  // ================= Inverse Kinematics Solvers (Strategy Pattern) =================

  /**
   * @class IIkSolver
   * @brief Generic abstract interface for arm inverse kinematics solvers.
   */
  class IIkSolver
  {
  public:
    virtual ~IIkSolver() = default;

    /**
     * @brief Computes joint angles to place TCP at target coordinates with specified orientation.
     * @param[in] x Target X position in base frame (m).
     * @param[in] y Target Y position in base frame (m).
     * @param[in] z Target Z position in base frame (m).
     * @param[in] pitch Desired approach pitch angle (rad).
     * @param[in] roll Desired wrist roll angle (rad).
     * @return IkResult containing validity flag, joint solution, and diagnostics.
     */
    virtual IkResult solve(double x, double y, double z, double pitch, double roll) const = 0;

    /**
     * @brief Returns theoretical maximum reach envelope radius.
     * @return Upper bound on reach distance in meters.
     */
    virtual double reach_bound() const = 0;
  };

  /**
   * @class SO101AnalyticalSolver
   * @brief Closed-form analytical inverse kinematics solver for the SO-101 5-DoF manipulator.
   *
   * Solves arm kinematics analytically by decoupling base pan yaw, calculating
   * 2D planar linkage angles using law of cosines, deriving wrist pitch to maintain
   * the requested end-effector tilt, and verifying candidate configurations with FK.
   */
  class SO101AnalyticalSolver : public IIkSolver
  {
  public:
    /**
     * @brief Constructs analytical solver calibrated against URDF kinematic parameters.
     * @param[in] model Parsed and validated KinematicsModel.
     * @throws std::invalid_argument if model contains inconsistent joint axes or link lengths.
     */
    explicit SO101AnalyticalSolver(const KinematicsModel &model);

    /**
     * @brief Solves analytical inverse kinematics for Cartesian target and approach angles.
     *
     * @details The algorithm executes in several phases:
     * 1. Validates coordinate finiteness and checks model readiness.
     * 2. Computes wrist roll angle and accounts for mechanical tool offsets.
     * 3. Determines lateral shoulder offset and solves analytical pan yaw candidates.
     * 4. Applies law of cosines across the two planar pitch links (shoulder to elbow, elbow to wrist).
     * 5. Solves wrist flex pitch angle to achieve the commanded tool pitch.
     * 6. Validates joint limits and performs forward kinematics verification against tolerances.
     *
     * @param[in] x Target X coordinate in base frame (m).
     * @param[in] y Target Y coordinate in base frame (m).
     * @param[in] z Target Z coordinate in base frame (m).
     * @param[in] pitch Target approach pitch angle (rad, -pi/2 for vertical top-down).
     * @param[in] roll Target tool roll angle (rad).
     * @return IkResult containing joint array on success or failure diagnostic reason.
     * @pre Input coordinates and angles must be finite.
     * @post Returns valid solution with verified forward kinematics residual < 1mm.
     */
    IkResult solve(double x, double y, double z, double pitch, double roll) const override;

    /**
     * @brief Returns the maximum reachable sphere radius.
     * @return Upper bound on radial reach in meters.
     */
    double reach_bound() const override { return reach_bound_; }

    /**
     * @brief Returns link lengths [shoulder_to_elbow, elbow_to_wrist].
     * @return 2-element array containing link lengths in meters.
     */
    const std::array<double, 2> &link_lengths() const noexcept { return link_lengths_; }

    /**
     * @brief Returns zero-pose reference angles for pitch links.
     * @return 2-element array of reference angles in radians.
     */
    const std::array<double, 2> &link_angles() const noexcept { return link_angles_; }

  private:
    /**
     * @brief Extracts geometric offsets, axes, and link lengths from URDF model.
     * @param[in] model Kinematic model chain.
     */
    void init_geometry(const KinematicsModel &model);

    KinematicsModel model_;                         ///< Cached URDF chain and limits.
    std::array<double, 5> signs_{};                 ///< Directional sign multipliers for joints.
    Eigen::Vector3d pan_origin_{Eigen::Vector3d::Zero()};   ///< Base pan joint rotation origin.
    Eigen::Matrix3d plane_basis_{Eigen::Matrix3d::Identity()}; ///< Basis vectors defining the arm pitch plane.
    Eigen::Matrix3d tool_basis_{Eigen::Matrix3d::Identity()};  ///< Frame orientation matrix of gripper TCP.
    Eigen::Vector3d shoulder_{Eigen::Vector3d::Zero()};     ///< Shoulder offset vector in plane coordinates.
    Eigen::Vector3d first_link_{Eigen::Vector3d::Zero()};   ///< Upper arm link vector (shoulder to elbow).
    Eigen::Vector3d second_link_{Eigen::Vector3d::Zero()};  ///< Forearm link vector (elbow to wrist).
    Eigen::Vector3d wrist_origin_{Eigen::Vector3d::Zero()}; ///< Wrist roll origin point.
    std::array<double, 2> link_lengths_{};          ///< Length of upper arm and forearm segments (m).
    std::array<double, 2> link_angles_{};           ///< Neutral pitch angle offsets for arm links (rad).
    double yaw_offset_{0.0};                        ///< Angular offset between base X axis and plane normal.
    double tool_pitch_offset_{0.0};                 ///< Intrinsic pitch offset of gripper attachment.
    double tool_roll_offset_{0.0};                  ///< Intrinsic roll offset of gripper attachment.
    double reach_bound_{0.0};                       ///< Cumulative reach envelope radius (m).
  };

  // ================= Core Utilities =================

  /**
   * @brief Extracts kinematic parameters and limits directly from a parsed URDF ModelInterface.
   *
   * @details Traverses the URDF tree backwards from tip_frame to base_frame, validates that
   * all 5 joints are independent revolute joints matching joint_names in order, checks limit
   * boundaries, and computes reach bounds.
   *
   * @param[in] urdf Parsed URDF model interface.
   * @param[in] base_frame Name of mobile base reference link.
   * @param[in] tip_frame Name of arm end-effector link.
   * @param[in] joint_names Ordered vector of the 5 expected arm joint names.
   * @param[out] out_model Output KinematicsModel populated on success.
   * @param[out] error_msg Diagnostic message describing failure cause if return is false.
   * @param[in] safety_margin_rad Margin subtracted from joint limits (default 0.05 rad).
   * @return True if kinematics chain successfully validated and parsed.
   */
  bool extract_kinematics_from_urdf(
      const urdf::ModelInterface &urdf,
      const std::string &base_frame,
      const std::string &tip_frame,
      const std::vector<std::string> &joint_names,
      KinematicsModel &out_model,
      std::string &error_msg,
      double safety_margin_rad = 0.05);

  /**
   * @brief Projects a 3D coordinate point from board frame into planar base frame.
   * @param[in] pt 3D coordinate point in board frame.
   * @param[in] base Planar pose of the mobile base in board frame.
   * @return Point3D transformed into local base_footprint frame.
   */
  Point3D transform_point_to_base_frame(
      const Point3D &pt,
      const BasePose &base);

} // namespace lekiwi_motion::workspace

#endif // LEKIWI_MOTION__WORKSPACE_KINEMATICS_HPP_


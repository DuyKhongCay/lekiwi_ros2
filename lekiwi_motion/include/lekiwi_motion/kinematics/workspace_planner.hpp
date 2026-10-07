/**
 * @file workspace_planner.hpp
 * @brief Multi-tier mobile standoff and arm manipulation move planner for chess playing.
 *
 * Implements hierarchical motion planning searching for minimal-navigation solutions:
 * - Tier 0: Zero-Nav (robot stationary at current base pose).
 * - Tier 1: Single Standoff Base (single navigation goal covering pick and place).
 * - Tier 2: Dual Standoff Bases (independent pick base and place base).
 * - Tier 3: Capture Multi-Base (orchestrating clear, pick, and place across 1, 2, or 3 poses).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__WORKSPACE_PLANNER_HPP_
#define LEKIWI_MOTION__WORKSPACE_PLANNER_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "lekiwi_motion/kinematics/workspace_kinematics.hpp"

namespace lekiwi_motion::workspace
{

  // Plan types matching service contract
  constexpr uint8_t PLAN_ZERO_NAV = 0;             ///< Quiet move executed entirely from current chassis location.
  constexpr uint8_t PLAN_SINGLE_BASE = 1;          ///< Quiet move executed from a single standoff navigation pose.
  constexpr uint8_t PLAN_DUAL_BASE = 2;            ///< Quiet move requiring separate pick base and place base.
  constexpr uint8_t PLAN_CAPTURE_ZERO_NAV = 3;     ///< Capture move executed entirely from current chassis location.
  constexpr uint8_t PLAN_CAPTURE_SINGLE_BASE = 4;  ///< Capture move executed from a single standoff navigation pose.
  constexpr uint8_t PLAN_CAPTURE_DUAL_BASE = 5;    ///< Capture move sharing one base for clear/pick or pick/place.
  constexpr uint8_t PLAN_CAPTURE_TRIPLE_BASE = 6;  ///< Capture move requiring three distinct standoff bases.

  /**
   * @struct WorkspaceConfig
   * @brief Physical board parameters and numerical standoff search budget.
   */
  struct WorkspaceConfig
  {
    double half_w{0.195};            ///< Half-width of board envelope along X in meters.
    double half_h{0.195};            ///< Half-height of board envelope along Y in meters.
    double edge_clearance{0.147};    ///< Nominal vehicle standoff distance from board boundary in meters.
    double sample_step{0.025};       ///< Standoff grid discretization sampling step in meters.
    int max_samples{257};            ///< Maximum candidate poses evaluated per move query.
    double default_pitch{-M_PI_2};   ///< Default end-effector approach pitch in radians (-pi/2 for top-down).
    double default_roll{0.0};        ///< Default end-effector wrist roll in radians.

    /**
     * @brief Validates config fields for positive finite dimensions.
     * @return True if parameters are positive, finite, and within valid search bounds.
     */
    [[nodiscard]] bool is_valid() const noexcept
    {
      return std::isfinite(half_w) && half_w > 0.0 &&
             std::isfinite(half_h) && half_h > 0.0 &&
             std::isfinite(edge_clearance) && edge_clearance > 0.0 &&
             std::isfinite(sample_step) && sample_step > 0.0 &&
             max_samples >= 1 && max_samples <= 1001 &&
             std::isfinite(default_pitch) && std::isfinite(default_roll);
    }
  };

  /**
   * @struct PlanningRequest
   * @brief High-level Cartesian endpoints for quiet or capture chess move execution.
   */
  struct PlanningRequest
  {
    Point3D clear;          ///< Metric position of captured piece to clear from board (board frame).
    Point3D pick;           ///< Metric position of piece to pick up (board frame).
    Point3D place;          ///< Metric position where piece will be placed (board frame).
    double pitch{-M_PI_2};  ///< Desired gripper approach pitch angle in radians.
    bool is_capture{false}; ///< True if move captures an opposing piece.

    /**
     * @brief Checks if request endpoints contain finite coordinates.
     * @return True if pick, place, pitch, and (if capture) clear coordinates are finite.
     */
    [[nodiscard]] bool is_valid() const noexcept
    {
      return pick.is_finite() && place.is_finite() && std::isfinite(pitch) &&
             (!is_capture || clear.is_finite());
    }
  };

  /**
   * @struct PlanningContext
   * @brief Current environment context such as robot's existing chassis position.
   */
  struct PlanningContext
  {
    std::optional<BasePose> current_base; ///< Existing robot base pose in board frame, if available.
  };

  /**
   * @enum FeasibilityStatus
   * @brief Strongly-typed enumeration of move planning outcomes and blocker reasons.
   */
  enum class FeasibilityStatus : uint8_t
  {
    SUCCESS = 0,                   ///< Valid reach configuration and standoff plan discovered.
    UNREACHABLE_KINEMATICS,        ///< Target square beyond physical arm reach or joint limits.
    BASE_STANDOFF_EXHAUSTED,       ///< Candidate search budget exceeded without finding viable standoff.
    TF_STALE,                      ///< Coordinate frame transform between board and base is stale.
    MODEL_NOT_READY,               ///< Kinematics model or analytical solver not initialized.
    MALFORMED_REQUEST              ///< Input points or notation violate domain contracts.
  };

  /**
   * @brief Converts FeasibilityStatus to human-readable string_view.
   * @param[in] status Status enum value.
   * @return String literal representation.
   */
  constexpr std::string_view to_string(FeasibilityStatus status) noexcept
  {
    switch (status)
    {
    case FeasibilityStatus::SUCCESS:
      return "SUCCESS";
    case FeasibilityStatus::UNREACHABLE_KINEMATICS:
      return "UNREACHABLE_KINEMATICS";
    case FeasibilityStatus::BASE_STANDOFF_EXHAUSTED:
      return "BASE_STANDOFF_EXHAUSTED";
    case FeasibilityStatus::TF_STALE:
      return "TF_STALE";
    case FeasibilityStatus::MODEL_NOT_READY:
      return "MODEL_NOT_READY";
    case FeasibilityStatus::MALFORMED_REQUEST:
      return "MALFORMED_REQUEST";
    }
    return "UNKNOWN";
  }

  /**
   * @struct PlanResult
   * @brief Output plan specifying standoff bases, joint angles, and plan classification.
   */
  struct PlanResult
  {
    bool feasible{false};                                         ///< True if move is executable.
    uint8_t plan_type{0};                                         ///< Plan classification (PLAN_ZERO_NAV .. PLAN_CAPTURE_TRIPLE_BASE).
    FeasibilityStatus status{FeasibilityStatus::MODEL_NOT_READY}; ///< Discrete outcome code.
    BasePose clear_base;                                          ///< Target base pose for clearing captured piece.
    BasePose pick_base;                                           ///< Target base pose for picking chess piece.
    BasePose place_base;                                          ///< Target base pose for placing chess piece.
    std::optional<std::array<double, 5>> clear_joints;            ///< 5-DoF joint angles for piece clearance.
    std::array<double, 5> pick_joints{};                          ///< 5-DoF joint angles for piece pick.
    std::optional<std::array<double, 5>> place_joints;            ///< 5-DoF joint angles for piece placement.
    std::string message;                                          ///< Diagnostic summary or plan type name.
    int evaluated_candidates{0};                                  ///< Number of candidate base poses evaluated.
  };

  /**
   * @class WorkspacePlanner
   * @brief Domain service for mobile standoff candidate search and multi-tier move planning.
   *
   * Evaluates concentric shell standoff candidates along board edges to minimize navigation
   * overhead while guaranteeing kinematic feasibility for pick, place, and capture tasks.
   */
  class WorkspacePlanner
  {
  public:
    /**
     * @brief Constructs workspace planner with kinematics model, IK solver, and search configuration.
     * @param[in] model Shared pointer to immutable KinematicsModel.
     * @param[in] solver Shared pointer to analytical inverse kinematics solver.
     * @param[in] config Board dimensions and search sampling configuration.
     */
    WorkspacePlanner(
        std::shared_ptr<const KinematicsModel> model,
        std::shared_ptr<const IIkSolver> solver,
        WorkspaceConfig config);

    /**
     * @brief Plans minimal-navigation standoff poses and joint solutions for a chess move request.
     *
     * @details Evaluates hierarchical planning tiers:
     * - First tests current chassis pose for Zero-Nav feasibility.
     * - Searches single standoff poses along nearest board perimeter edges.
     * - Expands to multi-base sequences if single base cannot simultaneously reach endpoints.
     *
     * @param[in] req PlanningRequest containing target points in board coordinates.
     * @param[in] ctx PlanningContext containing current robot base pose.
     * @param[in] base_z Base elevation in board frame (meters).
     * @return PlanResult populated with base goals and joint configurations on success.
     */
    [[nodiscard]] PlanResult plan(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z) const;

    /**
     * @brief Generates ranked mobile base standoff candidates around board edges.
     *
     * @details Evaluates outer perimeter boundary lines, sorts edges nearest to target
     * centroids, and generates outward expanding concentric standoff shells with lateral offsets.
     *
     * @param[in] targets Target points that candidates must enclose within arm reach.
     * @param[in] base_z Chassis elevation in board coordinates.
     * @return Vector of candidate BasePose objects ranked by proximity.
     */
    [[nodiscard]] std::vector<BasePose> generate_standoff_candidates(
        const std::vector<Point3D> &targets,
        double base_z) const;

    [[nodiscard]] const WorkspaceConfig &config() const noexcept { return config_; }
    [[nodiscard]] const KinematicsModel &model() const noexcept { return *model_; }
    [[nodiscard]] const IIkSolver &solver() const noexcept { return *solver_; }

  private:
    using EndpointSolution = std::pair<BasePose, IkResult>;

    // Sub-pipeline planners for quiet and capture workflows
    PlanResult plan_capture(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z) const;

    PlanResult plan_quiet(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z) const;

    // Helper evaluation routines
    IkResult solve_at_base(
        const Point3D &pt,
        const BasePose &base,
        double pitch) const;

    std::optional<PlanResult> try_single_pose(
        const PlanningRequest &req,
        const BasePose &base,
        uint8_t plan_type) const;

    bool is_pose_within_reach(
        const BasePose &base,
        const std::vector<Point3D> &targets,
        double reach) const;

    std::optional<PlanResult> try_single_pose_capture(
        const PlanningRequest &req,
        const BasePose &base,
        uint8_t plan_type) const;

    std::optional<EndpointSolution> find_endpoint(
        const Point3D &pt,
        double pitch,
        double base_z,
        int limit,
        int &evaluated) const;

    std::optional<EndpointSolution> resolve_clear_endpoint(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z,
        int &evaluated) const;

    std::optional<EndpointSolution> resolve_pick_endpoint(
        const PlanningRequest &req,
        const BasePose &clear_base,
        double base_z,
        int &evaluated) const;

    std::optional<EndpointSolution> resolve_place_endpoint(
        const PlanningRequest &req,
        const BasePose &clear_base,
        const BasePose &pick_base,
        double base_z,
        int &evaluated) const;

    PlanResult build_capture_plan_result(
        const EndpointSolution &clear_ep,
        const EndpointSolution &pick_ep,
        const EndpointSolution &place_ep,
        const PlanningContext &ctx,
        int evaluated) const;

    std::shared_ptr<const KinematicsModel> model_;  ///< Kinematics chain specification.
    std::shared_ptr<const IIkSolver> solver_;        ///< Analytical IK solver instance.
    WorkspaceConfig config_;                        ///< Standoff parameters and search limits.
  };

} // namespace lekiwi_motion::workspace

#endif // LEKIWI_MOTION__WORKSPACE_PLANNER_HPP_


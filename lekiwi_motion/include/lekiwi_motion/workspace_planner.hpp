// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

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

#include "lekiwi_motion/workspace_kinematics.hpp"

namespace lekiwi_motion::workspace
{

  // Plan types matching service contract
  constexpr uint8_t PLAN_ZERO_NAV = 0;
  constexpr uint8_t PLAN_SINGLE_BASE = 1;
  constexpr uint8_t PLAN_DUAL_BASE = 2;
  constexpr uint8_t PLAN_CAPTURE_ZERO_NAV = 3;
  constexpr uint8_t PLAN_CAPTURE_SINGLE_BASE = 4;
  constexpr uint8_t PLAN_CAPTURE_DUAL_BASE = 5;
  constexpr uint8_t PLAN_CAPTURE_TRIPLE_BASE = 6;

  // Board & search budget configuration
  struct WorkspaceConfig
  {
    double half_w{0.195};
    double half_h{0.195};
    double edge_clearance{0.147};
    double sample_step{0.025};
    int max_samples{257};
    double default_pitch{-M_PI_2};
    double default_roll{0.0};

    bool is_valid() const noexcept
    {
      return std::isfinite(half_w) && half_w > 0.0 &&
             std::isfinite(half_h) && half_h > 0.0 &&
             std::isfinite(edge_clearance) && edge_clearance > 0.0 &&
             std::isfinite(sample_step) && sample_step > 0.0 &&
             max_samples >= 1 && max_samples <= 1001 &&
             std::isfinite(default_pitch) && std::isfinite(default_roll);
    }
  };

  // Endpoint planning request
  struct PlanningRequest
  {
    Point3D clear;
    Point3D pick;
    Point3D place;
    double pitch{-M_PI_2};
    bool is_capture{false};

    bool is_valid() const noexcept
    {
      return pick.is_finite() && place.is_finite() && std::isfinite(pitch) &&
             (!is_capture || clear.is_finite());
    }
  };

  // Planning context (current mobile base pose if available from TF)
  struct PlanningContext
  {
    std::optional<BasePose> current_base;
  };

  // Strongly-typed feasibility status replacing stringly-typed messages
  enum class FeasibilityStatus : uint8_t
  {
    SUCCESS = 0,
    UNREACHABLE_KINEMATICS,
    BASE_STANDOFF_EXHAUSTED,
    TF_STALE,
    MODEL_NOT_READY,
    MALFORMED_REQUEST
  };

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

  // Complete move plan result
  struct PlanResult
  {
    bool feasible{false};
    uint8_t plan_type{0};
    FeasibilityStatus status{FeasibilityStatus::MODEL_NOT_READY};
    BasePose clear_base;
    BasePose pick_base;
    BasePose place_base;
    std::optional<std::array<double, 5>> clear_joints;
    std::array<double, 5> pick_joints{};
    std::optional<std::array<double, 5>> place_joints;
    std::string message;
    int evaluated_candidates{0};
  };

  /**
   * @brief Domain service for mobile standoff candidate search and multi-tier move planning.
   */
  class WorkspacePlanner
  {
  public:
    WorkspacePlanner(
        std::shared_ptr<const KinematicsModel> model,
        std::shared_ptr<const IIkSolver> solver,
        WorkspaceConfig config);

    PlanResult plan(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z) const;

    // Generalized standoff candidate generator across N target points
    std::vector<BasePose> generate_standoff_candidates(
        const std::vector<Point3D> &targets,
        double base_z) const;

    // Overloads for backwards compatibility
    std::vector<BasePose> generate_standoff_candidates(
        const Point3D &pick,
        const Point3D &place,
        double base_z) const;

    std::vector<BasePose> generate_standoff_candidates_3(
        const Point3D &p1,
        const Point3D &p2,
        const Point3D &p3,
        double base_z) const;

    const WorkspaceConfig &config() const noexcept { return config_; }
    const KinematicsModel &model() const noexcept { return *model_; }
    const IIkSolver &solver() const noexcept { return *solver_; }

  private:
    using EndpointSolution = std::pair<BasePose, IkResult>;

    // Sub-pipeline planners (Cognitive Complexity reduction)
    PlanResult plan_capture(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z) const;

    PlanResult plan_quiet(
        const PlanningRequest &req,
        const PlanningContext &ctx,
        double base_z) const;

    // Helper solvers
    IkResult solve_at_base(
        const Point3D &pt,
        const BasePose &base,
        double pitch) const;

    std::optional<PlanResult> try_single_pose(
        const PlanningRequest &req,
        const BasePose &base,
        uint8_t plan_type) const;

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

    std::shared_ptr<const KinematicsModel> model_;
    std::shared_ptr<const IIkSolver> solver_;
    WorkspaceConfig config_;
  };

} // namespace lekiwi_motion::workspace

#endif // LEKIWI_MOTION__WORKSPACE_PLANNER_HPP_

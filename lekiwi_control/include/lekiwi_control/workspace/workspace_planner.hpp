// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_CONTROL__WORKSPACE__WORKSPACE_PLANNER_HPP_
#define LEKIWI_CONTROL__WORKSPACE__WORKSPACE_PLANNER_HPP_

#include <optional>
#include <string>
#include <vector>

#include <lekiwi_interfaces/srv/check_move_feasibility.hpp>

#include "lekiwi_control/workspace/kinematics_engine.hpp"
#include "lekiwi_control/workspace/types.hpp"

namespace lekiwi_control::workspace
{

  using SrvResponse = lekiwi_interfaces::srv::CheckMoveFeasibility::Response;
  constexpr uint8_t PLAN_ZERO_NAV = SrvResponse::PLAN_ZERO_NAV;
  constexpr uint8_t PLAN_SINGLE_BASE = SrvResponse::PLAN_SINGLE_BASE;
  constexpr uint8_t PLAN_DUAL_BASE = SrvResponse::PLAN_DUAL_BASE;

  inline IkResult solve_at_base(
      const Point3D &pt, const BasePose &base, double pitch,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    Point3D pt_base = transform_point_to_base_frame(pt, base);
    return solve_analytical_ik(
        pt_base.x, pt_base.y, pt_base.z, kinematics, pitch, workspace.default_roll);
  }

  inline std::optional<PlanResult> try_pair(
      const PlanningRequest &req, const BasePose &base, uint8_t plan_type,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    // 1. Solve pick first (Short-circuit optimization)
    IkResult pick_ik = solve_at_base(req.pick, base, req.pitch, kinematics, workspace);
    if (!pick_ik.success)
    {
      return std::nullopt;
    }

    // 2. Solve place only when pick is reachable
    IkResult place_ik = solve_at_base(req.place, base, req.pitch, kinematics, workspace);
    if (!place_ik.success)
    {
      return std::nullopt;
    }

    PlanResult res;
    res.feasible = true;
    res.plan_type = plan_type;
    res.pick_base = base;
    res.place_base = base;
    res.pick_joints = pick_ik.joints;
    res.place_joints = place_ik.joints;
    res.message = (plan_type == PLAN_ZERO_NAV) ? "ZERO_NAV" : "SINGLE_BASE";
    return res;
  }

  inline PlanResult plan_capture(
      const PlanningRequest &req, double base_z,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    auto [base, edge] = compute_standoff_pose(req.pick.x, req.pick.y, base_z, workspace);
    IkResult ik = solve_at_base(req.pick, base, req.pitch, kinematics, workspace);

    PlanResult res;
    if (!ik.success)
    {
      res.feasible = false;
      res.message = "Capture pick: " + ik.reason;
      return res;
    }

    res.feasible = true;
    res.plan_type = PLAN_SINGLE_BASE;
    res.pick_base = base;
    res.place_base = base;
    res.pick_joints = ik.joints;
    res.place_joints = std::nullopt;
    res.message = "Capture pick at " + edge + "; onboard drop is not evaluated";
    return res;
  }

  inline std::optional<PlanResult> try_zero_nav(
      const PlanningRequest &req, const PlanningContext &ctx,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    if (!ctx.current_base.has_value())
    {
      return std::nullopt;
    }
    return try_pair(req, ctx.current_base.value(), PLAN_ZERO_NAV, kinematics, workspace);
  }

  inline std::optional<PlanResult> try_single_base(
      const PlanningRequest &req, double base_z,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    if (!is_single_base_geometrically_possible(req.pick, req.place, kinematics))
    {
      return std::nullopt;
    }

    auto candidates = generate_standoff_candidates(req.pick, req.place, base_z, workspace);
    for (const auto &base : candidates)
    {
      auto res = try_pair(req, base, PLAN_SINGLE_BASE, kinematics, workspace);
      if (res.has_value())
      {
        return res;
      }
    }
    return std::nullopt;
  }

  inline PlanResult plan_dual_base(
      const PlanningRequest &req, double base_z,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    auto [pick_base, pick_edge] = compute_standoff_pose(req.pick.x, req.pick.y, base_z, workspace);
    auto [place_base, place_edge] = compute_standoff_pose(req.place.x, req.place.y, base_z, workspace);

    IkResult pick_ik = solve_at_base(req.pick, pick_base, req.pitch, kinematics, workspace);
    IkResult place_ik = solve_at_base(req.place, place_base, req.pitch, kinematics, workspace);

    PlanResult res;
    if (!pick_ik.success || !place_ik.success)
    {
      res.feasible = false;
      res.message = "Dual base IK: pick=" + pick_ik.reason + "; place=" + place_ik.reason;
      return res;
    }

    res.feasible = true;
    res.plan_type = PLAN_DUAL_BASE;
    res.pick_base = pick_base;
    res.place_base = place_base;
    res.pick_joints = pick_ik.joints;
    res.place_joints = place_ik.joints;
    res.message = "DUAL_BASE: reposition between pick and place; trajectories are not evaluated";
    return res;
  }

  inline PlanResult plan_move(
      const PlanningRequest &req, const PlanningContext &ctx, double base_z,
      const KinematicsConfig &kinematics, const WorkspaceConfig &workspace)
  {
    PlanResult res;
    if (!req.is_valid())
    {
      res.feasible = false;
      res.message = "Planning request contains nonfinite values";
      return res;
    }
    if (ctx.current_base.has_value() && !ctx.current_base->is_finite())
    {
      res.feasible = false;
      res.message = "Current base pose contains nonfinite values";
      return res;
    }

    if (req.is_capture)
    {
      return plan_capture(req, base_z, kinematics, workspace);
    }

    auto zero = try_zero_nav(req, ctx, kinematics, workspace);
    if (zero.has_value())
    {
      return zero.value();
    }

    auto single = try_single_base(req, base_z, kinematics, workspace);
    if (single.has_value())
    {
      return single.value();
    }

    return plan_dual_base(req, base_z, kinematics, workspace);
  }

} // namespace lekiwi_control::workspace

#endif // LEKIWI_CONTROL__WORKSPACE__WORKSPACE_PLANNER_HPP_

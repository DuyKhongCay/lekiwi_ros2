// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/workspace_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lekiwi_motion::workspace
{

  namespace
  {
    BasePose compute_edge_base_pose(
        int edge, double along, double clearance, double base_z, double half_w, double half_h)
    {
      BasePose base;
      base.z = base_z;
      if (edge == 0)
      {
        base.x = along;
        base.y = -half_h - clearance;
        base.yaw = M_PI_2;
      }
      else if (edge == 1)
      {
        base.x = along;
        base.y = half_h + clearance;
        base.yaw = -M_PI_2;
      }
      else if (edge == 2)
      {
        base.x = -half_w - clearance;
        base.y = along;
        base.yaw = 0.0;
      }
      else
      {
        base.x = half_w + clearance;
        base.y = along;
        base.yaw = M_PI;
      }
      return base;
    }

    std::array<int, 4> sort_edges_by_distance(
        double mid_x, double mid_y, double half_w, double half_h)
    {
      const std::array<double, 4> edge_dist{
          mid_y + half_h, half_h - mid_y,
          mid_x + half_w, half_w - mid_x};
      std::array<int, 4> edges{0, 1, 2, 3};
      std::stable_sort(edges.begin(), edges.end(), [&](int a, int b)
                       { return edge_dist[a] < edge_dist[b]; });
      return edges;
    }
  } // namespace

  WorkspacePlanner::WorkspacePlanner(
      std::shared_ptr<const KinematicsModel> model,
      std::shared_ptr<const IIkSolver> solver,
      WorkspaceConfig config)
      : model_(std::move(model)), solver_(std::move(solver)), config_(std::move(config))
  {
  }

  IkResult WorkspacePlanner::solve_at_base(
      const Point3D &pt, const BasePose &base, double pitch) const
  {
    const auto local = transform_point_to_base_frame(pt, base);
    return solver_->solve(local.x, local.y, local.z, pitch, config_.default_roll);
  }

  std::optional<PlanResult> WorkspacePlanner::try_single_pose(
      const PlanningRequest &req, const BasePose &base, uint8_t plan_type) const
  {
    const auto pick = solve_at_base(req.pick, base, req.pitch);
    if (!pick.success)
    {
      return std::nullopt;
    }
    const auto place = solve_at_base(req.place, base, req.pitch);
    if (!place.success)
    {
      return std::nullopt;
    }

    PlanResult result;
    result.feasible = true;
    result.status = FeasibilityStatus::SUCCESS;
    result.plan_type = plan_type;
    result.pick_base = result.place_base = base;
    result.pick_joints = pick.joints;
    result.place_joints = place.joints;
    result.message = (plan_type == PLAN_ZERO_NAV) ? "ZERO_NAV" : "SINGLE_BASE";
    return result;
  }

  std::optional<PlanResult> WorkspacePlanner::try_single_pose_capture(
      const PlanningRequest &req, const BasePose &base, uint8_t plan_type) const
  {
    const auto clear = solve_at_base(req.clear, base, req.pitch);
    if (!clear.success)
    {
      return std::nullopt;
    }
    const auto pick = solve_at_base(req.pick, base, req.pitch);
    if (!pick.success)
    {
      return std::nullopt;
    }
    const auto place = solve_at_base(req.place, base, req.pitch);
    if (!place.success)
    {
      return std::nullopt;
    }

    PlanResult result;
    result.feasible = true;
    result.status = FeasibilityStatus::SUCCESS;
    result.plan_type = plan_type;
    result.clear_base = result.pick_base = result.place_base = base;
    result.clear_joints = clear.joints;
    result.pick_joints = pick.joints;
    result.place_joints = place.joints;
    result.message = (plan_type == PLAN_CAPTURE_ZERO_NAV) ? "CAPTURE_ZERO_NAV" : "CAPTURE_SINGLE_BASE";
    return result;
  }

  std::optional<WorkspacePlanner::EndpointSolution> WorkspacePlanner::find_endpoint(
      const Point3D &pt, double pitch, double base_z, int limit, int &evaluated) const
  {
    if (evaluated >= limit)
    {
      return std::nullopt;
    }
    for (const auto &base : generate_standoff_candidates(pt, pt, base_z))
    {
      if (evaluated >= limit)
      {
        break;
      }
      ++evaluated;
      auto ik = solve_at_base(pt, base, pitch);
      if (ik.success)
      {
        return std::make_pair(base, ik);
      }
    }
    return std::nullopt;
  }

  std::vector<BasePose> WorkspacePlanner::generate_standoff_candidates(
      const std::vector<Point3D> &targets,
      double base_z) const
  {
    std::vector<BasePose> candidates;
    if (!config_.is_valid() || targets.empty() || !std::isfinite(base_z) || !solver_)
    {
      return candidates;
    }
    for (const auto &pt : targets)
    {
      if (!pt.is_finite())
      {
        return candidates;
      }
    }
    candidates.reserve(config_.max_samples);

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &pt : targets)
    {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    const double inv_n = 1.0 / static_cast<double>(targets.size());
    const double mid_x = sum_x * inv_n;
    const double mid_y = sum_y * inv_n;

    const auto edges = sort_edges_by_distance(mid_x, mid_y, config_.half_w, config_.half_h);
    const double reach = solver_->reach_bound();
    int generated = 0;

    for (int shell = 0; shell < config_.max_samples && static_cast<int>(candidates.size()) < config_.max_samples; ++shell)
    {
      for (int depth = 0; depth <= shell && static_cast<int>(candidates.size()) < config_.max_samples; ++depth)
      {
        const int lateral_idx = shell - depth;
        const double clearance = config_.edge_clearance + depth * config_.sample_step;
        if (clearance > reach)
        {
          continue;
        }

        for (int edge : edges)
        {
          if (++generated > 8 * config_.max_samples)
          {
            return candidates;
          }

          const bool horizontal = (edge < 2);
          const double half_span = horizontal ? config_.half_w : config_.half_h;
          const double center = std::clamp(horizontal ? mid_x : mid_y, -half_span, half_span);

          for (int dir : {1, -1})
          {
            if (dir == -1 && lateral_idx == 0)
            {
              continue;
            }
            const double along = center + dir * lateral_idx * config_.sample_step;
            if (along < -half_span || along > half_span)
            {
              continue;
            }

            const BasePose base = compute_edge_base_pose(
                edge, along, clearance, base_z, config_.half_w, config_.half_h);

            bool reachable = true;
            for (const auto &pt : targets)
            {
              if (std::hypot(pt.x - base.x, pt.y - base.y, pt.z - base.z) > reach)
              {
                reachable = false;
                break;
              }
            }

            if (reachable)
            {
              candidates.push_back(base);
              if (static_cast<int>(candidates.size()) >= config_.max_samples)
              {
                return candidates;
              }
            }
          }
        }
      }
      if (shell * config_.sample_step > reach + 2.0 * std::max(config_.half_w, config_.half_h))
      {
        break;
      }
    }
    return candidates;
  }

  std::vector<BasePose> WorkspacePlanner::generate_standoff_candidates(
      const Point3D &pick,
      const Point3D &place,
      double base_z) const
  {
    return generate_standoff_candidates(std::vector<Point3D>{pick, place}, base_z);
  }

  std::vector<BasePose> WorkspacePlanner::generate_standoff_candidates_3(
      const Point3D &p1,
      const Point3D &p2,
      const Point3D &p3,
      double base_z) const
  {
    return generate_standoff_candidates(std::vector<Point3D>{p1, p2, p3}, base_z);
  }

  PlanResult WorkspacePlanner::plan(
      const PlanningRequest &req,
      const PlanningContext &ctx,
      double base_z) const
  {
    if (!config_.is_valid() || !req.is_valid() || !std::isfinite(base_z) ||
        (ctx.current_base && !ctx.current_base->is_finite()) || !model_ || model_->chain.empty() || !solver_)
    {
      PlanResult result;
      result.feasible = false;
      result.status = FeasibilityStatus::MALFORMED_REQUEST;
      result.message = "Invalid planning input or uninitialized model";
      return result;
    }

    return req.is_capture ? plan_capture(req, ctx, base_z) : plan_quiet(req, ctx, base_z);
  }

  PlanResult WorkspacePlanner::plan_quiet(
      const PlanningRequest &req,
      const PlanningContext &ctx,
      double base_z) const
  {
    PlanResult result;
    int evaluated = 0;

    // Tier 0: Current Base (Zero-Nav)
    if (ctx.current_base)
    {
      ++evaluated;
      auto zero = try_single_pose(req, *ctx.current_base, PLAN_ZERO_NAV);
      if (zero)
      {
        zero->status = FeasibilityStatus::SUCCESS;
        zero->evaluated_candidates = evaluated;
        zero->clear_base = zero->pick_base;
        return *zero;
      }
    }

    // Tier 1: Single Standoff Base
    const int remaining = config_.max_samples - evaluated;
    const int single_limit = evaluated + (remaining <= 2 ? remaining : remaining / 2);
    for (const auto &base : generate_standoff_candidates(req.pick, req.place, base_z))
    {
      if (evaluated >= single_limit)
      {
        break;
      }
      ++evaluated;
      auto single = try_single_pose(req, base, PLAN_SINGLE_BASE);
      if (single)
      {
        single->status = FeasibilityStatus::SUCCESS;
        single->evaluated_candidates = evaluated;
        single->clear_base = single->pick_base;
        return *single;
      }
    }

    // Tier 2: Dual Standoff Bases
    const int pick_limit = evaluated + (config_.max_samples - evaluated) / 2;
    auto pick = find_endpoint(req.pick, req.pitch, base_z, pick_limit, evaluated);
    if (pick)
    {
      auto place = find_endpoint(req.place, req.pitch, base_z, config_.max_samples, evaluated);
      if (place)
      {
        result.feasible = true;
        result.status = FeasibilityStatus::SUCCESS;
        result.plan_type = PLAN_DUAL_BASE;
        result.clear_base = pick->first;
        result.pick_base = pick->first;
        result.place_base = place->first;
        result.pick_joints = pick->second.joints;
        result.place_joints = place->second.joints;
        result.message = "DUAL_BASE";
        result.evaluated_candidates = evaluated;
        return result;
      }
    }

    result.evaluated_candidates = evaluated;
    result.feasible = false;
    result.status = FeasibilityStatus::BASE_STANDOFF_EXHAUSTED;
    result.message = "No feasible base candidate within search budget";
    return result;
  }

  PlanResult WorkspacePlanner::plan_capture(
      const PlanningRequest &req,
      const PlanningContext &ctx,
      double base_z) const
  {
    PlanResult result;
    int evaluated = 0;

    // Tier 0: Current Base reaches all 3 points (Zero-Nav)
    if (ctx.current_base)
    {
      ++evaluated;
      auto zero = try_single_pose_capture(req, *ctx.current_base, PLAN_CAPTURE_ZERO_NAV);
      if (zero)
      {
        zero->status = FeasibilityStatus::SUCCESS;
        zero->evaluated_candidates = evaluated;
        return *zero;
      }
    }

    // Tier 1: Single Standoff Base covering all 3 points
    const int remaining = config_.max_samples - evaluated;
    const int single_limit = evaluated + (remaining <= 2 ? remaining : remaining / 2);
    for (const auto &base : generate_standoff_candidates_3(req.clear, req.pick, req.place, base_z))
    {
      if (evaluated >= single_limit)
      {
        break;
      }
      ++evaluated;
      auto single = try_single_pose_capture(req, base, PLAN_CAPTURE_SINGLE_BASE);
      if (single)
      {
        single->status = FeasibilityStatus::SUCCESS;
        single->evaluated_candidates = evaluated;
        return *single;
      }
    }

    // Tier 2 & 3: Multi-base planning
    auto clear_ep = resolve_clear_endpoint(req, ctx, base_z, evaluated);
    if (!clear_ep)
    {
      result.evaluated_candidates = evaluated;
      result.status = FeasibilityStatus::BASE_STANDOFF_EXHAUSTED;
      result.message = "Cannot find reachable standoff for captured piece";
      return result;
    }

    auto pick_ep = resolve_pick_endpoint(req, clear_ep->first, base_z, evaluated);
    if (!pick_ep)
    {
      result.evaluated_candidates = evaluated;
      result.status = FeasibilityStatus::BASE_STANDOFF_EXHAUSTED;
      result.message = "Cannot find reachable standoff for pick square";
      return result;
    }

    auto place_ep = resolve_place_endpoint(req, clear_ep->first, pick_ep->first, base_z, evaluated);
    if (!place_ep)
    {
      result.evaluated_candidates = evaluated;
      result.status = FeasibilityStatus::BASE_STANDOFF_EXHAUSTED;
      result.message = "Cannot find reachable standoff for place square";
      return result;
    }

    return build_capture_plan_result(*clear_ep, *pick_ep, *place_ep, ctx, evaluated);
  }

  std::optional<WorkspacePlanner::EndpointSolution> WorkspacePlanner::resolve_clear_endpoint(
      const PlanningRequest &req,
      const PlanningContext &ctx,
      double base_z,
      int &evaluated) const
  {
    if (ctx.current_base)
    {
      auto cur_clear = solve_at_base(req.clear, *ctx.current_base, req.pitch);
      if (cur_clear.success)
      {
        return std::make_pair(*ctx.current_base, cur_clear);
      }
    }
    const int clear_limit = evaluated + (config_.max_samples - evaluated) / 3;
    return find_endpoint(req.clear, req.pitch, base_z, clear_limit, evaluated);
  }

  std::optional<WorkspacePlanner::EndpointSolution> WorkspacePlanner::resolve_pick_endpoint(
      const PlanningRequest &req,
      const BasePose &clear_base,
      double base_z,
      int &evaluated) const
  {
    auto pick_at_clear = solve_at_base(req.pick, clear_base, req.pitch);
    if (pick_at_clear.success)
    {
      return std::make_pair(clear_base, pick_at_clear);
    }
    const int pick_limit = evaluated + (config_.max_samples - evaluated) / 2;
    return find_endpoint(req.pick, req.pitch, base_z, pick_limit, evaluated);
  }

  std::optional<WorkspacePlanner::EndpointSolution> WorkspacePlanner::resolve_place_endpoint(
      const PlanningRequest &req,
      const BasePose &clear_base,
      const BasePose &pick_base,
      double base_z,
      int &evaluated) const
  {
    const bool place_matches_clear = (std::abs(req.place.x - req.clear.x) < 1e-4 &&
                                      std::abs(req.place.y - req.clear.y) < 1e-4 &&
                                      std::abs(req.place.z - req.clear.z) < 1e-4);
    if (place_matches_clear)
    {
      auto place_at_clear = solve_at_base(req.place, clear_base, req.pitch);
      if (place_at_clear.success)
      {
        return std::make_pair(clear_base, place_at_clear);
      }
    }
    else
    {
      auto place_at_pick = solve_at_base(req.place, pick_base, req.pitch);
      if (place_at_pick.success)
      {
        return std::make_pair(pick_base, place_at_pick);
      }
      auto place_at_clear = solve_at_base(req.place, clear_base, req.pitch);
      if (place_at_clear.success)
      {
        return std::make_pair(clear_base, place_at_clear);
      }
    }

    return find_endpoint(req.place, req.pitch, base_z, config_.max_samples, evaluated);
  }

  PlanResult WorkspacePlanner::build_capture_plan_result(
      const EndpointSolution &clear_ep,
      const EndpointSolution &pick_ep,
      const EndpointSolution &place_ep,
      const PlanningContext &ctx,
      int evaluated) const
  {
    PlanResult result;
    result.feasible = true;
    result.status = FeasibilityStatus::SUCCESS;
    result.clear_base = clear_ep.first;
    result.pick_base = pick_ep.first;
    result.place_base = place_ep.first;
    result.clear_joints = clear_ep.second.joints;
    result.pick_joints = pick_ep.second.joints;
    result.place_joints = place_ep.second.joints;
    result.evaluated_candidates = evaluated;

    const bool robot_already_at_clear = ctx.current_base.has_value() &&
                                        (std::hypot(ctx.current_base->x - clear_ep.first.x, ctx.current_base->y - clear_ep.first.y) < 0.05);

    const bool clear_matches_pick =
        (std::hypot(clear_ep.first.x - pick_ep.first.x, clear_ep.first.y - pick_ep.first.y) < 0.05);

    if (robot_already_at_clear || clear_matches_pick)
    {
      result.plan_type = PLAN_CAPTURE_DUAL_BASE;
      result.message = "CAPTURE_DUAL_BASE";
    }
    else
    {
      result.plan_type = PLAN_CAPTURE_TRIPLE_BASE;
      result.message = "CAPTURE_TRIPLE_BASE";
    }
    return result;
  }

} // namespace lekiwi_motion::workspace

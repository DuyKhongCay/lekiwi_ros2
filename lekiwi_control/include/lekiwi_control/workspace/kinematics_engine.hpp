// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_CONTROL__WORKSPACE__KINEMATICS_ENGINE_HPP_
#define LEKIWI_CONTROL__WORKSPACE__KINEMATICS_ENGINE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "lekiwi_control/workspace/types.hpp"

namespace lekiwi_control::workspace
{

  inline std::tuple<double, double, double, double> forward_kinematics_2d(
      double q1, double q2, double q3, double q4, const KinematicsConfig &config)
  {
    double a1 = q2;
    double a2 = q2 + q3;
    double a3 = q2 + q3 + q4;

    double radius = config.link_lengths[0] * std::cos(a1) +
                    config.link_lengths[1] * std::cos(a2) +
                    config.link_lengths[2] * std::cos(a3);
    double height = config.link_lengths[0] * std::sin(a1) +
                    config.link_lengths[1] * std::sin(a2) +
                    config.link_lengths[2] * std::sin(a3);

    double x = config.base_offset[0] + radius * std::cos(q1);
    double y = config.base_offset[1] + radius * std::sin(q1);
    double z = config.base_offset[2] + height;

    return {x, y, z, a3};
  }

  inline std::pair<bool, std::string> check_joint_limits(
      const std::array<double, 5> &joints, const KinematicsConfig &config)
  {
    for (size_t i = 0; i < 5; ++i)
    {
      double angle = joints[i];
      if (!std::isfinite(angle) || angle < config.lower_limits[i] || angle > config.upper_limits[i])
      {
        return {
            false,
            "Joint " + config.joint_names[i] + " value " + std::to_string(angle) +
                " exceeds limits [" + std::to_string(config.lower_limits[i]) + ", " +
                std::to_string(config.upper_limits[i]) + "]"};
      }
    }
    return {true, ""};
  }

  inline IkResult solve_analytical_ik(
      double target_x, double target_y, double target_z,
      const KinematicsConfig &config, double required_pitch, double target_roll)
  {
    IkResult result;
    if (!std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(target_z) ||
        !std::isfinite(required_pitch) || !std::isfinite(target_roll))
    {
      result.radius = std::numeric_limits<double>::infinity();
      result.reason = "IK input must be finite";
      return result;
    }

    double dx = target_x - config.base_offset[0];
    double dy = target_y - config.base_offset[1];
    double dz = target_z - config.base_offset[2];
    double radius = std::hypot(dx, dy);
    result.radius = radius;

    double q1 = std::atan2(dy, dx);
    if (q1 < config.lower_limits[0] || q1 > config.upper_limits[0])
    {
      result.reason = "Target yaw exceeds shoulder_pan limits";
      return result;
    }

    double l1 = config.link_lengths[0];
    double l2 = config.link_lengths[1];
    double l3 = config.link_lengths[2];

    double rw = radius - l3 * std::cos(required_pitch);
    double zw = dz - l3 * std::sin(required_pitch);
    double wrist_dist = std::hypot(rw, zw);

    if (wrist_dist < 1e-8)
    {
      result.reason = "Wrist center too close to shoulder singularity";
      return result;
    }
    if (wrist_dist > l1 + l2 || wrist_dist < std::abs(l1 - l2))
    {
      result.reason = "Target geometrically unreachable";
      return result;
    }

    double cos_q3 = (wrist_dist * wrist_dist - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    cos_q3 = std::max(-1.0, std::min(1.0, cos_q3));
    double q3 = -std::acos(cos_q3); // elbow-up branch
    double q2 = std::atan2(zw, rw) - std::atan2(l2 * std::sin(q3), l1 + l2 * std::cos(q3));
    double q4 = required_pitch - q2 - q3;

    result.joints = {q1, q2, q3, q4, target_roll};
    auto [valid, reason] = check_joint_limits(result.joints, config);
    if (!valid)
    {
      result.reason = "Kinematic solution violates limits: " + reason;
      return result;
    }

    result.success = true;
    result.reason = "REACHABLE (Elbow-Up)";
    return result;
  }

  inline std::pair<BasePose, std::string> compute_standoff_pose(
      double target_x, double target_y, double base_z, const WorkspaceConfig &config)
  {
    double half_w = config.half_w;
    double half_h = config.half_h;

    double south_dist = target_y + half_h;
    double north_dist = half_h - target_y;
    double west_dist = target_x + half_w;
    double east_dist = half_w - target_x;

    std::string edge = "SOUTH";
    double min_dist = south_dist;
    if (north_dist < min_dist)
    {
      min_dist = north_dist;
      edge = "NORTH";
    }
    if (west_dist < min_dist)
    {
      min_dist = west_dist;
      edge = "WEST";
    }
    if (east_dist < min_dist)
    {
      min_dist = east_dist;
      edge = "EAST";
    }

    constexpr double nominal_reach = 0.245;
    double standoff_dist = std::max(config.edge_clearance, nominal_reach - min_dist);
    double margin_x = half_w + standoff_dist;
    double margin_y = half_h + standoff_dist;

    constexpr double corner_margin = 0.05;
    double clamp_x_bound = std::max(0.0, half_w - corner_margin);
    double clamp_y_bound = std::max(0.0, half_h - corner_margin);
    double x = std::max(-clamp_x_bound, std::min(target_x, clamp_x_bound));
    double y = std::max(-clamp_y_bound, std::min(target_y, clamp_y_bound));

    BasePose pose;
    pose.z = base_z;

    if (edge == "SOUTH")
    {
      pose.x = x;
      pose.y = -margin_y;
      pose.yaw = M_PI_2;
    }
    else if (edge == "NORTH")
    {
      pose.x = x;
      pose.y = margin_y;
      pose.yaw = -M_PI_2;
    }
    else if (edge == "WEST")
    {
      pose.x = -margin_x;
      pose.y = y;
      pose.yaw = 0.0;
    }
    else
    { // EAST
      pose.x = margin_x;
      pose.y = y;
      pose.yaw = M_PI;
    }

    return {pose, edge};
  }

  inline bool is_single_base_geometrically_possible(
      const Point3D &pick, const Point3D &place, const KinematicsConfig &config)
  {
    double dx = pick.x - place.x;
    double dy = pick.y - place.y;
    double dz = pick.z - place.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double max_reach = config.link_lengths[0] + config.link_lengths[1] + config.link_lengths[2];
    return dist <= 2.0 * max_reach;
  }

  inline Point3D transform_point_to_base_frame(const Point3D &pt, const BasePose &base)
  {
    double dx = pt.x - base.x;
    double dy = pt.y - base.y;
    double c = std::cos(base.yaw);
    double s = std::sin(base.yaw);
    return Point3D{c * dx + s * dy, -s * dx + c * dy, pt.z - base.z};
  }

  inline std::vector<BasePose> generate_standoff_candidates(
      const Point3D &pick, const Point3D &place, double base_z, const WorkspaceConfig &config)
  {
    double mid_x = 0.5 * (pick.x + place.x);
    double mid_y = 0.5 * (pick.y + place.y);
    auto [base, edge] = compute_standoff_pose(mid_x, mid_y, base_z, config);
    bool horizontal = (edge == "SOUTH" || edge == "NORTH");
    double half_span = horizontal ? config.half_w : config.half_h;
    double center = horizontal ? mid_x : mid_y;

    std::vector<BasePose> candidates;
    candidates.reserve(config.max_samples);

    for (int index = 0; index < config.max_samples; ++index)
    {
      double offset = ((index + 1) / 2) * config.sample_step * ((index % 2 != 0) ? 1.0 : -1.0);
      double value = std::max(-half_span, std::min(center + offset, half_span));

      // Avoid duplicate clamped values at bounds
      bool duplicate = false;
      for (const auto &c : candidates)
      {
        double existing_val = horizontal ? c.x : c.y;
        if (std::abs(existing_val - value) < 1e-5)
        {
          duplicate = true;
          break;
        }
      }
      if (duplicate)
      {
        continue;
      }

      candidates.push_back(BasePose{
          horizontal ? value : base.x,
          horizontal ? base.y : value,
          base.z,
          base.yaw});
    }
    return candidates;
  }

} // namespace lekiwi_control::workspace

#endif // LEKIWI_CONTROL__WORKSPACE__KINEMATICS_ENGINE_HPP_

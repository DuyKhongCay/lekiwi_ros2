// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_CONTROL__WORKSPACE__TYPES_HPP_
#define LEKIWI_CONTROL__WORKSPACE__TYPES_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lekiwi_control::workspace
{

  struct Point3D
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr bool is_finite() const noexcept
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
  };

  struct BasePose
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};

    constexpr bool is_finite() const noexcept
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(yaw);
    }
  };

  struct KinematicsConfig
  {
    std::array<double, 3> base_offset{0.0, 0.0, 0.0};
    std::array<double, 3> link_lengths{0.0, 0.0, 0.0};
    std::vector<std::string> joint_names;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;

    bool is_valid() const noexcept
    {
      if (joint_names.size() != 5 || lower_limits.size() != 5 || upper_limits.size() != 5)
      {
        return false;
      }
      for (double len : link_lengths)
      {
        if (!std::isfinite(len) || len <= 0.0)
        {
          return false;
        }
      }
      for (double off : base_offset)
      {
        if (!std::isfinite(off))
        {
          return false;
        }
      }
      for (size_t i = 0; i < 5; ++i)
      {
        if (!std::isfinite(lower_limits[i]) || !std::isfinite(upper_limits[i]) ||
            lower_limits[i] >= upper_limits[i])
        {
          return false;
        }
      }
      return true;
    }
  };

  struct WorkspaceConfig
  {
    double half_w{0.195};
    double half_h{0.195};
    double edge_clearance{0.085};
    double sample_step{0.025};
    int max_samples{25};
    double default_pitch{-1.57079632679};
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

  struct PlanningRequest
  {
    Point3D pick;
    Point3D place;
    double pitch{-1.57079632679};
    bool is_capture{false};

    bool is_valid() const noexcept
    {
      if (!pick.is_finite() || !std::isfinite(pitch))
      {
        return false;
      }
      if (!is_capture && !place.is_finite())
      {
        return false;
      }
      return true;
    }
  };

  struct PlanningContext
  {
    std::optional<BasePose> current_base{std::nullopt};
  };

  struct IkResult
  {
    bool success{false};
    std::array<double, 5> joints{0.0, 0.0, 0.0, 0.0, 0.0};
    double radius{0.0};
    std::string reason;
  };

  struct PlanResult
  {
    bool feasible{false};
    uint8_t plan_type{0};
    BasePose pick_base{};
    BasePose place_base{};
    std::array<double, 5> pick_joints{0.0, 0.0, 0.0, 0.0, 0.0};
    std::optional<std::array<double, 5>> place_joints{std::nullopt};
    std::string message;
  };

} // namespace lekiwi_control::workspace

#endif // LEKIWI_CONTROL__WORKSPACE__TYPES_HPP_

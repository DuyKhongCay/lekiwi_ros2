// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

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

  // 3D Cartesian point with finite coordinate checking
  struct Point3D
  {
    double x{0.0}, y{0.0}, z{0.0};
    bool is_finite() const noexcept
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
  };

  // 2D Mobile-base pose in horizontal plane (board frame)
  struct BasePose
  {
    double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
    bool is_finite() const noexcept
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(yaw);
    }
  };

  // 5-DoF Arm IK Solution
  struct IkResult
  {
    bool success{false};
    std::array<double, 5> joints{};
    double radius{0.0};
    std::string reason;
  };

  // Retained URDF segment for forward kinematics verification
  struct ChainSegment
  {
    std::string name;
    Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    int joint_index{-1};
  };

  // Pure URDF kinematic representation (links, joints, limits, frames)
  struct KinematicsModel
  {
    std::string base_frame;
    std::string tip_frame;
    std::vector<ChainSegment> chain;
    std::array<std::string, 5> joint_names{};
    std::array<double, 5> lower_limits{};
    std::array<double, 5> upper_limits{};
    double reach_bound{0.0};
  };

  // Forward kinematics evaluated directly across the URDF chain
  Eigen::Isometry3d forward_kinematics(
      const KinematicsModel &model,
      const std::array<double, 5> &joints);

  // ================= Inverse Kinematics Solvers (Strategy Pattern) =================

  // Generic IK Solver Interface (Open-Closed Principle)
  class IIkSolver
  {
  public:
    virtual ~IIkSolver() = default;
    virtual IkResult solve(double x, double y, double z, double pitch, double roll) const = 0;
    virtual double reach_bound() const = 0;
  };

  // SO-101 5-DoF Closed-form Analytical IK Solver
  class SO101AnalyticalSolver : public IIkSolver
  {
  public:
    explicit SO101AnalyticalSolver(const KinematicsModel &model);

    IkResult solve(double x, double y, double z, double pitch, double roll) const override;
    double reach_bound() const override { return reach_bound_; }

    const std::array<double, 2> &link_lengths() const noexcept { return link_lengths_; }
    const std::array<double, 2> &link_angles() const noexcept { return link_angles_; }

  private:
    void init_geometry(const KinematicsModel &model);

    KinematicsModel model_;
    std::array<double, 5> signs_{};
    Eigen::Vector3d pan_origin_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d plane_basis_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d tool_basis_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d shoulder_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d first_link_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d second_link_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d wrist_origin_{Eigen::Vector3d::Zero()};
    std::array<double, 2> link_lengths_{};
    std::array<double, 2> link_angles_{};
    double yaw_offset_{0.0};
    double tool_pitch_offset_{0.0};
    double tool_roll_offset_{0.0};
    double reach_bound_{0.0};
  };

  // ================= Core Utilities =================

  // Extract kinematic parameters directly from URDF model into KinematicsModel
  bool extract_kinematics_from_urdf(
      const urdf::ModelInterface &urdf,
      const std::string &base_frame,
      const std::string &tip_frame,
      const std::vector<std::string> &joint_names,
      KinematicsModel &out_model,
      std::string &error_msg,
      double safety_margin_rad = 0.05);

  // Coordinate projection from board frame into planar base frame
  Point3D transform_point_to_base_frame(
      const Point3D &pt,
      const BasePose &base);

} // namespace lekiwi_motion::workspace

#endif // LEKIWI_MOTION__WORKSPACE_KINEMATICS_HPP_

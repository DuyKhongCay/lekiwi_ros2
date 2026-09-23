// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_CONTROL__WORKSPACE_KINEMATICS_HPP_
#define LEKIWI_CONTROL__WORKSPACE_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <urdf_model/model.h>

namespace lekiwi_control::workspace
{

  // Plan types matching service contract
  constexpr uint8_t PLAN_ZERO_NAV = 0;
  constexpr uint8_t PLAN_SINGLE_BASE = 1;
  constexpr uint8_t PLAN_DUAL_BASE = 2;

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
    Point3D pick;
    Point3D place;
    double pitch{-M_PI_2};
    bool is_capture{false};

    bool is_valid() const noexcept
    {
      return pick.is_finite() && std::isfinite(pitch) && (is_capture || place.is_finite());
    }
  };

  // Planning context (current mobile base pose if available from TF)
  struct PlanningContext
  {
    std::optional<BasePose> current_base;
  };

  // 5-DoF Arm IK Solution
  struct IkResult
  {
    bool success{false};
    std::array<double, 5> joints{};
    double radius{0.0};
    std::string reason;
  };

  // Complete move plan result
  struct PlanResult
  {
    bool feasible{false};
    uint8_t plan_type{0};
    BasePose pick_base;
    BasePose place_base;
    std::array<double, 5> pick_joints{};
    std::optional<std::array<double, 5>> place_joints;
    std::string message;
    int evaluated_candidates{0};
  };

  // Retained URDF segment for forward kinematics verification
  struct ChainSegment
  {
    std::string name;
    Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    int joint_index{-1};
  };

  // Extracted SO-101 analytical kinematics representation
  struct KinematicsModel
  {
    std::string base_frame;
    std::string tip_frame;
    std::vector<ChainSegment> chain;
    std::array<std::string, 5> joint_names;
    std::array<double, 5> lower_limits{};
    std::array<double, 5> upper_limits{};
    std::array<double, 5> signs{};
    Eigen::Vector3d pan_origin{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d plane_basis{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d tool_basis{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d shoulder{Eigen::Vector3d::Zero()};
    Eigen::Vector3d first_link{Eigen::Vector3d::Zero()};
    Eigen::Vector3d second_link{Eigen::Vector3d::Zero()};
    Eigen::Vector3d wrist_origin{Eigen::Vector3d::Zero()};
    std::array<double, 2> link_lengths{};
    std::array<double, 2> link_angles{};
    double yaw_offset{0.0};
    double tool_pitch_offset{0.0};
    double tool_roll_offset{0.0};
    double reach_bound{0.0};
  };

  // ================= Core API Declarations =================

  // Extract SO-101 5-DoF kinematic parameters directly from URDF model
  bool extract_kinematics_from_urdf(
      const urdf::ModelInterface &urdf,
      const std::string &base_frame,
      const std::string &tip_frame,
      const std::vector<std::string> &joint_names,
      KinematicsModel &out_model,
      std::string &error_msg,
      double safety_margin_rad = 0.05);

  // Forward kinematics evaluated directly across the URDF chain
  Eigen::Isometry3d forward_kinematics(
      const KinematicsModel &model,
      const std::array<double, 5> &joints);

  // Closed-form analytical inverse kinematics with FK verification
  IkResult solve_analytical_ik(
      double x, double y, double z,
      const KinematicsModel &model,
      double pitch = -M_PI_2,
      double roll = 0.0);

  // Coordinate projection from board frame into planar base frame
  Point3D transform_point_to_base_frame(
      const Point3D &pt,
      const BasePose &base);

  // Generate candidate base poses around board perimeter sorted by target proximity
  std::vector<BasePose> generate_standoff_candidates(
      const Point3D &pick,
      const Point3D &place,
      double base_z,
      const KinematicsModel &model,
      const WorkspaceConfig &config);

  // 3-Tier Move Planning (Zero-Nav -> Single-Base -> Dual-Base)
  PlanResult plan_move(
      const PlanningRequest &req,
      const PlanningContext &ctx,
      double base_z,
      const KinematicsModel &model,
      const WorkspaceConfig &workspace);

} // namespace lekiwi_control::workspace

#endif // LEKIWI_CONTROL__WORKSPACE_KINEMATICS_HPP_

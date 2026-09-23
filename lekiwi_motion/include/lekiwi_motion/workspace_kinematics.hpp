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

  // ================= Pure Domain Service =================

  // Domain service for mobile standoff candidate search and 3-tier move planning
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

    std::vector<BasePose> generate_standoff_candidates(
        const Point3D &pick,
        const Point3D &place,
        double base_z) const;

    const WorkspaceConfig &config() const noexcept { return config_; }
    const KinematicsModel &model() const noexcept { return *model_; }
    const IIkSolver &solver() const noexcept { return *solver_; }

  private:
    IkResult solve_at_base(
        const Point3D &pt,
        const BasePose &base,
        double pitch) const;

    std::optional<PlanResult> try_single_pose(
        const PlanningRequest &req,
        const BasePose &base,
        uint8_t plan_type) const;

    std::optional<std::pair<BasePose, IkResult>> find_endpoint(
        const Point3D &pt,
        double pitch,
        double base_z,
        int limit,
        int &evaluated) const;

    std::shared_ptr<const KinematicsModel> model_;
    std::shared_ptr<const IIkSolver> solver_;
    WorkspaceConfig config_;
  };

  // ================= Core Utilities & Backward-Compatible API =================

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

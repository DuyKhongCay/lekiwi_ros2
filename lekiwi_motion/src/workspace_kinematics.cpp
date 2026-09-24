// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/workspace_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace lekiwi_motion::workspace
{

  namespace
  {
    // Internal convergence and solver tolerances
    constexpr double AXIS_TOLERANCE = 1e-4;
    constexpr double POSITION_TOLERANCE = 1e-4;
    constexpr double ORIENTATION_TOLERANCE = 1e-3;

    // Default TCP approach direction and up axis for SO-101 gripper in TCP frame
    const Eigen::Vector3d DEFAULT_APPROACH_AXIS{0.0, 1.0, 0.0};
    const Eigen::Vector3d DEFAULT_UP_AXIS{0.0, 0.0, 1.0};

    // Map joint angle into URDF joint limits considering 2*pi periodicity
    std::optional<double> fit_joint_angle(double angle, double lower, double upper)
    {
      if (!std::isfinite(angle) || !std::isfinite(lower) || !std::isfinite(upper) || lower >= upper)
      {
        return std::nullopt;
      }
      if (angle >= lower - 1e-12 && angle <= upper + 1e-12)
      {
        return std::clamp(angle, lower, upper);
      }
      const double turn = 2.0 * M_PI;
      double value = angle + turn * std::ceil((lower - angle) / turn);
      if (value > upper + 1e-12)
      {
        return std::nullopt;
      }
      return std::clamp(value, lower, upper);
    }

    // Compute target tool rotation matrix from yaw, pitch, roll
    Eigen::Matrix3d target_tool_rotation(double yaw, double pitch, double roll)
    {
      return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
    }

    // Helper: Traverse URDF backwards from tip to base to collect joints
    bool build_chain_segments(
        const urdf::ModelInterface &urdf,
        const std::string &base_frame,
        const std::string &tip_frame,
        const std::vector<std::string> &joint_names,
        double safety_margin_rad,
        KinematicsModel &model,
        std::string &error_msg)
    {
      if (joint_names.size() != 5 || std::set<std::string>(joint_names.begin(), joint_names.end()).size() != 5)
      {
        error_msg = "Exactly five distinct arm joints required";
        return false;
      }

      auto link = urdf.getLink(tip_frame);
      if (!link || !urdf.getLink(base_frame))
      {
        error_msg = "Missing base or tip link in URDF: " + base_frame + " -> " + tip_frame;
        return false;
      }

      std::vector<urdf::JointConstSharedPtr> reversed;
      while (link && link->name != base_frame)
      {
        if (!link->parent_joint)
        {
          error_msg = "Tip " + tip_frame + " is not a descendant of " + base_frame;
          return false;
        }
        reversed.push_back(link->parent_joint);
        link = urdf.getLink(link->parent_joint->parent_link_name);
      }
      if (!link)
      {
        error_msg = "Disconnected chain from " + tip_frame + " to " + base_frame;
        return false;
      }
      std::reverse(reversed.begin(), reversed.end());

      size_t joint_idx = 0;

      for (const auto &joint : reversed)
      {
        ChainSegment segment;
        segment.name = joint->name;
        const auto &p = joint->parent_to_joint_origin_transform.position;
        const auto &r = joint->parent_to_joint_origin_transform.rotation;
        Eigen::Quaterniond rot(r.w, r.x, r.y, r.z);

        segment.origin.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        segment.origin.linear() = rot.normalized().toRotationMatrix();

        if (joint->type != urdf::Joint::FIXED)
        {
          if (joint->type != urdf::Joint::REVOLUTE || joint->mimic)
          {
            error_msg = "Joint must be independent revolute: " + joint->name;
            return false;
          }
          if (joint_idx >= 5 || joint->name != joint_names[joint_idx])
          {
            error_msg = "Unexpected joint or wrong order: " + joint->name;
            return false;
          }
          segment.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z).normalized();

          if (!joint->limits || joint->limits->upper - joint->limits->lower <= 2.0 * safety_margin_rad)
          {
            error_msg = "Invalid or exhausted joint limits at " + joint->name;
            return false;
          }

          model.joint_names[joint_idx] = joint->name;
          model.lower_limits[joint_idx] = joint->limits->lower + safety_margin_rad;
          model.upper_limits[joint_idx] = joint->limits->upper - safety_margin_rad;
          segment.joint_index = static_cast<int>(joint_idx++);
        }
        model.reach_bound += segment.origin.translation().norm();
        model.chain.push_back(segment);
      }

      if (joint_idx != 5)
      {
        error_msg = "Base-to-tip chain does not contain all five specified joints";
        return false;
      }
      return true;
    }

  } // namespace

  // ================= Implementation of Public API =================

  bool extract_kinematics_from_urdf(
      const urdf::ModelInterface &urdf,
      const std::string &base_frame,
      const std::string &tip_frame,
      const std::vector<std::string> &joint_names,
      KinematicsModel &out_model,
      std::string &error_msg,
      double safety_margin_rad)
  {
    out_model = KinematicsModel();
    out_model.base_frame = base_frame;
    out_model.tip_frame = tip_frame;

    return build_chain_segments(urdf, base_frame, tip_frame, joint_names, safety_margin_rad,
                                out_model, error_msg);
  }

  Eigen::Isometry3d forward_kinematics(
      const KinematicsModel &model,
      const std::array<double, 5> &joints)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (const auto &segment : model.chain)
    {
      pose = pose * segment.origin;
      if (segment.joint_index >= 0)
      {
        pose.rotate(Eigen::AngleAxisd(joints.at(segment.joint_index), segment.axis));
      }
    }
    return pose;
  }

  // ================= SO101AnalyticalSolver Implementation =================

  SO101AnalyticalSolver::SO101AnalyticalSolver(const KinematicsModel &model)
  {
    init_geometry(model);
  }

  void SO101AnalyticalSolver::init_geometry(const KinematicsModel &model)
  {
    model_ = model;

    tool_basis_.col(0) = DEFAULT_APPROACH_AXIS;
    tool_basis_.col(1) = DEFAULT_UP_AXIS.cross(DEFAULT_APPROACH_AXIS).normalized();
    tool_basis_.col(2) = DEFAULT_APPROACH_AXIS.cross(tool_basis_.col(1));

    if (model_.chain.empty() || model_.joint_names.size() != 5)
    {
      throw std::invalid_argument("SO101AnalyticalSolver requires an initialized 5-DoF KinematicsModel");
    }

    std::array<Eigen::Vector3d, 5> origins{};
    std::array<Eigen::Vector3d, 5> axes{};
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (const auto &segment : model_.chain)
    {
      pose = pose * segment.origin;
      if (segment.joint_index >= 0 && segment.joint_index < 5)
      {
        origins[segment.joint_index] = pose.translation();
        axes[segment.joint_index] = pose.linear() * segment.axis;
      }
    }

    if (axes[0].cross(Eigen::Vector3d::UnitZ()).norm() > AXIS_TOLERANCE ||
        std::abs(axes[1].z()) > AXIS_TOLERANCE)
    {
      throw std::invalid_argument("Pan joint must be vertical (Z) and shoulder lift horizontal");
    }

    Eigen::Isometry3d tip_pose = forward_kinematics(model_, {0.0, 0.0, 0.0, 0.0, 0.0});

    Eigen::Vector3d normal(axes[1].x(), axes[1].y(), 0.0);
    normal.normalize();
    if (normal.cross(Eigen::Vector3d::UnitZ()).dot(tip_pose.linear() * DEFAULT_APPROACH_AXIS) < 0.0)
    {
      normal = -normal;
    }

    plane_basis_.col(0) = normal.cross(Eigen::Vector3d::UnitZ());
    plane_basis_.col(1) = normal;
    plane_basis_.col(2) = Eigen::Vector3d::UnitZ();

    signs_[0] = axes[0].z() > 0.0 ? 1.0 : -1.0;
    for (size_t i = 1; i <= 3; ++i)
    {
      if (axes[i].cross(normal).norm() > AXIS_TOLERANCE)
      {
        throw std::invalid_argument("Pitch axis not parallel to shoulder at " + model_.joint_names[i]);
      }
      signs_[i] = axes[i].dot(normal) > 0.0 ? -1.0 : 1.0;
    }

    Eigen::Vector3d approach = tip_pose.linear() * DEFAULT_APPROACH_AXIS;
    signs_[4] = axes[4].dot(approach) > 0.0 ? 1.0 : -1.0;

    Eigen::Matrix3d tool_rot = plane_basis_.transpose() * tip_pose.linear() * tool_basis_;
    tool_pitch_offset_ = std::atan2(tool_rot(2, 0), tool_rot(0, 0));
    Eigen::Matrix3d roll_rot = Eigen::AngleAxisd(tool_pitch_offset_, Eigen::Vector3d::UnitY()) * tool_rot;
    tool_roll_offset_ = std::atan2(roll_rot(2, 1), roll_rot(1, 1));

    pan_origin_ = origins[0];
    wrist_origin_ = origins[3];
    shoulder_ = plane_basis_.transpose() * (origins[1] - origins[0]);
    first_link_ = plane_basis_.transpose() * (origins[2] - origins[1]);
    second_link_ = plane_basis_.transpose() * (origins[3] - origins[2]);

    const std::array<Eigen::Vector3d, 2> links{first_link_, second_link_};
    for (size_t i = 0; i < 2; ++i)
    {
      link_lengths_[i] = std::hypot(links[i].x(), links[i].z());
      link_angles_[i] = std::atan2(links[i].z(), links[i].x());
      if (link_lengths_[i] <= POSITION_TOLERANCE)
      {
        throw std::invalid_argument("Degenerate pitch link at " + model_.joint_names[i + 2]);
      }
    }
    yaw_offset_ = std::atan2(plane_basis_(1, 0), plane_basis_(0, 0));
    reach_bound_ = model_.reach_bound;
  }

  IkResult SO101AnalyticalSolver::solve(
      double x, double y, double z,
      double pitch,
      double roll) const
  {
    IkResult result;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(pitch) || !std::isfinite(roll))
    {
      result.reason = "Target coordinates must be finite";
      return result;
    }
    if (model_.chain.empty() || link_lengths_[0] <= 0.0 || link_lengths_[1] <= 0.0)
    {
      result.reason = "Kinematics model is uninitialized";
      return result;
    }

    auto q5 = fit_joint_angle((roll - tool_roll_offset_) / signs_[4],
                              model_.lower_limits[4], model_.upper_limits[4]);
    if (!q5)
    {
      result.reason = "Wrist roll out of joint limits";
      return result;
    }

    std::array<double, 5> roll_only{0.0, 0.0, 0.0, 0.0, *q5};
    const Eigen::Vector3d tool = plane_basis_.transpose() *
                                 (forward_kinematics(model_, roll_only).translation() - wrist_origin_);
    const double pitch_sum = pitch - tool_pitch_offset_;
    const Eigen::Vector3d rotated_tool = Eigen::AngleAxisd(-pitch_sum, Eigen::Vector3d::UnitY()) * tool;
    const double lateral = shoulder_.y() + first_link_.y() + second_link_.y() + rotated_tool.y();

    const Eigen::Vector3d target(x, y, z);
    const Eigen::Vector3d delta = target - pan_origin_;
    const double radius = std::hypot(delta.x(), delta.y());
    result.radius = radius;

    if (std::abs(lateral) > radius + 1e-12)
    {
      result.reason = "Target inside lateral offset singularity";
      return result;
    }

    const double phi = std::atan2(delta.y(), delta.x());
    const double side_angle = radius < 1e-12 ? 0.0 : std::asin(std::clamp(lateral / radius, -1.0, 1.0));
    std::array<double, 2> yaws{phi - side_angle, phi - M_PI + side_angle};

    result.reason = "No joint-limit and FK-valid analytical branch";
    for (double yaw : yaws)
    {
      auto q1 = fit_joint_angle((yaw - yaw_offset_) / signs_[0],
                                model_.lower_limits[0], model_.upper_limits[0]);
      if (!q1)
      {
        continue;
      }

      const Eigen::Vector3d local = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * delta;
      const double rw = local.x() - shoulder_.x() - rotated_tool.x();
      const double zw = local.z() - shoulder_.z() - rotated_tool.z();
      const double l1 = link_lengths_[0], l2 = link_lengths_[1];
      const double cos_elbow = (rw * rw + zw * zw - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);

      if (cos_elbow < -1.0 - 1e-12 || cos_elbow > 1.0 + 1e-12 || std::hypot(rw, zw) < 1e-12)
      {
        continue;
      }

      const double elbow = std::acos(std::clamp(cos_elbow, -1.0, 1.0));
      for (double bend : {-elbow, elbow})
      {
        const double first_angle = std::atan2(zw, rw) - std::atan2(l2 * std::sin(bend), l1 + l2 * std::cos(bend));
        std::array<double, 5> joints{
            *q1,
            (first_angle - link_angles_[0]) / signs_[1],
            (bend - link_angles_[1] + link_angles_[0]) / signs_[2],
            0.0,
            *q5};
        joints[3] = (pitch_sum - signs_[1] * joints[1] - signs_[2] * joints[2]) / signs_[3];

        bool valid = true;
        for (size_t i = 1; i < 4; ++i)
        {
          auto angle = fit_joint_angle(joints[i], model_.lower_limits[i], model_.upper_limits[i]);
          if (!angle)
          {
            valid = false;
            break;
          }
          joints[i] = *angle;
        }
        if (!valid)
        {
          continue;
        }

        const auto fk = forward_kinematics(model_, joints);
        const Eigen::Matrix3d desired = target_tool_rotation(yaw, pitch, roll) * tool_basis_.transpose();
        const double angle_error = Eigen::AngleAxisd(desired.transpose() * fk.linear()).angle();

        if ((fk.translation() - target).norm() > POSITION_TOLERANCE ||
            angle_error > ORIENTATION_TOLERANCE)
        {
          continue;
        }

        result.success = true;
        result.joints = joints;
        result.reason = "REACHABLE (FK verified)";
        return result;
      }
    }
    return result;
  }

  Point3D transform_point_to_base_frame(const Point3D &pt, const BasePose &base)
  {
    const double dx = pt.x - base.x, dy = pt.y - base.y;
    const double c = std::cos(base.yaw), s = std::sin(base.yaw);
    return {c * dx + s * dy, -s * dx + c * dy, pt.z - base.z};
  }

} // namespace lekiwi_motion::workspace

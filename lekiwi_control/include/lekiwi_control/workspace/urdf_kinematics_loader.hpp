// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_CONTROL__WORKSPACE__URDF_KINEMATICS_LOADER_HPP_
#define LEKIWI_CONTROL__WORKSPACE__URDF_KINEMATICS_LOADER_HPP_

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <urdf_model/model.h>

#include "lekiwi_control/workspace/types.hpp"

namespace lekiwi_control::workspace
{

  inline KinematicsConfig extract_kinematics_from_urdf(
      const urdf::ModelInterface &model,
      const std::vector<std::string> &joint_names = {
          "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
          "arm_wrist_flex", "arm_wrist_roll"},
      double safety_margin_rad = 0.05)
  {
    KinematicsConfig config;
    config.joint_names = joint_names;

    if (joint_names.size() != 5)
    {
      throw std::invalid_argument("URDF arm kinematics requires exactly 5 joint names");
    }

    // 1. Joint Limits
    for (const auto &name : joint_names)
    {
      auto joint = model.getJoint(name);
      if (!joint)
      {
        throw std::runtime_error("Joint not found in URDF: " + name);
      }
      if (!joint->limits)
      {
        throw std::runtime_error("Joint has no limits in URDF: " + name);
      }
      double lower = joint->limits->lower;
      double upper = joint->limits->upper;
      if (safety_margin_rad > 0.0 && (upper - lower) > 2.0 * safety_margin_rad)
      {
        lower += safety_margin_rad;
        upper -= safety_margin_rad;
      }
      config.lower_limits.push_back(lower);
      config.upper_limits.push_back(upper);
    }

    // 2. Base Offset (arm_shoulder_pan origin relative to base)
    auto pan_joint = model.getJoint(joint_names[0]);
    const auto &pan_pos = pan_joint->parent_to_joint_origin_transform.position;
    config.base_offset = {pan_pos.x, pan_pos.y, pan_pos.z};

    // 3. Link Lengths:
    // L1: distance along upper_arm to arm_elbow_flex
    auto elbow_joint = model.getJoint(joint_names[2]);
    const auto &elbow_pos = elbow_joint->parent_to_joint_origin_transform.position;
    double l1 = std::hypot(elbow_pos.x, elbow_pos.y, elbow_pos.z);

    // L2: distance along lower_arm to arm_wrist_flex
    auto wrist_flex_joint = model.getJoint(joint_names[3]);
    const auto &wrist_flex_pos = wrist_flex_joint->parent_to_joint_origin_transform.position;
    double l2 = std::hypot(wrist_flex_pos.x, wrist_flex_pos.y, wrist_flex_pos.z);

    // L3: wrist to end-effector center
    // Distance from wrist_flex origin to wrist_roll origin + gripper tip offset
    auto wrist_roll_joint = model.getJoint(joint_names[4]);
    double l3_wrist = 0.0;
    if (wrist_roll_joint)
    {
      const auto &roll_pos = wrist_roll_joint->parent_to_joint_origin_transform.position;
      l3_wrist = std::hypot(roll_pos.x, roll_pos.y, roll_pos.z);
    }
    // Standard gripper tool center distance: if gripper joint is found, calculate its offset
    auto gripper_joint = model.getJoint("arm_gripper");
    double l3_gripper = 0.04168; // nominal gripper TCP length if not specified
    if (gripper_joint)
    {
      const auto &grip_pos = gripper_joint->parent_to_joint_origin_transform.position;
      double d = std::hypot(grip_pos.x, grip_pos.y, grip_pos.z);
      if (d > 0.01)
      {
        l3_gripper = d;
      }
    }
    double l3 = l3_wrist + l3_gripper;
    if (std::abs(l3 - 0.105368) > 0.05)
    {
      // Keep nominal 0.105368 if URDF gripper structure differs
      l3 = 0.105368;
    }

    config.link_lengths = {l1, l2, l3};

    if (!config.is_valid())
    {
      throw std::runtime_error("Extracted kinematics config is invalid");
    }

    return config;
  }

  // Fallback nominal config matching LeKiwi arm specs when URDF is unavailable
  inline KinematicsConfig get_default_lekiwi_kinematics()
  {
    KinematicsConfig config;
    config.base_offset = {0.0461807, 0.0000127994, 0.1696};
    config.link_lengths = {0.115998, 0.135000, 0.105368};
    config.joint_names = {
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll"};
    config.lower_limits = {-1.74919, -1.74533, -1.65765, -1.56161, -2.64577};
    config.upper_limits = {1.74147, 1.74533, 1.57121, 1.57999, 2.76475};
    return config;
  }

} // namespace lekiwi_control::workspace

#endif // LEKIWI_CONTROL__WORKSPACE__URDF_KINEMATICS_LOADER_HPP_

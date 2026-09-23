// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#ifndef LEKIWI_CONTROL__TORQUE_COMMAND_STATE_HPP_
#define LEKIWI_CONTROL__TORQUE_COMMAND_STATE_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <lekiwi_interfaces/srv/set_torque_enabled.hpp>

namespace lekiwi_control
{
  // Tracks requested commands only; it never represents measured motor state.
  class TorqueCommandState
  {
  public:
    // Validate the complete controller ordering before accepting any commands.
    TorqueCommandState(std::vector<std::string> joints,
                       const std::vector<std::string> &arm, const std::vector<std::string> &base)
        : joints_(std::move(joints)), arm_(arm.begin(), arm.end()), base_(base.begin(), base.end())
    {
      std::set<std::string> all(arm_);
      all.insert(base_.begin(), base_.end());
      const std::set<std::string> ordered(joints_.begin(), joints_.end());
      if (arm.empty() || base.empty() || ordered.count("") ||
          ordered.size() != joints_.size() || all.size() != arm.size() + base.size() ||
          ordered != all)
      {
        throw std::invalid_argument(
            "Torque joint order must contain exactly the nonempty disjoint ARM and BASE groups");
      }
      states_ = std::vector<double>(joints_.size(), 1.0);
    }

    // Reject invalid target and forget assumptions when the controller is absent.
    std::vector<double> apply(uint8_t target, bool enabled, bool controller_available)
    {
      using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;
      if (target != Request::TARGET_ALL && target != Request::TARGET_ARM && target != Request::TARGET_BASE)
      {
        throw std::invalid_argument("Invalid target: " + std::to_string(target));
      }
      if (!controller_available)
      {
        reset();
        throw std::runtime_error("Torque controller subscriber unavailable; command state invalidated");
      }
      auto states = states_.value_or(std::vector<double>(joints_.size(), 1.0));
      const auto &group = target == Request::TARGET_ARM ? arm_ : base_;
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        if (target == Request::TARGET_ALL || group.count(joints_[i]))
        {
          states[i] = enabled ? 1.0 : 0.0;
        }
      }
      states_ = states;
      return states;
    }

    // Check if the target group is currently considered enabled.
    bool is_group_enabled(uint8_t target) const
    {
      using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;
      if (target != Request::TARGET_ALL && target != Request::TARGET_ARM && target != Request::TARGET_BASE)
      {
        throw std::invalid_argument("Invalid target: " + std::to_string(target));
      }
      const auto &states = states_.value_or(std::vector<double>(joints_.size(), 1.0));
      const auto &group = target == Request::TARGET_ARM ? arm_ : base_;
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        if (target == Request::TARGET_ALL || group.count(joints_[i]))
        {
          if (states[i] > 0.5)
          {
            return true;
          }
        }
      }
      return false;
    }

    // A failed submission must not leave a trusted command vector behind.
    void reset() { states_.reset(); }

  private:
    std::vector<std::string> joints_;
    std::set<std::string> arm_;
    std::set<std::string> base_;
    std::optional<std::vector<double>> states_;
  };
} // namespace lekiwi_control
#endif

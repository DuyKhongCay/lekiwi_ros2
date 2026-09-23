// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#ifndef LEKIWI_MOTION__TORQUE_MANAGER_NODE_HPP_
#define LEKIWI_MOTION__TORQUE_MANAGER_NODE_HPP_

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lekiwi_interfaces/srv/set_torque_enabled.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace lekiwi_motion
{
  // Pure domain logic for tracking requested torque commands with direct memory slicing.
  class TorqueCommandState
  {
  public:
    static constexpr double kTorqueEnabledValue = 1.0;
    static constexpr double kTorqueDisabledValue = 0.0;
    static constexpr double kTorqueStateThreshold = 0.5;

    TorqueCommandState() = default;

    TorqueCommandState(const std::vector<std::string> &arm,
                       const std::vector<std::string> &base)
        : arm_count_(arm.size()),
          total_count_(arm.size() + base.size()),
          states_(total_count_, kTorqueEnabledValue)
    {
      if (arm.empty() || base.empty())
      {
        throw std::invalid_argument("ARM and BASE joint groups must be nonempty");
      }
    }

    const std::vector<double> &apply(uint8_t target, bool enabled)
    {
      using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;
      if (target != Request::TARGET_ALL && target != Request::TARGET_ARM && target != Request::TARGET_BASE)
      {
        throw std::invalid_argument("Invalid target: " + std::to_string(target));
      }

      const double value = enabled ? kTorqueEnabledValue : kTorqueDisabledValue;
      if (target == Request::TARGET_ALL)
      {
        std::fill(states_.begin(), states_.end(), value);
      }
      else if (target == Request::TARGET_ARM)
      {
        std::fill(states_.begin(), states_.begin() + arm_count_, value);
      }
      else
      {
        std::fill(states_.begin() + arm_count_, states_.end(), value);
      }
      return states_;
    }

    [[nodiscard]] bool is_group_enabled(uint8_t target) const
    {
      using Request = lekiwi_interfaces::srv::SetTorqueEnabled::Request;
      if (target != Request::TARGET_ALL && target != Request::TARGET_ARM && target != Request::TARGET_BASE)
      {
        throw std::invalid_argument("Invalid target: " + std::to_string(target));
      }

      auto is_active = [](double val) noexcept
      { return val > kTorqueStateThreshold; };

      if (target == Request::TARGET_ALL)
      {
        return std::any_of(states_.begin(), states_.end(), is_active);
      }
      if (target == Request::TARGET_ARM)
      {
        return std::any_of(states_.begin(), states_.begin() + arm_count_, is_active);
      }
      return std::any_of(states_.begin() + arm_count_, states_.end(), is_active);
    }

    void reset() noexcept
    {
      std::fill(states_.begin(), states_.end(), kTorqueEnabledValue);
    }

    [[nodiscard]] size_t arm_count() const noexcept { return arm_count_; }
    [[nodiscard]] size_t total_count() const noexcept { return total_count_; }
    [[nodiscard]] const std::vector<double> &current_states() const noexcept { return states_; }

  private:
    size_t arm_count_{0};
    size_t total_count_{0};
    std::vector<double> states_;
  };

  // Adapts explicit torque requests and manages controller lifecycle (active/inactive).
  class TorqueManagerNode : public rclcpp::Node
  {
  public:
    explicit TorqueManagerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    using SetTorque = lekiwi_interfaces::srv::SetTorqueEnabled;
    using SwitchController = controller_manager_msgs::srv::SwitchController;

    void handle_request(
        std::shared_ptr<rclcpp::Service<SetTorque>> service,
        std::shared_ptr<rmw_request_id_t> header,
        SetTorque::Request::SharedPtr req);

    void switch_controllers_async(
        const std::vector<std::string> &controllers,
        bool activate,
        std::function<void(bool ok, const std::string &msg)> on_complete);

    std::vector<std::string> get_target_controllers(uint8_t target) const;

    TorqueCommandState state_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Service<SetTorque>::SharedPtr service_;
    rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;

    bool manage_controllers_{true};
    std::vector<std::string> arm_controllers_;
    std::vector<std::string> base_controllers_;
    std::atomic<bool> switch_in_progress_{false};
  };
} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__TORQUE_MANAGER_NODE_HPP_

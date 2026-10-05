// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file torque_manager_node.hpp
 * @brief Node and domain logic coordinating hardware torque commands and controller states.
 * @details Synchronizes ros2_control controller activation/deactivation with low-level
 *          motor bus torque toggles to prevent hardware runaway or joint drops.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

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
  /**
   * @class TorqueCommandState
   * @brief Domain logic maintaining joint torque command vectors across robot sub-chains.
   * @details Maintains separate partitions for arm joints and base wheels, allowing group-level
   *          torque enablement and status queries.
   */
  class TorqueCommandState
  {
  public:
    static constexpr double kTorqueEnabledValue = 1.0;
    static constexpr double kTorqueDisabledValue = 0.0;
    static constexpr double kTorqueStateThreshold = 0.5;

    /**
     * @brief Default constructor creating uninitialized state.
     */
    TorqueCommandState() = default;

    /**
     * @brief Initializes joint state partitions for arm and base groups.
     * @param[in] arm Joint name collection for manipulator arm.
     * @param[in] base Joint name collection for omni base wheels.
     * @throws std::invalid_argument If either arm or base joint collection is empty.
     */
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

    /**
     * @brief Applies torque enable/disable setting to specified target joint group.
     * @param[in] target Target group identifier (TARGET_ALL, TARGET_ARM, or TARGET_BASE).
     * @param[in] enabled Desired torque activation state.
     * @return Reference to updated command values vector.
     * @throws std::invalid_argument If target identifier is unrecognised.
     */
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

    /**
     * @brief Evaluates whether any joints in specified target group are currently enabled.
     * @param[in] target Target joint group identifier.
     * @return True if at least one joint has torque enabled.
     * @throws std::invalid_argument If target identifier is unrecognized.
     */
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

    /**
     * @brief Resets all joints to enabled state.
     */
    void reset() noexcept
    {
      std::fill(states_.begin(), states_.end(), kTorqueEnabledValue);
    }

    /**
     * @brief Retrieves number of arm joints configured.
     * @return Joint count in arm group.
     */
    [[nodiscard]] size_t arm_count() const noexcept { return arm_count_; }

    /**
     * @brief Retrieves total number of joints across all groups.
     * @return Total configured joints count.
     */
    [[nodiscard]] size_t total_count() const noexcept { return total_count_; }

    /**
     * @brief Provides read-only view of current commanded states.
     * @return Const reference to internal states vector.
     */
    [[nodiscard]] const std::vector<double> &current_states() const noexcept { return states_; }

  private:
    size_t arm_count_{0};
    size_t total_count_{0};
    std::vector<double> states_;
  };

  /**
   * @class TorqueManagerNode
   * @brief ROS 2 node coordinating torque states and ros2_control controller switches.
   * @details Exposes `/set_torque_enabled` service and executes safe transitions:
   *          deactivates controllers before cutting torque; enables torque before activating controllers.
   */
  class TorqueManagerNode : public rclcpp::Node
  {
  public:
    /**
     * @brief Constructs TorqueManagerNode and initializes parameters, publishers, and services.
     * @param[in] options Node options passed to parent rclcpp::Node.
     */
    explicit TorqueManagerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    using SetTorque = lekiwi_interfaces::srv::SetTorqueEnabled;
    using SwitchController = controller_manager_msgs::srv::SwitchController;

    /**
     * @brief Processes incoming torque change or toggle service request.
     * @param[in] service Shared pointer to service server instance.
     * @param[in] header RMW request identifier.
     * @param[in] req Service request containing target, enable state, and toggle flag.
     */
    void handle_request(
        std::shared_ptr<rclcpp::Service<SetTorque>> service,
        std::shared_ptr<rmw_request_id_t> header,
        SetTorque::Request::SharedPtr req);

    /**
     * @brief Asynchronously requests controller manager to activate or deactivate controllers.
     * @param[in] controllers Controller names to switch.
     * @param[in] activate True to activate; false to deactivate.
     * @param[in] on_complete Completion callback receiving success flag and message.
     */
    void switch_controllers_async(
        const std::vector<std::string> &controllers,
        bool activate,
        std::function<void(bool ok, const std::string &msg)> on_complete);

    /**
     * @brief Resolves list of controller names associated with target joint group.
     * @param[in] target Target group identifier.
     * @return List of matching controller names.
     */
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

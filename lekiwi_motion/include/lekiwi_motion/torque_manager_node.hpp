// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#ifndef LEKIWI_CONTROL__TORQUE_MANAGER_NODE_HPP_
#define LEKIWI_CONTROL__TORQUE_MANAGER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include "lekiwi_control/torque_command_state.hpp"

namespace lekiwi_control
{
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

    void execute_torque_publish_and_respond(
        std::shared_ptr<rclcpp::Service<SetTorque>> service,
        std::shared_ptr<rmw_request_id_t> header,
        const SetTorque::Request::SharedPtr req,
        const std::string &extra_message);

    std::vector<std::string> get_target_controllers(uint8_t target) const;

    std::unique_ptr<TorqueCommandState> state_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Service<SetTorque>::SharedPtr service_;
    rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;

    bool manage_controllers_{true};
    std::vector<std::string> arm_controllers_;
    std::vector<std::string> base_controllers_;
    bool switch_in_progress_{false};
  };
} // namespace lekiwi_control
#endif

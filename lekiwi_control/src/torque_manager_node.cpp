// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#include "lekiwi_control/torque_manager_node.hpp"

#include <functional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace lekiwi_control
{
  TorqueManagerNode::TorqueManagerNode(const rclcpp::NodeOptions &options)
      : Node("torque_manager", options)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Static torque adapter configuration";
    desc.read_only = true;

    const auto joints = declare_parameter<std::vector<std::string>>("joint_names", {}, desc);
    const auto arm = declare_parameter<std::vector<std::string>>("arm_joints", {}, desc);
    const auto base = declare_parameter<std::vector<std::string>>("base_joints", {}, desc);
    const auto topic = declare_parameter<std::string>("torque_controller_topic", "", desc);
    const auto policy = declare_parameter<std::string>("startup_policy", "enabled", desc);

    manage_controllers_ = declare_parameter<bool>("manage_controllers", true, desc);
    const auto switch_service = declare_parameter<std::string>(
        "switch_controller_service", "/controller_manager/switch_controller", desc);
    arm_controllers_ = declare_parameter<std::vector<std::string>>(
        "arm_controllers", {"arm_trajectory_controller"}, desc);
    base_controllers_ = declare_parameter<std::vector<std::string>>(
        "base_controllers", {"omni_base_controller"}, desc);

    if (policy != "preserve" && policy != "enabled")
    {
      throw std::invalid_argument("startup_policy must be 'preserve' or 'enabled'");
    }
    if (topic.empty())
    {
      throw std::invalid_argument("Torque topic must be nonempty");
    }

    state_ = std::make_unique<TorqueCommandState>(joints, arm, base);
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        topic, rclcpp::QoS(1).reliable().durability_volatile());

    if (manage_controllers_)
    {
      switch_controller_client_ = create_client<SwitchController>(switch_service);
    }

    service_ = create_service<SetTorque>(
        "set_torque_enabled",
        [this](
            std::shared_ptr<rclcpp::Service<SetTorque>> srv,
            std::shared_ptr<rmw_request_id_t> header,
            SetTorque::Request::SharedPtr req)
        {
          handle_request(srv, header, req);
        });

    RCLCPP_INFO(
        get_logger(),
        "TorqueManagerNode initialized (manage_controllers: %s, switch_service: %s)",
        manage_controllers_ ? "true" : "false", switch_service.c_str());
  }

  std::vector<std::string> TorqueManagerNode::get_target_controllers(uint8_t target) const
  {
    std::vector<std::string> list;
    if (target == SetTorque::Request::TARGET_ARM || target == SetTorque::Request::TARGET_ALL)
    {
      list.insert(list.end(), arm_controllers_.begin(), arm_controllers_.end());
    }
    if (target == SetTorque::Request::TARGET_BASE || target == SetTorque::Request::TARGET_ALL)
    {
      list.insert(list.end(), base_controllers_.begin(), base_controllers_.end());
    }
    return list;
  }

  void TorqueManagerNode::handle_request(
      std::shared_ptr<rclcpp::Service<SetTorque>> service,
      std::shared_ptr<rmw_request_id_t> header,
      SetTorque::Request::SharedPtr req)
  {
    if (switch_in_progress_)
    {
      SetTorque::Response res;
      res.success = false;
      res.message = "Another torque/controller switch operation is in progress";
      service->send_response(*header, res);
      return;
    }

    using Request = SetTorque::Request;
    if (req->target != Request::TARGET_ALL && req->target != Request::TARGET_ARM && req->target != Request::TARGET_BASE)
    {
      SetTorque::Response res;
      res.success = false;
      res.message = "Invalid target: " + std::to_string(req->target);
      service->send_response(*header, res);
      return;
    }

    if (cmd_pub_->get_subscription_count() == 0)
    {
      state_->reset();
      SetTorque::Response res;
      res.success = false;
      res.message = "Torque controller subscriber unavailable; command state invalidated";
      service->send_response(*header, res);
      return;
    }

    if (req->toggle)
    {
      req->enabled = !state_->is_group_enabled(req->target);
    }

    const auto target_controllers = get_target_controllers(req->target);

    // Case 1: Disabling Torque (enabled == false)
    // Deactivate controllers FIRST so they stop commanding setpoints, then disable motor torque.
    if (!req->enabled)
    {
      if (manage_controllers_ && !target_controllers.empty() &&
          switch_controller_client_ && switch_controller_client_->service_is_ready())
      {
        switch_in_progress_ = true;
        auto switch_req = std::make_shared<SwitchController::Request>();
        switch_req->deactivate_controllers = target_controllers;
        switch_req->strictness = SwitchController::Request::BEST_EFFORT;
        switch_req->timeout = rclcpp::Duration::from_seconds(2.0);

        switch_controller_client_->async_send_request(
            switch_req,
            [this, service, header, req](rclcpp::Client<SwitchController>::SharedFuture future)
            {
              switch_in_progress_ = false;
              auto switch_res = future.get();
              std::string extra_msg;
              if (switch_res && switch_res->ok)
              {
                extra_msg = "Controllers deactivated. ";
              }
              else
              {
                const std::string err = switch_res ? switch_res->message : "null response";
                RCLCPP_WARN(get_logger(), "SwitchController deactivation warning: %s", err.c_str());
                extra_msg = "Controllers deactivation warning (" + err + "). ";
              }
              execute_torque_publish_and_respond(service, header, req, extra_msg);
            });
        return;
      }

      if (manage_controllers_ && !target_controllers.empty())
      {
        RCLCPP_WARN(
            get_logger(),
            "SwitchController service unavailable; skipping controller deactivation.");
      }
      execute_torque_publish_and_respond(
          service, header, req,
          manage_controllers_ ? "Controller deactivation skipped (service unavailable). " : "");
      return;
    }

    // Case 2: Enabling Torque (enabled == true)
    // Enable motor torque FIRST so hardware is powered, then reactivate controllers.
    std_msgs::msg::Float64MultiArray command;
    try
    {
      command.data = state_->apply(req->target, true, true);
      cmd_pub_->publish(command);
    }
    catch (const std::exception &err)
    {
      state_->reset();
      SetTorque::Response res;
      res.success = false;
      res.message = std::string("Failed to enable torque: ") + err.what();
      service->send_response(*header, res);
      return;
    }

    if (manage_controllers_ && !target_controllers.empty() &&
        switch_controller_client_ && switch_controller_client_->service_is_ready())
    {
      switch_in_progress_ = true;
      auto switch_req = std::make_shared<SwitchController::Request>();
      switch_req->activate_controllers = target_controllers;
      switch_req->strictness = SwitchController::Request::BEST_EFFORT;
      switch_req->activate_asap = false;
      switch_req->timeout = rclcpp::Duration::from_seconds(2.0);

      switch_controller_client_->async_send_request(
          switch_req,
          [this, service, header](rclcpp::Client<SwitchController>::SharedFuture future)
          {
            switch_in_progress_ = false;
            auto switch_res = future.get();
            SetTorque::Response res;
            if (switch_res && switch_res->ok)
            {
              res.success = true;
              res.message = "Torque enabled and controllers reactivated successfully";
            }
            else
            {
              const std::string err = switch_res ? switch_res->message : "null response";
              RCLCPP_WARN(get_logger(), "SwitchController activation warning: %s", err.c_str());
              res.success = true;
              res.message = "Torque enabled, but controller activation warning: " + err;
            }
            service->send_response(*header, res);
          });
      return;
    }

    SetTorque::Response res;
    res.success = true;
    res.message = "Torque enabled" +
                  std::string(manage_controllers_ ? "; controller activation skipped (service unavailable)" : "");
    service->send_response(*header, res);
  }

  void TorqueManagerNode::execute_torque_publish_and_respond(
      std::shared_ptr<rclcpp::Service<SetTorque>> service,
      std::shared_ptr<rmw_request_id_t> header,
      const SetTorque::Request::SharedPtr req,
      const std::string &extra_message)
  {
    std_msgs::msg::Float64MultiArray command;
    SetTorque::Response res;
    try
    {
      command.data = state_->apply(req->target, req->enabled, cmd_pub_->get_subscription_count() > 0);
      cmd_pub_->publish(command);
      res.success = true;
      res.message = extra_message + "Torque disabled successfully";
    }
    catch (const std::exception &err)
    {
      state_->reset();
      res.success = false;
      res.message = extra_message + "Torque command failed: " + err.what();
      RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
    }
    service->send_response(*header, res);
  }
} // namespace lekiwi_control

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_control::TorqueManagerNode)

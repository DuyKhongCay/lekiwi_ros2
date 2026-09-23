// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
#include "lekiwi_motion/torque_manager_node.hpp"

#include <functional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace lekiwi_motion
{
  TorqueManagerNode::TorqueManagerNode(const rclcpp::NodeOptions &options)
      : Node("torque_manager", options)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Static torque adapter configuration";
    desc.read_only = true;

    const auto arm = declare_parameter<std::vector<std::string>>("arm_joints", {}, desc);
    const auto base = declare_parameter<std::vector<std::string>>("base_joints", {}, desc);
    const auto topic = declare_parameter<std::string>("torque_controller_topic", "", desc);

    manage_controllers_ = declare_parameter<bool>("manage_controllers", true, desc);
    const auto switch_service = declare_parameter<std::string>(
        "switch_controller_service", "/controller_manager/switch_controller", desc);
    arm_controllers_ = declare_parameter<std::vector<std::string>>(
        "arm_controllers", {"arm_trajectory_controller"}, desc);
    base_controllers_ = declare_parameter<std::vector<std::string>>(
        "base_controllers", {"omni_base_controller"}, desc);

    if (topic.empty())
    {
      throw std::invalid_argument("Torque topic must be nonempty");
    }

    state_ = TorqueCommandState(arm, base);
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
        "TorqueManagerNode initialized with %zu joints (manage_controllers: %s, switch_service: %s)",
        state_.total_count(), manage_controllers_ ? "true" : "false", switch_service.c_str());
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

  void TorqueManagerNode::switch_controllers_async(
      const std::vector<std::string> &controllers,
      bool activate,
      std::function<void(bool ok, const std::string &msg)> on_complete)
  {
    if (!manage_controllers_ || controllers.empty() ||
        !switch_controller_client_ || !switch_controller_client_->service_is_ready())
    {
      const std::string reason = (!manage_controllers_) ? "disabled" : controllers.empty() ? "no target controllers"
                                                                                           : "service unavailable";
      on_complete(false, "Controller switch skipped (" + reason + "). ");
      return;
    }

    switch_in_progress_.store(true, std::memory_order_release);
    auto switch_req = std::make_shared<SwitchController::Request>();
    if (activate)
    {
      switch_req->activate_controllers = controllers;
      switch_req->activate_asap = false;
    }
    else
    {
      switch_req->deactivate_controllers = controllers;
    }
    switch_req->strictness = SwitchController::Request::BEST_EFFORT;
    switch_req->timeout = rclcpp::Duration::from_seconds(2.0);

    std::weak_ptr<TorqueManagerNode> weak_self =
        std::static_pointer_cast<TorqueManagerNode>(shared_from_this());
    switch_controller_client_->async_send_request(
        switch_req,
        [weak_self, activate, on_complete](rclcpp::Client<SwitchController>::SharedFuture future)
        {
          auto self = weak_self.lock();
          if (!self)
          {
            return;
          }
          self->switch_in_progress_.store(false, std::memory_order_release);

          try
          {
            auto switch_res = future.get();
            if (switch_res && switch_res->ok)
            {
              on_complete(true, activate ? "Controllers activated. " : "Controllers deactivated. ");
            }
            else
            {
              const std::string err = switch_res ? switch_res->message : "null response";
              RCLCPP_WARN(
                  self->get_logger(), "SwitchController %s warning: %s",
                  activate ? "activation" : "deactivation", err.c_str());
              on_complete(
                  false,
                  "Controller " + std::string(activate ? "activation" : "deactivation") +
                      " warning (" + err + "). ");
            }
          }
          catch (const std::exception &err)
          {
            RCLCPP_ERROR(
                self->get_logger(), "SwitchController exception: %s", err.what());
            on_complete(false, "Controller switch exception (" + std::string(err.what()) + "). ");
          }
        });
  }

  void TorqueManagerNode::handle_request(
      std::shared_ptr<rclcpp::Service<SetTorque>> service,
      std::shared_ptr<rmw_request_id_t> header,
      SetTorque::Request::SharedPtr req)
  {
    if (switch_in_progress_.load(std::memory_order_acquire))
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
      state_.reset();
      SetTorque::Response res;
      res.success = false;
      res.message = "Torque controller subscriber unavailable; command state invalidated";
      service->send_response(*header, res);
      return;
    }

    if (req->toggle)
    {
      req->enabled = !state_.is_group_enabled(req->target);
    }

    const auto target_controllers = get_target_controllers(req->target);

    // Case 1: Disabling Torque (enabled == false)
    // Deactivate controllers FIRST so they stop commanding setpoints, then disable motor torque.
    if (!req->enabled)
    {
      std::weak_ptr<TorqueManagerNode> weak_self =
          std::static_pointer_cast<TorqueManagerNode>(shared_from_this());
      switch_controllers_async(
          target_controllers, false,
          [weak_self, service, header, req](bool /*ok*/, const std::string &extra_msg)
          {
            auto self = weak_self.lock();
            if (!self)
            {
              return;
            }

            SetTorque::Response res;
            try
            {
              std_msgs::msg::Float64MultiArray command;
              command.data = self->state_.apply(req->target, false);
              self->cmd_pub_->publish(command);
              res.success = true;
              res.message = extra_msg + "Torque disabled successfully";
            }
            catch (const std::exception &err)
            {
              self->state_.reset();
              res.success = false;
              res.message = extra_msg + "Failed to publish torque command: " + err.what();
              RCLCPP_ERROR(self->get_logger(), "%s", res.message.c_str());
            }
            service->send_response(*header, res);
          });
      return;
    }

    // Case 2: Enabling Torque (enabled == true)
    // Enable motor torque FIRST so hardware is powered, then reactivate controllers.
    SetTorque::Response early_res;
    try
    {
      std_msgs::msg::Float64MultiArray command;
      command.data = state_.apply(req->target, true);
      cmd_pub_->publish(command);
    }
    catch (const std::exception &err)
    {
      state_.reset();
      early_res.success = false;
      early_res.message = std::string("Failed to enable torque: ") + err.what();
      service->send_response(*header, early_res);
      return;
    }

    switch_controllers_async(
        target_controllers, true,
        [service, header](bool ok, const std::string &extra_msg)
        {
          SetTorque::Response res;
          res.success = true;
          res.message = ok ? "Torque enabled and controllers reactivated successfully"
                           : "Torque enabled, but " + extra_msg;
          service->send_response(*header, res);
        });
  }
} // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::TorqueManagerNode)

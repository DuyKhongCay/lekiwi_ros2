/**
 * @file lekiwi_feetech_hardware.cpp
 * @brief Implementation of the LeKiwiFeetechHardwareInterface ros2_control plugin adapter.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp"

#include <cmath>
#include <exception>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lekiwi_ftservo_hardware
{

  LeKiwiFeetechHardwareInterface::~LeKiwiFeetechHardwareInterface()
  {
    if (worker_)
    {
      worker_->stop();
    }
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &hardware_parameters = info_.hardware_parameters;
    const auto port_it = hardware_parameters.find("usb_port");
    if (port_it == hardware_parameters.end() || port_it->second.empty())
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Missing usb_port parameter");
      return hardware_interface::CallbackReturn::ERROR;
    }
    usb_port_ = port_it->second;

    const auto baud_it = hardware_parameters.find("baud_rate");
    if (baud_it != hardware_parameters.end())
    {
      try
      {
        baud_rate_ = std::stoi(baud_it->second);
      }
      catch (const std::exception &exception)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Invalid baud_rate '%s': %s",
                     baud_it->second.c_str(), exception.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    const auto timeout_it = hardware_parameters.find("timeout_ms");
    if (timeout_it != hardware_parameters.end())
    {
      try
      {
        timeout_ms_ = std::stoi(timeout_it->second);
      }
      catch (const std::exception &exception)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Invalid timeout_ms '%s': %s",
                     timeout_it->second.c_str(), exception.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Parse and validate joint topology
    std::string parse_error;
    if (!JointConfigParser::parse(info_, topology_, &parse_error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Joint configuration parsing failed: %s",
                   parse_error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Instantiate background I/O engine
    worker_ = std::make_unique<FeetechBusWorker>();
    std::string init_error;
    if (!worker_->init(topology_, &init_error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Worker initialization failed: %s",
                   init_error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_configure(
      const rclcpp_lifecycle::State &)
  {
    if (!worker_)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string config_error;
    if (!worker_->configure(usb_port_, baud_rate_, timeout_ms_, &config_error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", config_error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto &joint : topology_.joints)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("LeKiwiFeetechHardware"),
          "Configured %s joint '%s' (ID %u) with acceleration = %u",
          joint.velocity_command ? "velocity" : "position",
          joint.name.c_str(), joint.id, joint.acceleration);
    }

    if (get_node())
    {
      updater_ = std::make_shared<diagnostic_updater::Updater>(get_node());
      updater_->setHardwareID("lekiwi_feetech_servos");
      updater_->add(
          "Servo_Bus_Status", this,
          &LeKiwiFeetechHardwareInterface::produce_diagnostics);
      RCLCPP_INFO(
          rclcpp::get_logger("LeKiwiFeetechHardware"),
          "Diagnostic updater initialized for lekiwi_feetech_servos");
    }
    else
    {
      RCLCPP_WARN(
          rclcpp::get_logger("LeKiwiFeetechHardware"),
          "Default node is not available. Diagnostic updater will not be available.");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_activate(
      const rclcpp_lifecycle::State &)
  {
    if (!worker_)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string activate_error;
    if (!worker_->activate(&activate_error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Worker activation failed: %s",
                   activate_error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Seed initial state and command interfaces from initial snapshot
    const auto *snapshot = worker_->read_state_rt();
    const size_t num_joints = topology_.size();
    if (snapshot != nullptr && snapshot->valid &&
        snapshot->positions.size() >= num_joints &&
        snapshot->velocities.size() >= num_joints)
    {
      for (size_t i = 0; i < num_joints; ++i)
      {
        const auto &joint = topology_.joints[i];
        set_state(joint.position_state_name, snapshot->positions[i]);
        set_state(joint.velocity_state_name, snapshot->velocities[i]);

        const double init_cmd = joint.velocity_command ? 0.0 : snapshot->positions[i];
        set_command(joint.command_interface_name, init_cmd);

        if (joint.has_torque_enable_command)
        {
          set_command(joint.torque_enable_command_name, 1.0);
        }
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("LeKiwiFeetechHardware"),
        "Async I/O Worker started successfully for %zu STS servos.", topology_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    if (!worker_)
    {
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::string error;
    const bool success = worker_->deactivate(&error);
    if (!success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
    }
    return success ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    if (worker_)
    {
      worker_->cleanup();
    }
    updater_.reset();
    RCLCPP_INFO(rclcpp::get_logger("LeKiwiFeetechHardware"),
                "LeKiwiFeetechHardwareInterface cleaned up serial bus and resources successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type LeKiwiFeetechHardwareInterface::read(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Wait-free read from RealtimeBuffer (< 50 ns, zero heap allocation)
    const auto *snapshot = worker_->read_state_rt();
    const size_t num_joints = topology_.size();
    if (snapshot == nullptr || !snapshot->valid ||
        snapshot->positions.size() < num_joints ||
        snapshot->velocities.size() < num_joints)
    {
      return hardware_interface::return_type::OK;
    }

    for (size_t i = 0; i < num_joints; ++i)
    {
      set_state(topology_.joints[i].position_state_name, snapshot->positions[i]);
      set_state(topology_.joints[i].velocity_state_name, snapshot->velocities[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type LeKiwiFeetechHardwareInterface::write(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Wait-free push to atomic command buffer (< 50 ns, zero heap allocation)
    const size_t num_joints = topology_.size();
    bool torque_cmd_changed = false;

    for (size_t i = 0; i < num_joints; ++i)
    {
      const auto &joint = topology_.joints[i];
      double t_cmd = 1.0;
      bool just_enabled_torque = false;

      if (joint.has_torque_enable_command)
      {
        const double raw_t_cmd = get_command(joint.torque_enable_command_name);
        if (std::isfinite(raw_t_cmd))
        {
          t_cmd = raw_t_cmd;
          const double old_cmd = worker_->exchange_torque_enable(i, t_cmd);
          const bool is_en = (t_cmd >= 0.5);
          const bool was_en = (old_cmd >= 0.5);
          if (is_en != was_en)
          {
            torque_cmd_changed = true;
            if (is_en && !was_en)
            {
              just_enabled_torque = true;
            }
          }
        }
      }

      if (joint.velocity_command)
      {
        const double cmd = get_command(joint.command_interface_name);
        worker_->push_command(i, std::isfinite(cmd) ? cmd : 0.0);
      }
      else
      {
        // Position-controlled joint: Anti-Jerk & Lead-Through synchronization
        const bool torque_enabled = (t_cmd >= 0.5);
        if (!torque_enabled || just_enabled_torque)
        {
          // While torque is disabled OR at the exact transition when torque is re-enabled,
          // follow physical joint state to prevent violent snapping towards outdated targets.
          const double current_pos = get_state(joint.position_state_name);
          if (std::isfinite(current_pos))
          {
            set_command(joint.command_interface_name, current_pos);
            worker_->push_command(i, current_pos);
          }
        }
        else
        {
          const double cmd = get_command(joint.command_interface_name);
          if (std::isfinite(cmd))
          {
            worker_->push_command(i, cmd);
          }
        }
      }
    }

    if (torque_cmd_changed)
    {
      worker_->notify_torque_command_changed();
    }
    worker_->notify_new_command();

    return hardware_interface::return_type::OK;
  }

  std::vector<JointTelemetry> LeKiwiFeetechHardwareInterface::get_telemetry() const
  {
    if (!worker_)
    {
      return {};
    }
    const auto *snapshot = worker_->read_state_non_rt();
    return (snapshot != nullptr) ? snapshot->telemetry : std::vector<JointTelemetry>{};
  }

  void LeKiwiFeetechHardwareInterface::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    const JointStateSnapshot *snapshot = worker_ ? worker_->read_state_non_rt() : nullptr;
    const bool is_running = worker_ ? worker_->is_running() : false;

    FeetechDiagnostics::evaluate(
        snapshot,
        usb_port_,
        baud_rate_,
        topology_.size(),
        is_running,
        stat);
  }

} // namespace lekiwi_ftservo_hardware

PLUGINLIB_EXPORT_CLASS(
    lekiwi_ftservo_hardware::LeKiwiFeetechHardwareInterface, hardware_interface::SystemInterface)

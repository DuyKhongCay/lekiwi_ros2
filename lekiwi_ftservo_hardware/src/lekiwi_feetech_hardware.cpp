#include "lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp"

#include <cmath>
#include <chrono>
#include <exception>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "lekiwi_ftservo_hardware/velocity_codec.hpp"

namespace lekiwi_ftservo_hardware
{
  namespace
  {
    constexpr int kEncoderTicksPerRevolution = 4096;
    constexpr double kRadiansPerEncoderTick = (2.0 * M_PI) / kEncoderTicksPerRevolution;
    constexpr double kEncoderTicksPerRadian = kEncoderTicksPerRevolution / (2.0 * M_PI);
    constexpr uint8_t kModeRegister = StsProtocol::kModeRegister;
    constexpr uint8_t kTorqueEnableRegister = StsProtocol::kTorqueEnableRegister;
    constexpr uint8_t kPositionMode = 0;
    constexpr uint8_t kVelocityMode = 1;
  } // namespace

  LeKiwiFeetechHardwareInterface::~LeKiwiFeetechHardwareInterface()
  {
    io_running_ = false;
    if (io_worker_thread_.joinable())
    {
      io_worker_thread_.join();
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
    if (configure_joint_runtime() != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto joint_config_it = hardware_parameters.find("joint_config_file");
    if (joint_config_it != hardware_parameters.end() && !joint_config_it->second.empty() &&
        load_joint_configuration(joint_config_it->second) !=
            hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Allocate thread-safe state and command buffers
    const size_t num_joints = joints_.size();
    {
      std::lock_guard<std::mutex> lock(shared_state_.mutex);
      shared_state_.positions.assign(num_joints, 0.0);
      shared_state_.velocities.assign(num_joints, 0.0);
      shared_state_.telemetry.resize(num_joints);
      for (size_t i = 0; i < num_joints; ++i)
      {
        shared_state_.telemetry[i].name = joints_[i].name;
        shared_state_.telemetry[i].id = joints_[i].id;
      }
      shared_state_.valid = false;
    }
    {
      std::lock_guard<std::mutex> lock(shared_command_.mutex);
      shared_command_.commands.assign(num_joints, 0.0);
      shared_command_.has_new_command = false;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::configure_joint_runtime()
  {
    if (info_.joints.size() != 9U)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                   "Expected exactly 9 joints in URDF declaration, got %zu", info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joints_.clear();
    joint_ids_.clear();
    for (const auto &joint_info : info_.joints)
    {
      const auto id_it = joint_info.parameters.find("id");
      if (id_it == joint_info.parameters.end())
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                     "Missing required id parameter on joint %s", joint_info.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      int id = 0;
      try
      {
        id = std::stoi(id_it->second);
      }
      catch (const std::exception &exception)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                     "Invalid id parameter '%s' on joint %s: %s", id_it->second.c_str(),
                     joint_info.name.c_str(), exception.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (id <= 0 || id > 253)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                     "Out-of-range servo id %d on joint %s", id, joint_info.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint_info.command_interfaces.size() != 1U)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                     "Joint %s must declare exactly one command interface", joint_info.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      const auto &interface_name = joint_info.command_interfaces.front().name;
      const bool is_velocity = interface_name == hardware_interface::HW_IF_VELOCITY;
      const bool is_position = interface_name == hardware_interface::HW_IF_POSITION;
      if (!is_velocity && !is_position)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                     "Joint %s declares unsupported command interface '%s'", joint_info.name.c_str(),
                     interface_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      JointRuntime runtime;
      runtime.name = joint_info.name;
      runtime.id = static_cast<uint8_t>(id);
      runtime.velocity_command = is_velocity;
      runtime.velocity_radians_per_second_per_tick = 0.00785398;
      runtime.max_velocity_radians_per_second = 25.0;
      runtime.velocity_direction = 1;
      joints_.push_back(runtime);
      joint_ids_.push_back(runtime.id);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::load_joint_configuration(
      const std::string &file_path)
  {
    try
    {
      const YAML::Node document = YAML::LoadFile(file_path);
      const auto yaml_joints = document["joints"];
      if (!yaml_joints)
      {
        throw YAML::Exception(document.Mark(), "Missing joints map");
      }
      for (auto &joint : joints_)
      {
        if (!joint.velocity_command)
        {
          continue;
        }
        const auto yaml_joint = yaml_joints[joint.name];
        if (!yaml_joint)
        {
          throw YAML::Exception(yaml_joints.Mark(), "Wheel not found in YAML: " + joint.name);
        }
        joint.velocity_radians_per_second_per_tick =
            yaml_joint["velocity_radians_per_second_per_tick"].as<double>();
        joint.max_velocity_radians_per_second =
            yaml_joint["max_velocity_radians_per_second"].as<double>();
        joint.velocity_direction = yaml_joint["velocity_direction"].as<int>();
        if (joint.velocity_radians_per_second_per_tick <= 0.0 ||
            joint.max_velocity_radians_per_second <= 0.0 ||
            (joint.velocity_direction != -1 && joint.velocity_direction != 1))
        {
          throw YAML::Exception(yaml_joint.Mark(), "Invalid velocity conversion for " + joint.name);
        }
      }
    }
    catch (const YAML::Exception &exception)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                   "Cannot load joint configuration '%s': %s", file_path.c_str(), exception.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_configure(
      const rclcpp_lifecycle::State &)
  {
    protocol_ = std::make_unique<StsProtocol>();
    std::string error;
    if (!protocol_->open(usb_port_, baud_rate_, timeout_ms_, &error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
      (void)protocol_.release();
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto &joint : joints_)
    {
      if (!protocol_->write_register(joint.id, kTorqueEnableRegister, {0U}, &error) ||
          !protocol_->write_register(joint.id, kModeRegister,
                                     {joint.velocity_command ? kVelocityMode : kPositionMode}, &error))
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
        protocol_->close();
        (void)protocol_.release();
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_activate(
      const rclcpp_lifecycle::State &)
  {
    if (!protocol_)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Perform an initial synchronous read to seed starting positions
    std::vector<ServoFastState> initial_states;
    std::string error;
    if (!protocol_->sync_read_fast_state(joint_ids_, &initial_states, &error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
                   "Initial sync_read failed on activate: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const size_t num_joints = joints_.size();
    {
      std::lock_guard<std::mutex> lock(shared_state_.mutex);
      for (size_t i = 0; i < num_joints; ++i)
      {
        const double pos_rad = (initial_states[i].position_ticks - 2048) * kRadiansPerEncoderTick;
        const double vel_scale = joints_[i].velocity_command ? joints_[i].velocity_radians_per_second_per_tick : 0.0;
        const double vel_rad_s = initial_states[i].speed_ticks * vel_scale * joints_[i].velocity_direction;
        shared_state_.positions[i] = pos_rad;
        shared_state_.velocities[i] = vel_rad_s;
        shared_state_.telemetry[i].position_radians = pos_rad;
        shared_state_.telemetry[i].velocity_radians_per_second = vel_rad_s;

        set_state(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION, pos_rad);
        set_state(joints_[i].name + "/" + hardware_interface::HW_IF_VELOCITY, vel_rad_s);
      }
      shared_state_.valid = true;
      shared_state_.last_read_time = std::chrono::steady_clock::now();
    }

    // Seed initial command interfaces to prevent sudden jumps
    {
      std::lock_guard<std::mutex> lock(shared_command_.mutex);
      for (size_t i = 0; i < num_joints; ++i)
      {
        if (joints_[i].velocity_command)
        {
          set_command(joints_[i].name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
          shared_command_.commands[i] = 0.0;
        }
        else
        {
          const double initial_pos = shared_state_.positions[i];
          set_command(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION, initial_pos);
          shared_command_.commands[i] = initial_pos;
        }
      }
      shared_command_.has_new_command = false;
    }

    // Stop wheels and enable torque before launching async worker thread
    if (!stop_wheels(&error) || !set_all_torque(true, &error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Start Async I/O Worker Thread
    io_running_ = true;
    io_worker_thread_ = std::thread(&LeKiwiFeetechHardwareInterface::io_worker_loop, this);

    RCLCPP_INFO(rclcpp::get_logger("LeKiwiFeetechHardware"),
                "Async I/O Worker Thread started successfully for 9 STS servos.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    // 1. Stop Async I/O Worker Thread first
    io_running_ = false;
    if (io_worker_thread_.joinable())
    {
      io_worker_thread_.join();
    }

    // 2. Stop wheels and disable torque
    std::string error;
    const bool stopped = stop_wheels(&error);
    const bool torque_disabled = set_all_torque(false, &error);
    if (!stopped || !torque_disabled)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
    }
    if (protocol_)
    {
      protocol_->close();
      protocol_.reset();
    }
    return stopped && torque_disabled ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  }

  bool LeKiwiFeetechHardwareInterface::set_all_torque(const bool enabled, std::string *error)
  {
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Feetech protocol is not configured";
      }
      return false;
    }
    for (const auto &joint : joints_)
    {
      if (!protocol_->write_register(joint.id, kTorqueEnableRegister, {static_cast<uint8_t>(enabled ? 1 : 0)}, error))
      {
        return false;
      }
    }
    return true;
  }

  bool LeKiwiFeetechHardwareInterface::stop_wheels(std::string *error)
  {
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Feetech protocol is not configured";
      }
      return false;
    }
    std::vector<uint8_t> ids;
    std::vector<int> commands;
    for (const auto &joint : joints_)
    {
      if (joint.velocity_command)
      {
        ids.push_back(joint.id);
        commands.push_back(0);
      }
    }
    return ids.empty() || protocol_->sync_write_velocity(ids, commands, error);
  }

  hardware_interface::return_type LeKiwiFeetechHardwareInterface::read(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Fast memory read from shared buffer (executes in < 5 us, eliminating controller_manager overrun)
    std::lock_guard<std::mutex> lock(shared_state_.mutex);
    if (!shared_state_.valid)
    {
      return hardware_interface::return_type::OK;
    }

    const size_t num_joints = joints_.size();
    for (size_t i = 0; i < num_joints; ++i)
    {
      set_state(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION, shared_state_.positions[i]);
      set_state(joints_[i].name + "/" + hardware_interface::HW_IF_VELOCITY, shared_state_.velocities[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type LeKiwiFeetechHardwareInterface::write(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Fast memory push to command buffer (executes in < 5 us)
    const size_t num_joints = joints_.size();
    std::lock_guard<std::mutex> lock(shared_command_.mutex);
    for (size_t i = 0; i < num_joints; ++i)
    {
      if (joints_[i].velocity_command)
      {
        shared_command_.commands[i] = get_command(joints_[i].name + "/" + hardware_interface::HW_IF_VELOCITY);
      }
      else
      {
        const double cmd = get_command(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION);
        if (std::isfinite(cmd))
        {
          shared_command_.commands[i] = cmd;
        }
      }
    }
    shared_command_.has_new_command = true;
    return hardware_interface::return_type::OK;
  }

  void LeKiwiFeetechHardwareInterface::io_worker_loop()
  {
    const size_t num_joints = joints_.size();
    uint64_t iteration_count = 0;

    std::vector<uint8_t> velocity_ids;
    std::vector<uint8_t> position_ids;
    for (const auto &joint : joints_)
    {
      if (joint.velocity_command)
      {
        velocity_ids.push_back(joint.id);
      }
      else
      {
        position_ids.push_back(joint.id);
      }
    }

    std::vector<ServoFastState> fast_states;
    std::vector<ServoDiagnosticData> diag_states;
    std::string error;

    while (io_running_)
    {
      const auto loop_start = std::chrono::steady_clock::now();

      // 1. Hardware Read Phase:
      // Every 10 iterations (~10 Hz), perform full diagnostic sync_read (15 bytes),
      // otherwise perform fast state sync_read (4 bytes).
      const bool do_full_diagnostic = (iteration_count % 10 == 0);

      if (do_full_diagnostic)
      {
        if (protocol_->sync_read_diagnostics(joint_ids_, &diag_states, &error))
        {
          std::lock_guard<std::mutex> lock(shared_state_.mutex);
          for (size_t i = 0; i < num_joints; ++i)
          {
            const double pos_rad = (diag_states[i].position_ticks - 2048) * kRadiansPerEncoderTick;
            const double vel_scale = joints_[i].velocity_command ? joints_[i].velocity_radians_per_second_per_tick : 0.0;
            const double vel_rad_s = diag_states[i].speed_ticks * vel_scale * joints_[i].velocity_direction;

            shared_state_.positions[i] = pos_rad;
            shared_state_.velocities[i] = vel_rad_s;

            auto &telem = shared_state_.telemetry[i];
            telem.name = joints_[i].name;
            telem.id = joints_[i].id;
            telem.position_radians = pos_rad;
            telem.velocity_radians_per_second = vel_rad_s;
            telem.load_ratio = static_cast<double>(diag_states[i].load_raw) * 0.001;
            telem.voltage_v = diag_states[i].voltage_v;
            telem.temperature_c = diag_states[i].temperature_c;
            telem.current_a = diag_states[i].current_a;
            telem.moving = diag_states[i].moving;
            telem.status_flags = diag_states[i].status;
          }
          shared_state_.valid = true;
          shared_state_.last_read_time = std::chrono::steady_clock::now();
          ++shared_state_.update_count;
        }
      }
      else
      {
        if (protocol_->sync_read_fast_state(joint_ids_, &fast_states, &error))
        {
          std::lock_guard<std::mutex> lock(shared_state_.mutex);
          for (size_t i = 0; i < num_joints; ++i)
          {
            const double pos_rad = (fast_states[i].position_ticks - 2048) * kRadiansPerEncoderTick;
            const double vel_scale = joints_[i].velocity_command ? joints_[i].velocity_radians_per_second_per_tick : 0.0;
            const double vel_rad_s = fast_states[i].speed_ticks * vel_scale * joints_[i].velocity_direction;

            shared_state_.positions[i] = pos_rad;
            shared_state_.velocities[i] = vel_rad_s;
            shared_state_.telemetry[i].position_radians = pos_rad;
            shared_state_.telemetry[i].velocity_radians_per_second = vel_rad_s;
          }
          shared_state_.valid = true;
          shared_state_.last_read_time = std::chrono::steady_clock::now();
          ++shared_state_.update_count;
        }
      }

      // 2. Hardware Write Phase:
      std::vector<double> current_cmds;
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lock(shared_command_.mutex);
        if (shared_command_.has_new_command)
        {
          current_cmds = shared_command_.commands;
          has_cmd = true;
          shared_command_.has_new_command = false;
        }
      }

      if (has_cmd && current_cmds.size() == num_joints)
      {
        std::vector<int> velocity_commands;
        std::vector<int> position_commands;

        for (size_t i = 0; i < num_joints; ++i)
        {
          if (joints_[i].velocity_command)
          {
            velocity_commands.push_back(radians_per_second_to_ticks(
                current_cmds[i],
                joints_[i].velocity_radians_per_second_per_tick,
                joints_[i].max_velocity_radians_per_second,
                joints_[i].velocity_direction));
          }
          else
          {
            const double pos = current_cmds[i];
            if (std::isfinite(pos))
            {
              position_commands.push_back(
                  static_cast<int>(std::lround(pos * kEncoderTicksPerRadian)) + 2048);
            }
          }
        }

        if (!velocity_ids.empty() && !velocity_commands.empty())
        {
          protocol_->sync_write_velocity(velocity_ids, velocity_commands, &error);
        }
        if (!position_ids.empty() && !position_commands.empty())
        {
          protocol_->sync_write_position(position_ids, position_commands, &error);
        }
      }

      ++iteration_count;

      // 3. Pacing: Target ~100 Hz (10 ms period)
      const auto loop_elapsed = std::chrono::steady_clock::now() - loop_start;
      const auto target_period = std::chrono::milliseconds(10);
      if (loop_elapsed < target_period && io_running_)
      {
        std::this_thread::sleep_for(target_period - loop_elapsed);
      }
    }
  }

  std::vector<JointTelemetry> LeKiwiFeetechHardwareInterface::get_telemetry() const
  {
    std::lock_guard<std::mutex> lock(shared_state_.mutex);
    return shared_state_.telemetry;
  }

} // namespace lekiwi_ftservo_hardware

PLUGINLIB_EXPORT_CLASS(
    lekiwi_ftservo_hardware::LeKiwiFeetechHardwareInterface, hardware_interface::SystemInterface)

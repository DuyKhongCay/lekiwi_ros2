/**
 * @file feetech_bus_worker.cpp
 * @brief Implementation of FeetechBusWorker background I/O engine.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_ftservo_hardware/feetech_bus_worker.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "lekiwi_ftservo_hardware/velocity_codec.hpp"

namespace lekiwi_ftservo_hardware
{
  using namespace sts::resolution;

  FeetechBusWorker::FeetechBusWorker() = default;

  FeetechBusWorker::~FeetechBusWorker() noexcept
  {
    stop();
    cleanup();
  }

  bool FeetechBusWorker::init(const JointTopology &topology, std::string *) noexcept
  {
    topology_ = topology;
    const size_t num_joints = topology_.size();

    JointStateSnapshot initial_snapshot;
    initial_snapshot.positions.assign(num_joints, 0.0);
    initial_snapshot.velocities.assign(num_joints, 0.0);
    initial_snapshot.telemetry.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i)
    {
      initial_snapshot.telemetry[i].name = topology_.joints[i].name;
      initial_snapshot.telemetry[i].id = topology_.joints[i].id;
    }
    initial_snapshot.valid = false;
    rt_state_buffer_.initRT(initial_snapshot);

    command_buffer_ = std::vector<std::atomic<double>>(num_joints);
    torque_enable_buffer_ = std::vector<std::atomic<double>>(num_joints);
    active_torque_states_.assign(num_joints, 1U);

    for (size_t i = 0; i < num_joints; ++i)
    {
      command_buffer_[i].store(0.0, std::memory_order_relaxed);
      torque_enable_buffer_[i].store(1.0, std::memory_order_relaxed);
    }
    has_new_command_.store(false, std::memory_order_release);
    has_new_torque_command_.store(false, std::memory_order_release);

    return true;
  }

  bool FeetechBusWorker::configure(
      const std::string &usb_port,
      const int baud_rate,
      const int timeout_ms,
      std::string *error) noexcept
  {
    usb_port_ = usb_port;
    baud_rate_ = baud_rate;
    timeout_ms_ = timeout_ms;

    std::lock_guard<std::mutex> lock_serial(serial_mutex_);
    protocol_ = std::make_unique<StsProtocol>();
    if (!protocol_->open(usb_port_, baud_rate_, timeout_ms_, error))
    {
      protocol_.reset();
      return false;
    }

    for (const auto &joint : topology_.joints)
    {
      // 1. Ensure torque is disabled initially during configure
      if (!protocol_->write_register(joint.id, sts::register_addr::kTorqueEnable, {0U}, error))
      {
        protocol_->close();
        protocol_.reset();
        return false;
      }
      // 2. Configure operating mode (Velocity for continuous wheel, Position for arm joint)
      const uint8_t mode = joint.velocity_command ? sts::mode::kVelocity : sts::mode::kPosition;
      if (!protocol_->write_register(joint.id, sts::register_addr::kMode, {mode}, error))
      {
        protocol_->close();
        protocol_.reset();
        return false;
      }
      // 3. Configure acceleration ramp profile (e.g. 50 for arms, 0 for wheels)
      if (!protocol_->write_register(joint.id, sts::register_addr::kAcceleration, {joint.acceleration}, error))
      {
        protocol_->close();
        protocol_.reset();
        return false;
      }
    }

    return true;
  }

  bool FeetechBusWorker::activate(std::string *error) noexcept
  {
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Feetech protocol is unconfigured";
      }
      return false;
    }

    // Perform an initial synchronous read to seed starting positions
    std::vector<ServoFastState> initial_states;
    {
      std::lock_guard<std::mutex> lock_serial(serial_mutex_);
      if (!protocol_->sync_read_fast_state(topology_.all_ids, &initial_states, error))
      {
        return false;
      }
    }

    const size_t num_joints = topology_.size();
    JointStateSnapshot initial_snapshot;
    initial_snapshot.positions.resize(num_joints);
    initial_snapshot.velocities.resize(num_joints);
    initial_snapshot.telemetry.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i)
    {
      const auto &joint = topology_.joints[i];
      const double pos_rad = (initial_states[i].position_ticks - kEncoderCenterTicks) * kRadiansPerEncoderTick;
      const double vel_scale = joint.velocity_command ? joint.velocity_radians_per_second_per_tick : 0.0;
      const double vel_rad_s = initial_states[i].speed_ticks * vel_scale * joint.velocity_direction;

      initial_snapshot.positions[i] = pos_rad;
      initial_snapshot.velocities[i] = vel_rad_s;
      initial_snapshot.telemetry[i].name = joint.name;
      initial_snapshot.telemetry[i].id = joint.id;
      initial_snapshot.telemetry[i].position_radians = pos_rad;
      initial_snapshot.telemetry[i].velocity_radians_per_second = vel_rad_s;

      const double init_cmd = joint.velocity_command ? 0.0 : pos_rad;
      command_buffer_[i].store(init_cmd, std::memory_order_relaxed);
      torque_enable_buffer_[i].store(1.0, std::memory_order_relaxed);
      active_torque_states_[i] = 1U;
    }

    initial_snapshot.valid = true;
    initial_snapshot.last_read_time = std::chrono::steady_clock::now();
    rt_state_buffer_.initRT(initial_snapshot);

    has_new_command_.store(false, std::memory_order_release);
    has_new_torque_command_.store(false, std::memory_order_release);

    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    last_command_timestamp_ns_.store(now_ns, std::memory_order_release);
    consecutive_read_errors_.store(0, std::memory_order_relaxed);
    last_reconnect_attempt_ = std::chrono::steady_clock::now();

    // Stop velocity joints and enable torque on all joints before launching async worker thread
    if (!stop_velocity_joints(error) || !set_all_torque(true, error))
    {
      return false;
    }

    // Start Async I/O Worker Thread
    io_running_.store(true, std::memory_order_release);
    io_worker_thread_ = std::thread(&FeetechBusWorker::run_io_loop, this);

    return true;
  }

  bool FeetechBusWorker::deactivate(std::string *error) noexcept
  {
    stop();

    const bool stopped = stop_velocity_joints(error);
    const bool torque_disabled = set_all_torque(false, error);
    for (size_t i = 0; i < active_torque_states_.size(); ++i)
    {
      active_torque_states_[i] = 0U;
    }
    return stopped && torque_disabled;
  }

  void FeetechBusWorker::stop() noexcept
  {
    io_running_.store(false, std::memory_order_release);
    if (io_worker_thread_.joinable())
    {
      io_worker_thread_.join();
    }
  }

  void FeetechBusWorker::cleanup() noexcept
  {
    stop();
    std::lock_guard<std::mutex> lock_serial(serial_mutex_);
    if (protocol_)
    {
      protocol_->close();
      protocol_.reset();
    }
  }

  bool FeetechBusWorker::set_all_torque(const bool enabled, std::string *error) noexcept
  {
    std::lock_guard<std::mutex> lock_serial(serial_mutex_);
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Feetech protocol is not configured";
      }
      return false;
    }
    const std::vector<bool> enable_states(topology_.all_ids.size(), enabled);
    if (!protocol_->sync_write_torque(topology_.all_ids, enable_states, error))
    {
      return false;
    }
    for (size_t i = 0; i < active_torque_states_.size(); ++i)
    {
      active_torque_states_[i] = enabled ? 1U : 0U;
      torque_enable_buffer_[i].store(enabled ? 1.0 : 0.0, std::memory_order_relaxed);
    }
    return true;
  }

  bool FeetechBusWorker::stop_velocity_joints(std::string *error) noexcept
  {
    std::lock_guard<std::mutex> lock_serial(serial_mutex_);
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Feetech protocol is not configured";
      }
      return false;
    }
    std::vector<uint8_t> active_vel_ids;
    for (const size_t v_idx : topology_.velocity_joint_indices)
    {
      if (v_idx < active_torque_states_.size() && active_torque_states_[v_idx] != 0U)
      {
        active_vel_ids.push_back(topology_.joints[v_idx].id);
      }
    }
    if (active_vel_ids.empty())
    {
      return true;
    }
    const std::vector<int> commands(active_vel_ids.size(), 0);
    return protocol_->sync_write_velocity(active_vel_ids, commands, error);
  }

  void FeetechBusWorker::run_io_loop() noexcept
  {
    const size_t num_joints = topology_.size();
    uint64_t iteration_count = 0;

    std::vector<ServoFastState> fast_states;
    std::vector<ServoDiagnosticData> diag_states;
    std::string error;

    // Pre-allocated scratch vectors to guarantee zero dynamic heap allocations at 100 Hz (MISRA C++ Rule 18-0-1)
    std::vector<double> current_cmds(num_joints, 0.0);
    std::vector<int> velocity_commands;
    velocity_commands.reserve(topology_.velocity_joint_ids.size());
    std::vector<int> position_commands;
    position_commands.reserve(topology_.position_joint_ids.size());
    std::vector<uint8_t> active_vel_ids;
    active_vel_ids.reserve(topology_.velocity_joint_ids.size());
    std::vector<uint8_t> active_pos_ids;
    active_pos_ids.reserve(topology_.position_joint_ids.size());
    std::vector<uint8_t> torque_change_ids;
    torque_change_ids.reserve(num_joints);
    std::vector<bool> torque_change_states;
    torque_change_states.reserve(num_joints);

    // Local snapshot buffer updated in worker thread
    JointStateSnapshot local_snapshot;
    local_snapshot.positions.assign(num_joints, 0.0);
    local_snapshot.velocities.assign(num_joints, 0.0);
    local_snapshot.telemetry.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i)
    {
      local_snapshot.telemetry[i].name = topology_.joints[i].name;
      local_snapshot.telemetry[i].id = topology_.joints[i].id;
    }

    const auto target_period = std::chrono::milliseconds(sts::default_config::kDefaultLoopPeriodMs);
    auto next_cycle = std::chrono::steady_clock::now();

    while (io_running_.load(std::memory_order_relaxed))
    {
      // 1. Hardware Read Phase:
      // Every 10 iterations (~10 Hz), perform full diagnostic sync_read (15 bytes),
      // otherwise perform fast state sync_read (4 bytes).
      const bool do_full_diagnostic = (iteration_count % 10 == 0);
      bool read_success = false;

      {
        std::lock_guard<std::mutex> lock_serial(serial_mutex_);
        if (do_full_diagnostic)
        {
          read_success = protocol_->sync_read_diagnostics(topology_.all_ids, &diag_states, &error);
        }
        else
        {
          read_success = protocol_->sync_read_fast_state(topology_.all_ids, &fast_states, &error);
        }
      }

      if (read_success)
      {
        if (do_full_diagnostic)
        {
          for (size_t i = 0; i < num_joints; ++i)
          {
            const auto &joint = topology_.joints[i];
            const double pos_rad = (diag_states[i].position_ticks - kEncoderCenterTicks) * kRadiansPerEncoderTick;
            const double vel_scale = joint.velocity_command ? joint.velocity_radians_per_second_per_tick : 0.0;
            const double vel_rad_s = diag_states[i].speed_ticks * vel_scale * joint.velocity_direction;

            local_snapshot.positions[i] = pos_rad;
            local_snapshot.velocities[i] = vel_rad_s;

            auto &telem = local_snapshot.telemetry[i];
            telem.id = joint.id;
            telem.position_radians = pos_rad;
            telem.velocity_radians_per_second = vel_rad_s;
            telem.load_ratio = static_cast<double>(diag_states[i].load_raw) * sts::telemetry_scale::kLoadNormalizedPerUnit;
            telem.voltage_v = diag_states[i].voltage_v;
            telem.temperature_c = diag_states[i].temperature_c;
            telem.current_a = diag_states[i].current_a;
            telem.moving = diag_states[i].moving;
            telem.status_flags = diag_states[i].status;
          }
        }
        else
        {
          for (size_t i = 0; i < num_joints; ++i)
          {
            const auto &joint = topology_.joints[i];
            const double pos_rad = (fast_states[i].position_ticks - kEncoderCenterTicks) * kRadiansPerEncoderTick;
            const double vel_scale = joint.velocity_command ? joint.velocity_radians_per_second_per_tick : 0.0;
            const double vel_rad_s = fast_states[i].speed_ticks * vel_scale * joint.velocity_direction;

            local_snapshot.positions[i] = pos_rad;
            local_snapshot.velocities[i] = vel_rad_s;
            local_snapshot.telemetry[i].position_radians = pos_rad;
            local_snapshot.telemetry[i].velocity_radians_per_second = vel_rad_s;
          }
        }
        local_snapshot.valid = true;
        local_snapshot.last_read_time = std::chrono::steady_clock::now();
        ++local_snapshot.update_count;
        consecutive_read_errors_.store(0, std::memory_order_relaxed);

        // Push new snapshot to RealtimeBuffer (wait-free swap)
        rt_state_buffer_.writeFromNonRT(local_snapshot);
      }
      else
      {
        const uint64_t err_count = consecutive_read_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
        ++local_snapshot.read_error_count;

        // Fault Escalation Level 1: Flush serial input buffer immediately on error
        {
          std::lock_guard<std::mutex> lock_serial(serial_mutex_);
          if (protocol_)
          {
            protocol_->flush_input();
          }
        }

        // Fault Escalation Level 2: Invalidate telemetry after consecutive errors (50 ms)
        if (err_count >= sts::default_config::kMaxConsecutiveErrorsBeforeInvalid)
        {
          local_snapshot.valid = false;
          for (size_t i = 0; i < num_joints; ++i)
          {
            local_snapshot.velocities[i] = 0.0;
            local_snapshot.telemetry[i].velocity_radians_per_second = 0.0;
          }
        }
        rt_state_buffer_.writeFromNonRT(local_snapshot);

        // Fault Escalation Level 3: Auto-reconnect if bus remains offline for > 100 cycles (1.0 s)
        if (err_count >= sts::default_config::kMaxConsecutiveErrorsBeforeReconnect)
        {
          const auto now_reconnect = std::chrono::steady_clock::now();
          if (now_reconnect - last_reconnect_attempt_ > std::chrono::seconds(1))
          {
            last_reconnect_attempt_ = now_reconnect;
            RCLCPP_WARN(rclcpp::get_logger("FeetechBusWorker"),
                        "Serial bus unresponsive (%zu consecutive errors). Attempting port reconnect on %s...",
                        static_cast<size_t>(err_count), usb_port_.c_str());
            std::lock_guard<std::mutex> lock_serial(serial_mutex_);
            if (protocol_)
            {
              protocol_->close();
              std::string reopen_err;
              if (protocol_->open(usb_port_, baud_rate_, timeout_ms_, &reopen_err))
              {
                RCLCPP_INFO(rclcpp::get_logger("FeetechBusWorker"),
                            "Serial bus reconnected successfully on %s!", usb_port_.c_str());
                consecutive_read_errors_.store(0, std::memory_order_relaxed);
              }
            }
          }
        }
      }

      // 2. Hardware Write Phase:
      // A. Check for torque command updates:
      if (has_new_torque_command_.load(std::memory_order_acquire))
      {
        has_new_torque_command_.store(false, std::memory_order_release);
        torque_change_ids.clear();
        torque_change_states.clear();

        for (size_t i = 0; i < num_joints; ++i)
        {
          const bool desired_torque = (torque_enable_buffer_[i].load(std::memory_order_relaxed) >= 0.5);
          const bool current_torque = (active_torque_states_[i] != 0U);
          if (desired_torque != current_torque)
          {
            torque_change_ids.push_back(topology_.joints[i].id);
            torque_change_states.push_back(desired_torque);
            // Defensive secondary guard: seed command buffer with present feedback
            if (desired_torque && !topology_.joints[i].velocity_command && local_snapshot.valid)
            {
              command_buffer_[i].store(local_snapshot.positions[i], std::memory_order_relaxed);
            }
            active_torque_states_[i] = desired_torque ? 1U : 0U;
          }
        }

        if (!torque_change_ids.empty())
        {
          std::lock_guard<std::mutex> lock_serial(serial_mutex_);
          protocol_->sync_write_torque(torque_change_ids, torque_change_states, &error);
        }
      }

      // B. Check for motion command updates:
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
      const auto last_cmd_ns = last_command_timestamp_ns_.load(std::memory_order_acquire);
      const int64_t cmd_age_ms = (last_cmd_ns > 0) ? ((now_ns - last_cmd_ns) / 1'000'000) : 999999;
      const bool command_stale = (cmd_age_ms > sts::default_config::kCommandWatchdogTimeoutMs);

      if (command_stale && !topology_.velocity_joint_ids.empty())
      {
        // Deadman switch: Brake all velocity joints to 0 rad/s if controller stops publishing (> 100 ms)
        (void)stop_velocity_joints(&error);
      }
      else if (has_new_command_.load(std::memory_order_acquire))
      {
        for (size_t i = 0; i < num_joints; ++i)
        {
          current_cmds[i] = command_buffer_[i].load(std::memory_order_relaxed);
        }
        has_new_command_.store(false, std::memory_order_release);

        velocity_commands.clear();
        active_vel_ids.clear();
        for (const size_t v_idx : topology_.velocity_joint_indices)
        {
          if (active_torque_states_[v_idx] != 0U)
          {
            active_vel_ids.push_back(topology_.joints[v_idx].id);
            const double vel = current_cmds[v_idx];
            velocity_commands.push_back(radians_per_second_to_ticks(
                vel,
                topology_.joints[v_idx].velocity_radians_per_second_per_tick,
                topology_.joints[v_idx].max_velocity_radians_per_second,
                topology_.joints[v_idx].velocity_direction));
          }
        }

        position_commands.clear();
        active_pos_ids.clear();
        for (const size_t p_idx : topology_.position_joint_indices)
        {
          if (active_torque_states_[p_idx] != 0U)
          {
            active_pos_ids.push_back(topology_.joints[p_idx].id);
            const double pos = current_cmds[p_idx];
            if (std::isfinite(pos))
            {
              position_commands.push_back(
                  static_cast<int>(std::lround(pos * kEncoderTicksPerRadian)) + kEncoderCenterTicks);
            }
            else
            {
              position_commands.push_back(kEncoderCenterTicks);
            }
          }
        }

        std::lock_guard<std::mutex> lock_serial(serial_mutex_);
        if (!active_vel_ids.empty() && !velocity_commands.empty())
        {
          protocol_->sync_write_velocity(active_vel_ids, velocity_commands, &error);
        }
        if (!active_pos_ids.empty() && !position_commands.empty())
        {
          protocol_->sync_write_position(active_pos_ids, position_commands, &error);
        }
      }

      ++iteration_count;

      // 3. Drift-Free Pacing: Target ~100 Hz (10 ms period) via sleep_until
      next_cycle += target_period;
      if (std::chrono::steady_clock::now() < next_cycle && io_running_.load(std::memory_order_relaxed))
      {
        std::this_thread::sleep_until(next_cycle);
      }
      else
      {
        next_cycle = std::chrono::steady_clock::now();
      }
    }
  }

} // namespace lekiwi_ftservo_hardware

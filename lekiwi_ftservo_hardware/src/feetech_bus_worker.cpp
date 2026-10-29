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

#include "lekiwi_ftservo_hardware/joint_codec.hpp"

namespace lekiwi_ftservo_hardware
{
  using namespace sts::resolution;

  FeetechBusWorker::FeetechBusWorker() = default;

  FeetechBusWorker::~FeetechBusWorker() noexcept
  {
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

    bus_state_.store(sts::BusState::DISCONNECTED, std::memory_order_relaxed);
    active_dtc_.store(sts::dtc::kDtcNone, std::memory_order_relaxed);
    current_per_.store(0.0, std::memory_order_relaxed);
    per_window_.fill(0);
    per_window_idx_ = 0;
    per_error_count_ = 0;
    per_window_filled_ = 0;
    last_state_transition_ = std::chrono::steady_clock::now();
    reconnect_backoff_ms_ = std::chrono::milliseconds(sts::default_config::kReconnectBackoffInitialMs);

    return true;
  }

  void FeetechBusWorker::transition_to(const sts::BusState new_state, const uint32_t dtc) noexcept
  {
    bus_state_.store(new_state, std::memory_order_release);
    active_dtc_.store(dtc, std::memory_order_release);
    last_state_transition_ = std::chrono::steady_clock::now();
  }

  void FeetechBusWorker::record_cycle_result(const bool success) noexcept
  {
    const uint8_t err_val = success ? 0U : 1U;
    const uint8_t old_val = per_window_[per_window_idx_];
    per_window_[per_window_idx_] = err_val;
    per_error_count_ = per_error_count_ - old_val + err_val;
    per_window_idx_ = (per_window_idx_ + 1U) % sts::default_config::kPerWindowSize;
    if (per_window_filled_ < sts::default_config::kPerWindowSize)
    {
      ++per_window_filled_;
    }
    if (per_window_filled_ > 0)
    {
      const double rate = (static_cast<double>(per_error_count_) / static_cast<double>(per_window_filled_)) * 100.0;
      current_per_.store(rate, std::memory_order_relaxed);
    }
  }

  bool FeetechBusWorker::rearm_servo_hardware(std::string *error) noexcept
  {
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Serial protocol uninitialized";
      }
      return false;
    }

    for (const auto &joint : topology_.joints)
    {
      // 1. Ensure torque is disabled initially during configure / re-arming
      if (!protocol_->write_register(joint.id, sts::register_addr::kTorqueEnable, {0U}, error))
      {
        return false;
      }
      // 2. Configure operating mode (Velocity for continuous wheel, Position for arm joint)
      const uint8_t mode = joint.velocity_command ? sts::mode::kVelocity : sts::mode::kPosition;
      if (!protocol_->write_register(joint.id, sts::register_addr::kMode, {mode}, error))
      {
        return false;
      }
      // 3. Configure acceleration ramp profile (e.g. 50 for arms, 0 for wheels)
      if (!protocol_->write_register(joint.id, sts::register_addr::kAcceleration, {joint.acceleration}, error))
      {
        return false;
      }
    }

    return true;
  }

  bool FeetechBusWorker::execute_anti_jerk_resync(JointStateSnapshot &snapshot, std::string *error) noexcept
  {
    if (!protocol_)
    {
      if (error != nullptr)
      {
        *error = "Serial protocol uninitialized";
      }
      return false;
    }

    std::vector<ServoFastState> initial_states;
    if (!protocol_->sync_read_fast_state(topology_.all_ids, &initial_states, error))
    {
      return false;
    }

    const size_t num_joints = topology_.size();
    if (snapshot.positions.size() != num_joints)
    {
      snapshot.positions.resize(num_joints);
      snapshot.velocities.resize(num_joints);
      snapshot.telemetry.resize(num_joints);
    }

    for (size_t i = 0; i < num_joints; ++i)
    {
      const auto &joint = topology_.joints[i];
      const double pos_rad = (initial_states[i].position_ticks - kEncoderCenterTicks) * kRadiansPerEncoderTick;
      const double vel_scale = joint.velocity_command ? joint.velocity_radians_per_second_per_tick : 0.0;
      const double vel_rad_s = initial_states[i].speed_ticks * vel_scale * joint.velocity_direction;

      snapshot.positions[i] = pos_rad;
      snapshot.velocities[i] = vel_rad_s;
      snapshot.telemetry[i].name = joint.name;
      snapshot.telemetry[i].id = joint.id;
      snapshot.telemetry[i].position_radians = pos_rad;
      snapshot.telemetry[i].velocity_radians_per_second = vel_rad_s;

      const double init_cmd = joint.velocity_command ? 0.0 : pos_rad;
      command_buffer_[i].store(init_cmd, std::memory_order_relaxed);
      torque_enable_buffer_[i].store(1.0, std::memory_order_relaxed);
      active_torque_states_[i] = 1U;
    }

    snapshot.valid = true;
    snapshot.last_read_time = std::chrono::steady_clock::now();

    // Enable torque on all joints
    const std::vector<bool> enable_states(topology_.all_ids.size(), true);
    if (!protocol_->sync_write_torque(topology_.all_ids, enable_states, error))
    {
      return false;
    }

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

    std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
    protocol_ = std::make_unique<StsProtocol>();
    if (!protocol_->open(usb_port_, baud_rate_, timeout_ms_, error))
    {
      protocol_.reset();
      transition_to(sts::BusState::DISCONNECTED, sts::dtc::kDtcPortOpenError);
      return false;
    }

    if (!rearm_servo_hardware(error))
    {
      protocol_->close();
      protocol_.reset();
      transition_to(sts::BusState::DISCONNECTED, sts::dtc::kDtcServoConfigFailed);
      return false;
    }

    transition_to(sts::BusState::STANDBY, sts::dtc::kDtcNone);
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

    const size_t num_joints = topology_.size();
    JointStateSnapshot initial_snapshot;
    initial_snapshot.positions.resize(num_joints);
    initial_snapshot.velocities.resize(num_joints);
    initial_snapshot.telemetry.resize(num_joints);

    {
      std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
      if (!execute_anti_jerk_resync(initial_snapshot, error))
      {
        transition_to(sts::BusState::DISCONNECTED, sts::dtc::kDtcAntiJerkResyncFailed);
        return false;
      }
    }

    rt_state_buffer_.initRT(initial_snapshot);

    has_new_command_.store(false, std::memory_order_release);
    has_new_torque_command_.store(false, std::memory_order_release);

    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    last_command_timestamp_ns_.store(now_ns, std::memory_order_release);
    consecutive_read_errors_.store(0, std::memory_order_relaxed);

    // Stop velocity joints before starting loop (stop_velocity_joints safely acquires serial_mutex_)
    (void)stop_velocity_joints(error);

    transition_to(sts::BusState::OPERATIONAL, sts::dtc::kDtcNone);

    // Start Async I/O Worker Thread
    io_running_.store(true, std::memory_order_release);
    io_worker_thread_ = std::thread(&FeetechBusWorker::run_io_loop, this);

    return true;
  }

  bool FeetechBusWorker::deactivate(std::string *error) noexcept
  {
    stop();

    const bool torque_disabled = set_all_torque(false, error);
    for (size_t i = 0; i < active_torque_states_.size(); ++i)
    {
      active_torque_states_[i] = 0U;
    }
    if (protocol_)
    {
      transition_to(sts::BusState::STANDBY, sts::dtc::kDtcNone);
    }
    return torque_disabled;
  }

  void FeetechBusWorker::stop() noexcept
  {
    io_running_.store(false, std::memory_order_release);
    if (io_worker_thread_.joinable())
    {
      io_worker_thread_.join();
    }
    std::string brake_error;
    (void)stop_velocity_joints(&brake_error);
  }

  void FeetechBusWorker::cleanup() noexcept
  {
    stop();
    std::string torque_error;
    (void)set_all_torque(false, &torque_error);
    std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
    if (protocol_)
    {
      protocol_->close();
      protocol_.reset();
    }
    transition_to(sts::BusState::DISCONNECTED, sts::dtc::kDtcNone);
  }

  bool FeetechBusWorker::set_all_torque(const bool enabled, std::string *error) noexcept
  {
    std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
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
    ServoDiagnosticData single_diag;
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
      const auto state = bus_state_.load(std::memory_order_acquire);

      switch (state)
      {
      case sts::BusState::DISCONNECTED:
      {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_state_transition_ >= reconnect_backoff_ms_)
        {
          std::string verify_err;
          if (!StsProtocol::verify_device_node(usb_port_, &verify_err))
          {
            reconnect_backoff_ms_ = std::min(
                reconnect_backoff_ms_ * 2,
                std::chrono::milliseconds(sts::default_config::kReconnectBackoffMaxMs));
            last_state_transition_ = now;
            active_dtc_.store(sts::dtc::kDtcDeviceNodeMissing, std::memory_order_release);
            break;
          }

          std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
          if (protocol_)
          {
            protocol_->close();
          }
          protocol_ = std::make_unique<StsProtocol>();

          std::string reopen_err;
          if (!protocol_->open(usb_port_, baud_rate_, timeout_ms_, &reopen_err))
          {
            RCLCPP_WARN(rclcpp::get_logger("FeetechBusWorker"),
                        "Port open failed on %s: %s. Remaining DISCONNECTED...", usb_port_.c_str(), reopen_err.c_str());
            reconnect_backoff_ms_ = std::min(
                reconnect_backoff_ms_ * 2,
                std::chrono::milliseconds(sts::default_config::kReconnectBackoffMaxMs));
            last_state_transition_ = now;
            active_dtc_.store(sts::dtc::kDtcPortOpenError, std::memory_order_release);
            break;
          }

          std::string rearm_err;
          if (!rearm_servo_hardware(&rearm_err))
          {
            RCLCPP_WARN(rclcpp::get_logger("FeetechBusWorker"),
                        "Servo re-arming failed: %s. Remaining DISCONNECTED...", rearm_err.c_str());
            protocol_->close();
            reconnect_backoff_ms_ = std::min(
                reconnect_backoff_ms_ * 2,
                std::chrono::milliseconds(sts::default_config::kReconnectBackoffMaxMs));
            last_state_transition_ = now;
            active_dtc_.store(sts::dtc::kDtcServoConfigFailed, std::memory_order_release);
            break;
          }

          std::string resync_err;
          if (!execute_anti_jerk_resync(local_snapshot, &resync_err))
          {
            RCLCPP_WARN(rclcpp::get_logger("FeetechBusWorker"),
                        "Anti-Jerk resync failed: %s. Remaining DISCONNECTED...", resync_err.c_str());
            protocol_->close();
            reconnect_backoff_ms_ = std::min(
                reconnect_backoff_ms_ * 2,
                std::chrono::milliseconds(sts::default_config::kReconnectBackoffMaxMs));
            last_state_transition_ = now;
            active_dtc_.store(sts::dtc::kDtcAntiJerkResyncFailed, std::memory_order_release);
            break;
          }

          RCLCPP_INFO(rclcpp::get_logger("FeetechBusWorker"),
                      "Serial reconnect and anti-jerk resynchronization complete. Bus OPERATIONAL!");
          rt_state_buffer_.writeFromNonRT(local_snapshot);
          consecutive_read_errors_.store(0, std::memory_order_relaxed);
          per_window_.fill(0);
          per_error_count_ = 0;
          per_window_filled_ = 0;
          current_per_.store(0.0, std::memory_order_relaxed);
          reconnect_backoff_ms_ = std::chrono::milliseconds(sts::default_config::kReconnectBackoffInitialMs);
          transition_to(sts::BusState::OPERATIONAL, sts::dtc::kDtcNone);
        }
        break;
      }

      case sts::BusState::STANDBY:
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(sts::default_config::kDefaultLoopPeriodMs));
        break;
      }

      case sts::BusState::OPERATIONAL:
      case sts::BusState::DEGRADED:
      {
        // 1. Hardware Read Phase:
        // Always run sync_read_fast_state every cycle (100 Hz, 90 bytes total for 9 joints)
        // to maintain uniform bus timing and deterministic low-latency control loop.
        bool read_success = false;

        {
          std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
          read_success = protocol_->sync_read_fast_state(topology_.all_ids, &fast_states, &error);
        }

        record_cycle_result(read_success);

        if (read_success)
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

          // Round-Robin single servo diagnostics (1 servo per cycle):
          // At 100 Hz with 9 joints, each joint's complete diagnostic telemetry
          // refreshes at 100/9 = 11.1 Hz with negligible bus overhead (~0.29 ms).
          if (num_joints > 0)
          {
            const size_t diag_joint_idx = iteration_count % num_joints;
            const uint8_t diag_servo_id = topology_.joints[diag_joint_idx].id;
            std::string single_diag_err;
            bool diag_ok = false;
            {
              std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
              diag_ok = protocol_->read_servo_diagnostics(diag_servo_id, &single_diag, &single_diag_err);
            }
            if (diag_ok)
            {
              auto &telem = local_snapshot.telemetry[diag_joint_idx];
              telem.load_ratio = static_cast<double>(single_diag.load_raw) * sts::telemetry_scale::kLoadNormalizedPerUnit;
              telem.voltage_v = single_diag.voltage_v;
              telem.temperature_c = single_diag.temperature_c;
              telem.current_a = single_diag.current_a;
              telem.moving = single_diag.moving;
              telem.status_flags = single_diag.status;
            }
          }

          local_snapshot.valid = true;
          local_snapshot.last_read_time = std::chrono::steady_clock::now();
          ++local_snapshot.update_count;
          consecutive_read_errors_.store(0, std::memory_order_relaxed);

          // PER evaluation for Degraded mode
          const double current_per = current_per_.load(std::memory_order_relaxed);
          if (current_per >= sts::default_config::kPerDegradedThreshold && state == sts::BusState::OPERATIONAL)
          {
            transition_to(sts::BusState::DEGRADED, sts::dtc::kDtcHighPacketErrorRate);
          }
          else if (current_per <= sts::default_config::kPerRecoverThreshold && state == sts::BusState::DEGRADED)
          {
            transition_to(sts::BusState::OPERATIONAL, sts::dtc::kDtcNone);
          }

          rt_state_buffer_.writeFromNonRT(local_snapshot);
        }
        else
        {
          const uint64_t err_count = consecutive_read_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
          ++local_snapshot.read_error_count;

          {
            std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
            if (protocol_)
            {
              protocol_->flush_input();
            }
          }

          // Check if hardware file descriptor or kernel USB connection died
          bool port_dead = false;
          {
            std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
            port_dead = (!protocol_ || !protocol_->is_port_alive());
          }

          if (port_dead || err_count >= sts::default_config::kMaxConsecutiveErrorsBeforePortDrop)
          {
            local_snapshot.valid = false;
            for (size_t i = 0; i < num_joints; ++i)
            {
              local_snapshot.velocities[i] = 0.0;
              local_snapshot.telemetry[i].velocity_radians_per_second = 0.0;
            }
            rt_state_buffer_.writeFromNonRT(local_snapshot);

            RCLCPP_WARN(rclcpp::get_logger("FeetechBusWorker"),
                        "Bus port lost or %zu consecutive read errors! Dropping to DISCONNECTED...",
                        static_cast<size_t>(err_count));

            std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
            if (protocol_)
            {
              protocol_->close();
            }
            transition_to(sts::BusState::DISCONNECTED, port_dead ? sts::dtc::kDtcDeviceNodeMissing : sts::dtc::kDtcBusUnresponsive);
            break;
          }

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
              if (desired_torque && !topology_.joints[i].velocity_command && local_snapshot.valid)
              {
                command_buffer_[i].store(local_snapshot.positions[i], std::memory_order_relaxed);
              }
              active_torque_states_[i] = desired_torque ? 1U : 0U;
            }
          }

          if (!torque_change_ids.empty())
          {
            std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
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
              const auto &joint = topology_.joints[p_idx];
              active_pos_ids.push_back(joint.id);
              const double pos = current_cmds[p_idx];
              const int ticks = joint.has_position_limits
                  ? radians_to_clamped_ticks(pos, joint.min_position_radians, joint.max_position_radians)
                  : radians_to_clamped_ticks(pos, -M_PI, M_PI);
              position_commands.push_back(ticks);
            }
          }

          std::lock_guard<std::recursive_mutex> lock_serial(serial_mutex_);
          if (!active_vel_ids.empty() && !velocity_commands.empty())
          {
            protocol_->sync_write_velocity(active_vel_ids, velocity_commands, &error);
          }
          if (!active_pos_ids.empty() && !position_commands.empty())
          {
            protocol_->sync_write_position(active_pos_ids, position_commands, &error);
          }
        }
        break;
      }
      } // end switch

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

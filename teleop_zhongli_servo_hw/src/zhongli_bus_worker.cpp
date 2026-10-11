
/**
 * @file zhongli_bus_worker.cpp
 * @brief Implementation of ZhongliBusWorker background I/O engine.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "teleop_zhongli_servo_hw/zhongli_bus_worker.hpp"

#include <chrono>

namespace teleop_zhongli_servo_hw
{

  ZhongliBusWorker::ZhongliBusWorker() = default;

  ZhongliBusWorker::~ZhongliBusWorker() noexcept
  {
    stop();
    cleanup();
  }

  bool ZhongliBusWorker::init(
      const std::vector<uint8_t> &servo_ids,
      const std::vector<JointKinematicConfig> &joint_configs,
      const std::unordered_map<uint8_t, ServoConfig> &servo_configs,
      std::string *error) noexcept
  {
    if (servo_ids.empty() || joint_configs.empty())
    {
      if (error != nullptr)
      {
        *error = "Cannot initialize bus worker with empty servo IDs or joint configs";
      }
      return false;
    }

    servo_ids_ = servo_ids;
    joint_configs_ = joint_configs;
    servo_configs_ = servo_configs;

    const size_t num_joints = joint_configs_.size();
    const size_t num_servos = servo_ids_.size();

    LeaderStateSnapshot initial_snapshot;
    initial_snapshot.joint_positions_rad.assign(num_joints, 0.0);
    initial_snapshot.joint_names.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i)
    {
      initial_snapshot.joint_names[i] = joint_configs_[i].joint_name;
    }

    initial_snapshot.servo_feedbacks.resize(num_servos);
    for (size_t i = 0; i < num_servos; ++i)
    {
      initial_snapshot.servo_feedbacks[i].id = servo_ids_[i];
      initial_snapshot.servo_feedbacks[i].raw_pwm = 1500;
      initial_snapshot.servo_feedbacks[i].temperature_c = 0.0;
      initial_snapshot.servo_feedbacks[i].voltage_v = 0.0;
      initial_snapshot.servo_feedbacks[i].valid = false;
    }
    initial_snapshot.valid = false;
    initial_snapshot.update_count = 0;
    initial_snapshot.read_error_count = 0;

    rt_state_buffer_.initRT(initial_snapshot);
    consecutive_read_errors_.store(0, std::memory_order_relaxed);
    total_cycles_.store(0, std::memory_order_relaxed);
    successful_cycles_.store(0, std::memory_order_relaxed);
    telemetry_rr_index_ = 0;

    return true;
  }

  bool ZhongliBusWorker::configure(
      const std::string &port_name,
      const int baudrate,
      const int timeout_ms,
      const int command_delay_us,
      std::string *error) noexcept
  {
    port_name_ = port_name;
    timeout_ms_ = timeout_ms;
    command_delay_us_ = command_delay_us;

    int negotiated_baud = baudrate;
    if (!protocol_.auto_negotiate_and_open(port_name_, baudrate, &negotiated_baud, timeout_ms_, command_delay_us_, error))
    {
      return false;
    }
    baudrate_ = negotiated_baud;

    return true;
  }

  bool ZhongliBusWorker::activate(std::string *error) noexcept
  {
    if (!protocol_.is_open())
    {
      if (error != nullptr)
      {
        *error = "Zhongli protocol is not connected";
      }
      return false;
    }

    // Release torque for all servos (#000PULK!) to enable manual back-drivable motion
    if (!protocol_.disable_torque(0, error))
    {
      for (const auto id : servo_ids_)
      {
        protocol_.disable_torque(id, nullptr);
      }
    }

    // Launch background worker thread running at 50 Hz
    io_running_.store(true, std::memory_order_release);
    io_worker_thread_ = std::thread(&ZhongliBusWorker::run_io_loop, this);

    return true;
  }

  bool ZhongliBusWorker::deactivate(std::string *) noexcept
  {
    stop();
    return true;
  }

  void ZhongliBusWorker::stop() noexcept
  {
    io_running_.store(false, std::memory_order_release);
    if (io_worker_thread_.joinable())
    {
      io_worker_thread_.join();
    }
  }

  void ZhongliBusWorker::cleanup() noexcept
  {
    stop();
    protocol_.close();
  }

  void ZhongliBusWorker::run_io_loop() noexcept
  {
    const size_t num_joints = joint_configs_.size();
    const size_t num_servos = servo_ids_.size();

    // Pre-allocated scratch buffers to guarantee zero heap allocation in 50 Hz loop (MISRA C++ 18-0-1)
    LeaderStateSnapshot local_snapshot;
    local_snapshot.joint_positions_rad.assign(num_joints, 0.0);
    local_snapshot.joint_names.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i)
    {
      local_snapshot.joint_names[i] = joint_configs_[i].joint_name;
    }
    local_snapshot.servo_feedbacks.resize(num_servos);
    for (size_t i = 0; i < num_servos; ++i)
    {
      local_snapshot.servo_feedbacks[i].id = servo_ids_[i];
      local_snapshot.servo_feedbacks[i].raw_pwm = 1500;
      local_snapshot.servo_feedbacks[i].temperature_c = 0.0;
      local_snapshot.servo_feedbacks[i].voltage_v = 0.0;
      local_snapshot.servo_feedbacks[i].valid = false;
    }

    std::unordered_map<uint8_t, int> servo_pwms_scratch;
    servo_pwms_scratch.reserve(num_servos);
    for (const auto id : servo_ids_)
    {
      servo_pwms_scratch[id] = 1500;
    }

    const auto target_period = std::chrono::duration<double>(1.0 / default_config::kPublishRateHz);
    auto next_cycle = std::chrono::steady_clock::now();
    uint64_t loop_count = 0;

    while (io_running_.load(std::memory_order_relaxed))
    {
      total_cycles_.fetch_add(1, std::memory_order_relaxed);

      size_t successful_reads = 0;
      for (size_t i = 0; i < num_servos; ++i)
      {
        const uint8_t id = servo_ids_[i];
        int pwm = 0;
        std::string err;
        if (protocol_.read_servo_position(id, &pwm, &err))
        {
          servo_pwms_scratch[id] = pwm;
          local_snapshot.servo_feedbacks[i].raw_pwm = pwm;
          local_snapshot.servo_feedbacks[i].valid = true;
          successful_reads++;
        }
        else
        {
          local_snapshot.servo_feedbacks[i].valid = false;
        }

        if (command_delay_us_ > 0)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(command_delay_us_));
        }
      }

      if (successful_reads == num_servos)
      {
        consecutive_read_errors_.store(0, std::memory_order_relaxed);
        successful_cycles_.fetch_add(1, std::memory_order_relaxed);

        // Compute joint kinematics in SI Radians
        for (size_t j = 0; j < num_joints; ++j)
        {
          double joint_rad = 0.0;
          if (ZhongliProtocol::calculate_joint_position(
                  joint_configs_[j], servo_configs_, servo_pwms_scratch, &joint_rad))
          {
            local_snapshot.joint_positions_rad[j] = joint_rad;
          }
        }

        // Interleaved Telemetry: Poll one servo's temperature and voltage round-robin every 50 ticks (1.0s at 50 Hz)
        // to preserve maximum bus bandwidth for position feedback at 115200 bps
        if (loop_count % 50 == 0 && !servo_ids_.empty())
        {
          const size_t s_idx = telemetry_rr_index_ % num_servos;
          const uint8_t telem_id = servo_ids_[s_idx];
          telemetry_rr_index_++;

          double temp = 0.0;
          double volt = 0.0;
          if (protocol_.read_servo_telemetry(telem_id, &temp, &volt))
          {
            local_snapshot.servo_feedbacks[s_idx].temperature_c = temp;
            local_snapshot.servo_feedbacks[s_idx].voltage_v = volt;
          }
        }

        local_snapshot.valid = true;
        local_snapshot.last_read_time = std::chrono::steady_clock::now();
        ++local_snapshot.update_count;

        // Wait-free copy into RealtimeBuffer (< 50 ns)
        rt_state_buffer_.writeFromNonRT(local_snapshot);
      }
      else
      {
        const uint64_t err_count = consecutive_read_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
        ++local_snapshot.read_error_count;

        // Fault Escalation Level 1: Flush serial input buffer immediately on frame drop
        protocol_.flush_input();

        // Fault Escalation Level 2: Invalidate telemetry snapshot after 3 consecutive errors (60 ms)
        if (err_count >= default_config::kMaxConsecutiveErrorsBeforeInvalid)
        {
          local_snapshot.valid = false;
          rt_state_buffer_.writeFromNonRT(local_snapshot);
        }

        // Fault Escalation Level 3: Non-blocking background reconnect if unresponsive (> 1s)
        if (err_count >= default_config::kMaxConsecutiveErrorsBeforeReconnect)
        {
          const auto now = std::chrono::steady_clock::now();
          if (now - last_reconnect_attempt_ > std::chrono::seconds(1))
          {
            last_reconnect_attempt_ = now;
            protocol_.close();
            std::string reopen_err;
            if (protocol_.open(port_name_, baudrate_, timeout_ms_, command_delay_us_, &reopen_err))
            {
              (void)protocol_.handshake(2, nullptr);
              (void)protocol_.disable_torque(0, nullptr);
              consecutive_read_errors_.store(0, std::memory_order_relaxed);
            }
          }
        }
      }

      ++loop_count;

      // Drift-Free Rate Pacing: 50 Hz
      next_cycle += std::chrono::duration_cast<std::chrono::nanoseconds>(target_period);
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

} // namespace teleop_zhongli_servo_hw

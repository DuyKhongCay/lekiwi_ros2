/**
 * @file zhongli_bus_worker.hpp
 * @brief Background asynchronous I/O worker and real-time buffer engine for Zhongli bus servos.
 *
 * Encapsulates 115200 baud ASCII serial polling at 50 Hz, wait-free real-time buffers (< 50 ns),
 * zero-allocation scratch buffers in the control loop, interleaved diagnostics, and 3-tier fault escalation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "teleop_zhongli_servo_hw/realtime_buffer.hpp"
#include "teleop_zhongli_servo_hw/zhongli_protocol.hpp"
#include "teleop_zhongli_servo_hw/zhongli_types.hpp"

namespace teleop_zhongli_servo_hw
{

  /**
   * @brief High-rate asynchronous background I/O worker managing Zhongli ASCII serial bus.
   */
  class ZhongliBusWorker
  {
  public:
    ZhongliBusWorker();
    ~ZhongliBusWorker() noexcept;

    // Non-copyable, non-movable due to thread and atomic members
    ZhongliBusWorker(const ZhongliBusWorker &) = delete;
    ZhongliBusWorker &operator=(const ZhongliBusWorker &) = delete;
    ZhongliBusWorker(ZhongliBusWorker &&) = delete;
    ZhongliBusWorker &operator=(ZhongliBusWorker &&) = delete;

    /**
     * @brief Pre-allocates buffers, configures kinematics and servo mapping.
     */
    bool init(
        const std::vector<uint8_t> &servo_ids,
        const std::vector<JointKinematicConfig> &joint_configs,
        const std::unordered_map<uint8_t, ServoConfig> &servo_configs,
        std::string *error = nullptr) noexcept;

    /**
     * @brief Connects to serial bus at 115200 baud and validates handshake.
     */
    bool configure(
        const std::string &port_name,
        int baudrate = default_config::kBaudRate,
        int timeout_ms = default_config::kTimeoutMs,
        int command_delay_us = default_config::kCommandDelayUs,
        std::string *error = nullptr) noexcept;

    /**
     * @brief Releases torque across all servos and launches background I/O worker thread.
     */
    bool activate(std::string *error = nullptr) noexcept;

    /**
     * @brief Stops async thread, shuts down bus worker cleanly.
     */
    bool deactivate(std::string *error = nullptr) noexcept;

    /**
     * @brief Stops worker thread and joins it.
     */
    void stop() noexcept;

    /**
     * @brief Closes serial port and cleans up resources.
     */
    void cleanup() noexcept;

    /**
     * @brief Returns true if worker thread is actively spinning.
     */
    [[nodiscard]] bool is_running() const noexcept
    {
      return io_running_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Wait-free read from RealtimeBuffer (< 50 ns).
     */
    [[nodiscard]] const LeaderStateSnapshot *read_state_rt() const noexcept
    {
      return rt_state_buffer_.readFromRT();
    }

    /**
     * @brief Non-RT read from RealtimeBuffer for diagnostics/logging.
     */
    [[nodiscard]] const LeaderStateSnapshot *read_state_non_rt() const noexcept
    {
      return rt_state_buffer_.readFromNonRT();
    }

    /**
     * @brief Access consecutive error count for diagnostics.
     */
    [[nodiscard]] uint64_t consecutive_errors() const noexcept
    {
      return consecutive_read_errors_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Access total cycles executed.
     */
    [[nodiscard]] uint64_t total_cycles() const noexcept
    {
      return total_cycles_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Access successful cycles executed.
     */
    [[nodiscard]] uint64_t successful_cycles() const noexcept
    {
      return successful_cycles_.load(std::memory_order_relaxed);
    }

  private:
    /**
     * @brief Continuous I/O loop running at 50 Hz in background thread.
     */
    void run_io_loop() noexcept;

    std::vector<uint8_t> servo_ids_;
    std::vector<JointKinematicConfig> joint_configs_;
    std::unordered_map<uint8_t, ServoConfig> servo_configs_;

    std::string port_name_{"/dev/uarm_leader"};
    int baudrate_{default_config::kBaudRate};
    int timeout_ms_{default_config::kTimeoutMs};
    int command_delay_us_{default_config::kCommandDelayUs};

    ZhongliProtocol protocol_;
    std::thread io_worker_thread_;
    std::atomic<bool> io_running_{false};

    mutable RealtimeBuffer<LeaderStateSnapshot> rt_state_buffer_;

    std::atomic<uint64_t> consecutive_read_errors_{0};
    std::atomic<uint64_t> total_cycles_{0};
    std::atomic<uint64_t> successful_cycles_{0};
    std::chrono::steady_clock::time_point last_reconnect_attempt_{};

    // Telemetry round-robin state
    size_t telemetry_rr_index_{0};
  };

} // namespace teleop_zhongli_servo_hw

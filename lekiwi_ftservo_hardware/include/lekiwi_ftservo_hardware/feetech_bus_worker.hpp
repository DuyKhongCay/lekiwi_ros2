/**
 * @file feetech_bus_worker.hpp
 * @brief Background asynchronous I/O worker and safety engine for Feetech STS serial bus.
 *
 * Encapsulates high-rate (~100 Hz) RS485/TTL communication, zero-allocation pre-allocated
 * scratch vectors, wait-free real-time buffers, 3-tier fault escalation, and deadman safety.
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
#include <vector>

#include <realtime_tools/realtime_buffer.hpp>

#include "lekiwi_ftservo_hardware/joint_config_parser.hpp"
#include "lekiwi_ftservo_hardware/joint_state_snapshot.hpp"
#include "lekiwi_ftservo_hardware/sts_constants.hpp"
#include "lekiwi_ftservo_hardware/sts_protocol.hpp"

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief High-performance asynchronous background I/O worker managing STS protocol bus.
   */
  class FeetechBusWorker
  {
  public:
    FeetechBusWorker();
    ~FeetechBusWorker() noexcept;

    // Non-copyable, non-movable due to thread and atomic members
    FeetechBusWorker(const FeetechBusWorker &) = delete;
    FeetechBusWorker &operator=(const FeetechBusWorker &) = delete;
    FeetechBusWorker(FeetechBusWorker &&) = delete;
    FeetechBusWorker &operator=(FeetechBusWorker &&) = delete;

    /**
     * @brief Initializes buffers and verifies configuration.
     *
     * @param[in] topology Validated joint topology from JointConfigParser.
     */
    bool init(const JointTopology &topology, std::string *error) noexcept;

    /**
     * @brief Connects to serial bus and configures servo operating modes and acceleration.
     *
     * @param[in] usb_port Path to serial TTY device.
     * @param[in] baud_rate Serial baud rate in bps.
     * @param[in] timeout_ms Per-transaction timeout in ms.
     * @param[out] error Optional error message output on failure.
     * @return true on success, false on error.
     */
    bool configure(
        const std::string &usb_port,
        int baud_rate,
        int timeout_ms,
        std::string *error) noexcept;

    /**
     * @brief Performs initial synchronous state query, enables torque, and starts async I/O thread.
     *
     * @param[out] error Output string on failure.
     * @return true on success, false on failure.
     */
    bool activate(std::string *error) noexcept;

    /**
     * @brief Stops async thread, brakes velocity joints to 0, and safely disables motor torque.
     *
     * @param[out] error Output string on failure.
     * @return true if cleanly shutdown, false otherwise.
     */
    bool deactivate(std::string *error) noexcept;

    /**
     * @brief Stops the background worker thread if running and joins it.
     */
    void stop() noexcept;

    /**
     * @brief Closes serial port and cleans up all allocated protocol resources.
     */
    void cleanup() noexcept;

    /**
     * @brief Returns true if background worker thread is actively running.
     */
    [[nodiscard]] bool is_running() const noexcept
    {
      return io_running_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Wait-free read from RealtimeBuffer (< 50 ns).
     */
    [[nodiscard]] const JointStateSnapshot *read_state_rt() const noexcept
    {
      return rt_state_buffer_.readFromRT();
    }

    /**
     * @brief Non-RT read from RealtimeBuffer for diagnostics.
     */
    [[nodiscard]] const JointStateSnapshot *read_state_non_rt() const noexcept
    {
      return rt_state_buffer_.readFromNonRT();
    }

    /**
     * @brief Wait-free command storage into atomic buffer (< 50 ns).
     */
    void push_command(const size_t index, const double command) noexcept
    {
      if (index < command_buffer_.size())
      {
        command_buffer_[index].store(command, std::memory_order_relaxed);
      }
    }

    /**
     * @brief Wait-free torque enable command exchange into atomic buffer (< 50 ns).
     * @return Old torque enable command value.
     */
    double exchange_torque_enable(const size_t index, const double torque_cmd) noexcept
    {
      if (index < torque_enable_buffer_.size())
      {
        return torque_enable_buffer_[index].exchange(torque_cmd, std::memory_order_relaxed);
      }
      return 1.0;
    }

    /**
     * @brief Flags availability of new motion command and updates watchdog timestamp.
     */
    void notify_new_command() noexcept
    {
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
      last_command_timestamp_ns_.store(now_ns, std::memory_order_release);
      has_new_command_.store(true, std::memory_order_release);
    }

    /**
     * @brief Flags availability of new torque configuration command.
     */
    void notify_torque_command_changed() noexcept
    {
      has_new_torque_command_.store(true, std::memory_order_release);
    }

    /**
     * @brief Sends zero velocity (0 ticks) to active velocity joints.
     */
    bool stop_velocity_joints(std::string *error) noexcept;

    /**
     * @brief Enables or disables torque across all servos.
     */
    bool set_all_torque(bool enabled, std::string *error) noexcept;

  private:
    /**
     * @brief Continuous I/O loop running at 100 Hz in background thread.
     */
    void run_io_loop() noexcept;

    JointTopology topology_;
    std::string usb_port_;
    int baud_rate_{sts::default_config::kDefaultBaudRate};
    int timeout_ms_{sts::default_config::kDefaultTimeoutMs};

    std::unique_ptr<StsProtocol> protocol_;
    std::thread io_worker_thread_;
    std::atomic<bool> io_running_{false};

    mutable realtime_tools::RealtimeBuffer<JointStateSnapshot> rt_state_buffer_;
    std::vector<std::atomic<double>> command_buffer_;
    std::vector<std::atomic<double>> torque_enable_buffer_;
    std::atomic<bool> has_new_command_{false};
    std::atomic<bool> has_new_torque_command_{false};
    std::vector<uint8_t> active_torque_states_;

    std::atomic<int64_t> last_command_timestamp_ns_{0};
    std::atomic<uint64_t> consecutive_read_errors_{0};
    std::chrono::steady_clock::time_point last_reconnect_attempt_{};

    mutable std::mutex serial_mutex_;
  };

} // namespace lekiwi_ftservo_hardware

/**
 * @file realtime_buffer.hpp
 * @brief Self-contained, wait-free double buffer for real-time robotic state exchange.
 *
 * Provides a lock-free, wait-free read (< 10 ns) suitable for high-rate RT loops,
 * and a release-consistent atomic write from background I/O threads.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <atomic>
#include <mutex>

namespace teleop_zhongli_servo_hw
{

  /**
   * @brief Wait-free double-buffered snapshot exchange.
   *
   * Reader executes in O(1) time without blocking or lock acquisition.
   * Writer safely updates the offline buffer and swaps the atomic index.
   */
  template <typename T>
  class RealtimeBuffer
  {
  public:
    RealtimeBuffer() : current_index_{0} {}

    explicit RealtimeBuffer(const T &initial) : current_index_{0}
    {
      buffers_[0] = initial;
      buffers_[1] = initial;
    }

    void initRT(const T &initial)
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      buffers_[0] = initial;
      buffers_[1] = initial;
      current_index_.store(0, std::memory_order_release);
    }

    [[nodiscard]] const T *readFromRT() const noexcept
    {
      const size_t idx = current_index_.load(std::memory_order_acquire);
      return &buffers_[idx];
    }

    [[nodiscard]] const T *readFromNonRT() const noexcept
    {
      const size_t idx = current_index_.load(std::memory_order_acquire);
      return &buffers_[idx];
    }

    void writeFromNonRT(const T &value) noexcept
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      const size_t next_idx = 1U - current_index_.load(std::memory_order_relaxed);
      buffers_[next_idx] = value;
      current_index_.store(next_idx, std::memory_order_release);
    }

  private:
    T buffers_[2];
    std::atomic<size_t> current_index_{0};
    std::mutex write_mutex_;
  };

} // namespace teleop_zhongli_servo_hw


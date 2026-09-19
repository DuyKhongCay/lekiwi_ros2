/**
 * @file joint_state_snapshot.hpp
 * @brief Telemetry snapshot structures for Feetech STS servos.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Real-time telemetry snapshot for a single robot joint.
   */
  struct JointTelemetry
  {
    /// Joint name view matching static URDF specification in JointTopology.
    std::string_view name;
    /// Hardware ID of the Feetech servo on the serial bus.
    uint8_t id{0};
    /// Present joint position in SI units (radians).
    double position_radians{0.0};
    /// Present joint angular velocity in SI units (radians per second).
    double velocity_radians_per_second{0.0};
    /// Present motor load ratio (normalized to [-1.0, 1.0]).
    double load_ratio{0.0};
    /// Operating input supply voltage in Volts (V).
    double voltage_v{0.0};
    /// Motor PCB internal temperature in degrees Celsius (°C).
    double temperature_c{0.0};
    /// Motor driving current in Amperes (A).
    double current_a{0.0};
    /// True if servo reports active movement.
    bool moving{false};
    /// Raw hardware status bitmask.
    uint8_t status_flags{0};
  };

  /**
   * @brief Real-time snapshot of joint feedback and telemetry populated by the worker thread.
   */
  struct JointStateSnapshot
  {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<JointTelemetry> telemetry;
    uint64_t update_count{0};
    uint64_t read_error_count{0};
    std::chrono::steady_clock::time_point last_read_time{};
    bool valid{false};
  };

} // namespace lekiwi_ftservo_hardware

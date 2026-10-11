/**
 * @file zhongli_types.hpp
 * @brief Data structures and types for Zhongli serial bus servo communication and kinematics.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace teleop_zhongli_servo_hw
{

  namespace default_config
  {
    /// Standard high-rate bus baud rate (target 1 Mbps).
    inline constexpr int kBaudRate = 1000000;
    /// Fallback bus baud rate for standard USB-UART bridges (115200 bps).
    inline constexpr int kBaudRate115200 = 115200;
    /// Per-transaction timeout in milliseconds for 1 Mbps.
    inline constexpr int kTimeoutMs1M = 20;
    /// Per-transaction timeout in milliseconds for 115200 baud (safe margin 60 - 80 ms).
    inline constexpr int kTimeoutMs115200 = 70;
    /// Default per-transaction timeout.
    inline constexpr int kTimeoutMs = kTimeoutMs1M;
    /// Inter-command microsecond delay at 1 Mbps.
    inline constexpr int kCommandDelayUs1M = 300;
    /// Inter-command microsecond delay at 115200 bps.
    inline constexpr int kCommandDelayUs115200 = 50;
    /// Default inter-command delay.
    inline constexpr int kCommandDelayUs = kCommandDelayUs1M;
    /// Flash EEPROM commit delay after #PSCK! zero-calibration in milliseconds.
    inline constexpr int kEepromCommitDelayMs = 150;
    /// Default kinematic publishing rate in Hz for teleoperation (50 Hz).
    inline constexpr double kPublishRateHz = 50.0;
    /// Calibration tool interactive sampling rate in Hz (<= 20 Hz).
    inline constexpr double kCalibPublishRateHz = 20.0;
    /// Consecutive read errors before snapshot invalidation.
    inline constexpr uint64_t kMaxConsecutiveErrorsBeforeInvalid = 3;
    /// Consecutive read errors before attempting background serial reconnect.
    inline constexpr uint64_t kMaxConsecutiveErrorsBeforeReconnect = 50;
  } // namespace default_config

  /**
   * @brief Physical servo configuration parameters.
   */
  struct ServoConfig
  {
    /// Hardware ID of the servo on the bus (0..253).
    uint8_t id{0};
    /// Minimum valid raw PWM pulse limit (e.g. 500).
    int min_pwm{500};
    /// Maximum valid raw PWM pulse limit (e.g. 2500).
    int max_pwm{2500};
    /// Conversion factor from PWM deviation to radians.
    /// Default: 270 degrees span over 2000 PWM span (500 to 2500).
    /// rad_per_pwm = (270.0 * M_PI / 180.0) / 2000.0 ~= 0.00235619449 rad/pwm
    double rad_per_pwm{(270.0 * M_PI / 180.0) / 2000.0};
  };

  /**
   * @brief Weight and direction of a single servo contributing to a joint.
   */
  struct ServoContribution
  {
    /// Source servo ID.
    uint8_t servo_id{0};
    /// Weight/scale factor for differential or multi-servo setups (default 1.0).
    double weight{1.0};
    /// Motion direction sign: +1.0 (normal) or -1.0 (inverted).
    double sign{1.0};
  };

  /**
   * @brief Kinematic configuration mapping one or more servos to a ROS 2 joint (SI Radians).
   *
   * Note: The servo hardware midpoint 1500 represents 0.0 rad.
   */
  struct JointKinematicConfig
  {
    /// ROS 2 Joint name (e.g. "shoulder_pan", "wrist_roll", "gripper").
    std::string joint_name;
    /// Contributing servos and their weights/signs.
    std::vector<ServoContribution> sources;
    /// Optional joint range limits for validation / clamping.
    double min_limit_rad{-M_PI};
    double max_limit_rad{M_PI};
  };

  /**
   * @brief Telemetry feedback snapshot for a single servo.
   */
  struct ServoFeedback
  {
    /// Hardware ID of the servo.
    uint8_t id{0};
    /// Present raw position in PWM units (typically 500..2500, nominal midpoint 1500).
    int raw_pwm{1500};
    /// Internal temperature in degrees Celsius (from #PRTV!).
    double temperature_c{0.0};
    /// Operating voltage in Volts (from #PRTV!).
    double voltage_v{0.0};
    /// True if data was successfully received and parsed.
    bool valid{false};
  };

  /**
   * @brief Calculated joint state ready for ROS 2 publication.
   */
  struct JointStateData
  {
    std::string name;
    double position_rad{0.0};
  };

  /**
   * @brief Wait-free atomic/real-time state snapshot across all servos and joints.
   */
  struct LeaderStateSnapshot
  {
    /// Computed joint angles in SI Radians.
    std::vector<double> joint_positions_rad;
    /// Joint names corresponding to joint_positions_rad.
    std::vector<std::string> joint_names;
    /// Individual servo feedbacks (PWM, temperature, voltage).
    std::vector<ServoFeedback> servo_feedbacks;
    /// Timestamp of most recent successful bus read.
    std::chrono::steady_clock::time_point last_read_time{};
    /// Monotonically increasing cycle counter.
    uint64_t update_count{0};
    /// Read error counter.
    uint64_t read_error_count{0};
    /// True if data is fresh and valid.
    bool valid{false};
  };

} // namespace teleop_zhongli_servo_hw

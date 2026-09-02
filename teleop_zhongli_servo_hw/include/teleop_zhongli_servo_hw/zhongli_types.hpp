/**
 * @file zhongli_types.hpp
 * @brief Data structures and types for Zhongli serial bus servo communication and kinematics.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace teleop_zhongli_servo_hw
{

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

} // namespace teleop_zhongli_servo_hw

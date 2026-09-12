/**
 * @file sts_constants.hpp
 * @brief Unified hardware constants, register map, and configuration defaults for Feetech STS3215 servos.
 *
 * Provides a single source of truth (SSOT) for protocol opcodes, control table registers,
 * physical unit conversions, encoder resolution, and LeKiwi robot operational defaults.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace lekiwi_ftservo_hardware::sts
{

  /**
   * @brief Feetech STS serial packet framing constants and instruction opcodes.
   */
  namespace protocol
  {
    /// Magic packet preamble byte (0xFF).
    constexpr uint8_t kHeader = 0xFF;
    /// Broadcast ID addressing all servos on the bus (0xFE).
    constexpr uint8_t kBroadcastId = 0xFE;

    /// Instruction: Query servo status / presence.
    constexpr uint8_t kInstructionPing = 0x01;
    /// Instruction: Direct read from control table registers.
    constexpr uint8_t kInstructionRead = 0x02;
    /// Instruction: Direct write to control table registers.
    constexpr uint8_t kInstructionWrite = 0x03;
    /// Instruction: Staged register write (applied upon Action opcode).
    constexpr uint8_t kInstructionRegWrite = 0x04;
    /// Instruction: Trigger staged writes.
    constexpr uint8_t kInstructionAction = 0x05;
    /// Instruction: Synchronized multi-servo read.
    constexpr uint8_t kInstructionSyncRead = 0x82;
    /// Instruction: Synchronized multi-servo write in a single frame.
    constexpr uint8_t kInstructionSyncWrite = 0x83;
  } // namespace protocol

  /**
   * @brief Feetech STS3215 memory control table register addresses (SRAM / EEPROM).
   */
  namespace register_addr
  {
    /// Operating mode configuration register (0: Position, 1: Velocity, 2: PWM, 3: Step).
    constexpr uint8_t kMode = 33;
    /// Motor driving torque enable switch (0: Disabled, 1: Enabled).
    constexpr uint8_t kTorqueEnable = 40;
    /// Acceleration profile register (0: Step response / disabled ramp, 1..254: Hardware ramp limit).
    constexpr uint8_t kAcceleration = 41;
    /// Target goal position register (2 bytes, little-endian).
    constexpr uint8_t kGoalPosition = 42;
    /// Target goal speed / velocity register (2 bytes, sign-magnitude).
    constexpr uint8_t kGoalSpeed = 46;
    /// EEPROM write protection lock register (0: Unlocked, 1: Locked).
    constexpr uint8_t kLock = 55;
    /// Present position feedback register (2 bytes, 0..4095).
    constexpr uint8_t kPresentPosition = 56;
    /// Present speed feedback register (2 bytes, sign-magnitude).
    constexpr uint8_t kPresentSpeed = 58;
    /// Present motor load feedback register (2 bytes, 0.1% resolution).
    constexpr uint8_t kPresentLoad = 60;
    /// Present supply voltage register (1 byte, in 0.1V units).
    constexpr uint8_t kPresentVoltage = 62;
    /// Present internal temperature register (1 byte, in 1°C units).
    constexpr uint8_t kPresentTemperature = 63;
    /// Movement status indicator register (1 byte, 1: Moving, 0: Stationary).
    constexpr uint8_t kMoving = 66;
    /// Present motor drive current register (2 bytes, in 6.5mA units).
    constexpr uint8_t kPresentCurrent = 69;
  } // namespace register_addr

  /**
   * @brief Operating mode definitions for STS3215 servos.
   */
  namespace mode
  {
    /// Position control mode (closed-loop servo position with magnetic encoder).
    constexpr uint8_t kPosition = 0;
    /// Continuous velocity / wheel speed mode.
    constexpr uint8_t kVelocity = 1;
    /// Pulse Width Modulation mode.
    constexpr uint8_t kPWM = 2;
    /// Step mode.
    constexpr uint8_t kStep = 3;
  } // namespace mode

  /**
   * @brief STS3215 encoder resolution, velocity encoding bitmasks, and register limits.
   */
  namespace resolution
  {
    /// 12-bit magnetic encoder resolution: 4096 ticks per complete 360° turn.
    constexpr int kEncoderResolution = 4096;
    /// Mid-point encoder tick corresponding to center / zero angle (2048).
    constexpr int kEncoderCenter = kEncoderResolution / 2;
    /// Conversion multiplier from encoder ticks to radians: (2 * PI) / 4096.
    constexpr double kRadiansPerEncoderTick = (2.0 * M_PI) / kEncoderResolution;
    /// Conversion multiplier from radians to encoder ticks: 4096 / (2 * PI).
    constexpr double kEncoderTicksPerRadian = kEncoderResolution / (2.0 * M_PI);

    /// Sign bit position in STS 16-bit velocity word (Bit 15).
    constexpr int kVelocitySignBit = 15;
    /// Maximum tick magnitude expressible in 15-bit magnitude field (0x7FFF = 32767).
    constexpr int kMaxVelocityTicks = (1 << kVelocitySignBit) - 1;

    /// Maximum valid value for STS acceleration register 41.
    constexpr uint8_t kMaxAccelerationRegister = 254;
  } // namespace resolution

  /**
   * @brief Conversion multipliers from raw hardware register units to standard SI units.
   */
  namespace telemetry_scale
  {
    /// Supply voltage scaling: Register 62 reports in units of 0.1 V.
    constexpr double kVoltsPerUnit = 0.1;
    /// Drive current scaling: Register 69 reports in units of 6.5 mA (0.0065 A).
    constexpr double kAmperesPerUnit = 0.0065;
    /// Motor load normalization: Register 60 reports in units of 0.1% (0.001 ratio).
    constexpr double kLoadNormalizedPerUnit = 0.001;
  } // namespace telemetry_scale

  /**
   * @brief Default configuration and diagnostic safety thresholds for LeKiwi robot.
   */
  namespace default_config
  {
    /// Base omni wheels acceleration: 0 enforces immediate step response (hardware ramp disabled).
    constexpr uint8_t kWheelAcceleration = 0U;
    /// Follower arm joints default acceleration: 50 ensures smooth motion and protects 3D printed joints.
    constexpr uint8_t kDefaultArmAcceleration = 50U;

    /// Diagnostic freshness limit before flagging telemetry as stale (milliseconds).
    constexpr int64_t kTelemetryStaleTimeoutMs = 200;
    /// Temperature warning threshold in degrees Celsius.
    constexpr double kServoTempWarnLimitC = 60.0;
    /// Temperature critical error threshold in degrees Celsius.
    constexpr double kServoTempErrorLimitC = 70.0;
    /// Minimum safe battery voltage threshold (3S LiPo: 3.17V per cell = ~9.5V).
    constexpr double kBatteryLowVoltageLimitV = 9.5;
    /// Maximum allowable bus voltage threshold (3S LiPo full charge: 12.6V, margin to 13.5V).
    constexpr double kBatteryHighVoltageLimitV = 13.5;

    /// Default serial bus baud rate in bps.
    constexpr int kDefaultBaudRate = 1000000;
    /// Default serial read timeout in milliseconds.
    constexpr int kDefaultTimeoutMs = 20;
    /// Target control / IO worker loop period in milliseconds (10 ms = 100 Hz).
    constexpr int kDefaultLoopPeriodMs = 10;
  } // namespace default_config

} // namespace lekiwi_ftservo_hardware::sts

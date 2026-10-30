/**
 * @file joint_codec.hpp
 * @brief Unified encoding, decoding, unit conversion, and limit clamping for Feetech STS servos.
 *
 * This header provides purely computational, zero-allocation inline helper utilities for:
 * 1. Feetech STS discrete velocity units (signed / sign-magnitude ticks) and ROS angular velocity (rad/s).
 * 2. Feetech STS discrete position units (12-bit encoder ticks) and ROS joint position (radians),
 *    including deterministic physical safety clamping within URDF-defined limits.
 *
 * Designed to strictly satisfy MISRA C / embedded C++ real-time constraints: no heap allocations,
 * no exceptions, pure functions with noexcept and deterministic error fallback.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "lekiwi_ftservo_hardware/sts_constants.hpp"

namespace lekiwi_ftservo_hardware
{

  // =========================================================================
  // Velocity Constants & Codec Utilities
  // =========================================================================

  /**
   * @brief Bit index of the direction/sign bit in the Feetech STS 16-bit velocity register.
   */
  constexpr int kStsVelocitySignBit = sts::resolution::kVelocitySignBit;

  /**
   * @brief Maximum tick magnitude expressible in the STS 15-bit magnitude field (0x7FFF = 32767).
   */
  constexpr int kStsVelocityMaxTicks = sts::resolution::kMaxVelocityTicks;

  /**
   * @brief Converts signed STS velocity ticks to the protocol's 2-byte sign-magnitude word.
   *
   * In STS protocol, velocity values use sign-magnitude encoding where bit 15 represents
   * sign (1 = negative/CW, 0 = positive/CCW) and bits [14:0] represent magnitude.
   * Magnitudes exceeding @ref kStsVelocityMaxTicks (32767) are safely clamped without throwing.
   *
   * @param[in] ticks Signed integer velocity in servo tick units.
   * @return std::array<uint8_t, 2> 2-byte array with {low_byte, high_byte} in little-endian order.
   */
  inline std::array<uint8_t, 2> encode_velocity_ticks(const int ticks) noexcept
  {
    const int clamped = std::clamp(ticks, -kStsVelocityMaxTicks, kStsVelocityMaxTicks);
    const int magnitude = std::abs(clamped);
    const int encoded = (clamped < 0) ? (magnitude | (1 << kStsVelocitySignBit)) : magnitude;
    return {static_cast<uint8_t>(encoded & 0xff), static_cast<uint8_t>((encoded >> 8) & 0xff)};
  }

  /**
   * @brief Decodes the STS sign-magnitude 2-byte representation into signed ticks.
   *
   * @param[in] low Low byte from the servo register payload.
   * @param[in] high High byte from the servo register payload containing sign bit at bit 7.
   * @return int Signed velocity in ticks (negative for CW rotation, positive for CCW rotation).
   */
  inline int decode_velocity_ticks(const uint8_t low, const uint8_t high) noexcept
  {
    const int encoded = static_cast<int>(low) | (static_cast<int>(high) << 8);
    const int magnitude = encoded & kStsVelocityMaxTicks;
    return (encoded & (1 << kStsVelocitySignBit)) != 0 ? -magnitude : magnitude;
  }

  /**
   * @brief Converts a bounded ROS angular velocity (rad/s) into a bounded STS tick command.
   *
   * Clamps the input velocity within `[-max_radians_per_second, max_radians_per_second]`,
   * accounts for hardware mounting inversion (@p direction), and scales by the conversion factor.
   * Never throws exceptions; safely falls back to 0 ticks if input is non-finite.
   *
   * @param[in] radians_per_second Commanded angular velocity in rad/s.
   * @param[in] radians_per_second_per_tick Conversion scale from ticks to rad/s (must be > 0.0).
   * @param[in] max_radians_per_second Maximum allowable speed ceiling in rad/s (must be > 0.0).
   * @param[in] direction Direction polarity multiplier (must be either +1 or -1).
   * @return int Signed tick command ready to be encoded via @ref encode_velocity_ticks.
   */
  inline int radians_per_second_to_ticks(
      const double radians_per_second, const double radians_per_second_per_tick,
      const double max_radians_per_second, const int direction) noexcept
  {
    if (!std::isfinite(radians_per_second) ||
        radians_per_second_per_tick <= 0.0 ||
        max_radians_per_second <= 0.0 ||
        (direction != 1 && direction != -1))
    {
      return 0;
    }

    const double bounded = std::clamp(radians_per_second, -max_radians_per_second, max_radians_per_second);
    const double commanded_ticks = std::lround((bounded * static_cast<double>(direction)) / radians_per_second_per_tick);
    return std::clamp(static_cast<int>(commanded_ticks), -kStsVelocityMaxTicks, kStsVelocityMaxTicks);
  }

  // =========================================================================
  // Position Constants & Codec / Safety Clamping Utilities
  // =========================================================================

  /**
   * @brief Clamps a commanded joint position in radians strictly within `[min_rad, max_rad]`.
   *
   * Handles non-finite inputs (NaN, +Inf, -Inf) deterministically by clamping 0.0 within [min_rad, max_rad].
   *
   * @param[in] radians Joint command in radians.
   * @param[in] min_rad Lower bound in radians.
   * @param[in] max_rad Upper bound in radians.
   * @return double Clamped joint angle in radians.
   */
  inline double clamp_position_radians(
      const double radians, const double min_rad, const double max_rad) noexcept
  {
    if (!std::isfinite(radians))
    {
      return std::clamp(0.0, min_rad, max_rad);
    }
    return std::clamp(radians, min_rad, max_rad);
  }

  /**
   * @brief Converts joint position in radians to unconstrained STS discrete encoder ticks.
   *
   * Uses mid-point center tick (2048) and 4096 resolution.
   *
   * @param[in] radians Angular position in radians.
   * @return int Encoder tick value corresponding to input angle.
   */
  inline int radians_to_ticks(const double radians) noexcept
  {
    if (!std::isfinite(radians))
    {
      return sts::resolution::kEncoderCenterTicks;
    }
    return static_cast<int>(std::lround(radians * sts::resolution::kEncoderTicksPerRadian)) +
           sts::resolution::kEncoderCenterTicks;
  }

  /**
   * @brief Converts joint position in radians to strictly clamped STS discrete encoder ticks.
   *
   * Guarantees 2-fold safety:
   * 1. Clamps radian command to `[min_rad, max_rad]`.
   * 2. Encodes to discrete ticks and clamps to 12-bit hardware bounds `[0, kEncoderResolution - 1]`.
   *
   * @param[in] radians Angular position in radians.
   * @param[in] min_rad Lower angle bound in radians.
   * @param[in] max_rad Upper angle bound in radians.
   * @return int Clamped encoder tick in `[0, 4095]`.
   */
  inline int radians_to_clamped_ticks(
      const double radians, const double min_rad, const double max_rad) noexcept
  {
    const double clamped_rad = clamp_position_radians(radians, min_rad, max_rad);
    const int ticks = radians_to_ticks(clamped_rad);
    return std::clamp(ticks, 0, sts::resolution::kEncoderResolution - 1);
  }

  /**
   * @brief Converts discrete STS encoder ticks to ROS continuous joint position in radians.
   *
   * @param[in] ticks Discrete encoder ticks (nominally 0..4095).
   * @return double Position in radians centered at 2048 ticks.
   */
  inline double ticks_to_radians(const int ticks) noexcept
  {
    return static_cast<double>(ticks - sts::resolution::kEncoderCenterTicks) *
           sts::resolution::kRadiansPerEncoderTick;
  }

} // namespace lekiwi_ftservo_hardware


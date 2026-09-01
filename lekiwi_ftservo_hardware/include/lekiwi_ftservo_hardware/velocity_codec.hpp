/**
 * @file velocity_codec.hpp
 * @brief Velocity encoding, decoding, and unit conversion helpers for Feetech STS servos.
 *
 * This header provides purely computational, inline helper utilities for converting
 * between ROS standard angular velocity (radians per second) and Feetech STS series
 * servo discrete velocity units (signed / sign-magnitude ticks).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Bit index of the direction/sign bit in the Feetech STS 16-bit velocity register.
   *
   * Bit 15 indicates rotation direction (0 for CCW / positive magnitude, 1 for CW / negative magnitude).
   */
  constexpr int kStsVelocitySignBit = 15;

  /**
   * @brief Maximum tick magnitude expressible in the STS 15-bit magnitude field (0x7FFF = 32767).
   */
  constexpr int kStsVelocityMaxTicks = (1 << kStsVelocitySignBit) - 1;

  /**
   * @brief Converts signed STS velocity ticks to the protocol's 2-byte sign-magnitude word.
   *
   * In STS protocol, velocity values use sign-magnitude encoding where the highest bit
   * (bit 15) represents sign (1 = negative/CW, 0 = positive/CCW) and bits [14:0] represent magnitude.
   *
   * @param[in] ticks Signed integer velocity in servo tick units.
   * @return std::array<uint8_t, 2> 2-byte array with {low_byte, high_byte} in little-endian order.
   * @throws std::out_of_range If @p std::abs(ticks) exceeds @ref kStsVelocityMaxTicks (32767).
   */
  inline std::array<uint8_t, 2> encode_velocity_ticks(const int ticks)
  {
    if (std::abs(ticks) > kStsVelocityMaxTicks)
    {
      throw std::out_of_range("STS velocity command exceeds sign-magnitude range");
    }
    const int encoded = ticks < 0 ? std::abs(ticks) | (1 << kStsVelocitySignBit) : ticks;
    return {static_cast<uint8_t>(encoded & 0xff), static_cast<uint8_t>((encoded >> 8) & 0xff)};
  }

  /**
   * @brief Decodes the STS sign-magnitude 2-byte representation into signed ticks.
   *
   * @param[in] low Low byte from the servo register payload.
   * @param[in] high High byte from the servo register payload containing sign bit at bit 7 (overall bit 15).
   * @return int Signed velocity in ticks (negative for CW rotation, positive for CCW rotation).
   */
  inline int decode_velocity_ticks(const uint8_t low, const uint8_t high)
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
   *
   * @param[in] radians_per_second Commanded wheel angular velocity in rad/s.
   * @param[in] radians_per_second_per_tick Conversion scale from ticks to rad/s (must be > 0.0).
   * @param[in] max_radians_per_second Maximum allowable speed ceiling in rad/s (must be > 0.0).
   * @param[in] direction Direction polarity multiplier (must be either +1 or -1).
   * @return int Signed tick command ready to be encoded via @ref encode_velocity_ticks.
   * @throws std::invalid_argument If any scale/direction argument is non-positive or not in {-1, 1},
   *         or if @p radians_per_second is not finite (NaN or infinity).
   */
  inline int radians_per_second_to_ticks(
      const double radians_per_second, const double radians_per_second_per_tick,
      const double max_radians_per_second, const int direction)
  {
    if (!std::isfinite(radians_per_second) || radians_per_second_per_tick <= 0.0 ||
        max_radians_per_second <= 0.0 || (direction != -1 && direction != 1))
    {
      throw std::invalid_argument("Invalid wheel velocity conversion parameters");
    }
    const double bounded = std::clamp(
        radians_per_second, -max_radians_per_second, max_radians_per_second);
    return static_cast<int>(std::lround(bounded * static_cast<double>(direction) /
                                        radians_per_second_per_tick));
  }

} // namespace lekiwi_ftservo_hardware

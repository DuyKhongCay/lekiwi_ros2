#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace lekiwi_ftservo_hardware
{

  constexpr int kStsVelocitySignBit = 15;
  constexpr int kStsVelocityMaxTicks = (1 << kStsVelocitySignBit) - 1;

  // Converts signed STS velocity ticks to the protocol's sign-magnitude word.
  inline std::array<uint8_t, 2> encode_velocity_ticks(const int ticks)
  {
    if (std::abs(ticks) > kStsVelocityMaxTicks)
    {
      throw std::out_of_range("STS velocity command exceeds sign-magnitude range");
    }
    const int encoded = ticks < 0 ? std::abs(ticks) | (1 << kStsVelocitySignBit) : ticks;
    return {static_cast<uint8_t>(encoded & 0xff), static_cast<uint8_t>((encoded >> 8) & 0xff)};
  }

  // Decodes the STS sign-magnitude velocity representation into signed ticks.
  inline int decode_velocity_ticks(const uint8_t low, const uint8_t high)
  {
    const int encoded = static_cast<int>(low) | (static_cast<int>(high) << 8);
    const int magnitude = encoded & kStsVelocityMaxTicks;
    return (encoded & (1 << kStsVelocitySignBit)) != 0 ? -magnitude : magnitude;
  }

  // Converts a bounded ROS angular velocity into a bounded STS tick command.
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

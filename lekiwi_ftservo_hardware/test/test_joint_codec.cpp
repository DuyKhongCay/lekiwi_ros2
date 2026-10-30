/**
 * @file test_joint_codec.cpp
 * @brief Unit tests for unified JointCodec (velocity and position encoding, decoding, conversion, and clamping).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "lekiwi_ftservo_hardware/joint_codec.hpp"

namespace lekiwi_ftservo_hardware
{

  // =========================================================================
  // Velocity Codec Tests
  // =========================================================================

  TEST(JointCodecVelocity, EncodesAndDecodesSignedMagnitudeTicks)
  {
    EXPECT_EQ(encode_velocity_ticks(0), (std::array<uint8_t, 2>{0U, 0U}));
    EXPECT_EQ(encode_velocity_ticks(-42), (std::array<uint8_t, 2>{42U, 128U}));
    EXPECT_EQ(encode_velocity_ticks(42), (std::array<uint8_t, 2>{42U, 0U}));
    EXPECT_EQ(decode_velocity_ticks(42U, 128U), -42);
    EXPECT_EQ(decode_velocity_ticks(42U, 0U), 42);
  }

  TEST(JointCodecVelocity, BoundsAndAppliesWheelDirection)
  {
    EXPECT_EQ(radians_per_second_to_ticks(3.0, 0.1, 2.0, 1), 20);
    EXPECT_EQ(radians_per_second_to_ticks(3.0, 0.1, 2.0, -1), -20);
    EXPECT_EQ(radians_per_second_to_ticks(-3.0, 0.1, 2.0, 1), -20);
    EXPECT_EQ(radians_per_second_to_ticks(-3.0, 0.1, 2.0, -1), 20);
  }

  TEST(JointCodecVelocity, HandlesEdgeCasesWithoutThrowing)
  {
    // Non-finite input handling (NaN / Inf) -> safe fallback 0
    EXPECT_EQ(radians_per_second_to_ticks(std::numeric_limits<double>::quiet_NaN(), 0.1, 2.0, 1), 0);
    EXPECT_EQ(radians_per_second_to_ticks(std::numeric_limits<double>::infinity(), 0.1, 2.0, 1), 0);
    EXPECT_EQ(radians_per_second_to_ticks(-std::numeric_limits<double>::infinity(), 0.1, 2.0, 1), 0);

    // Invalid parameters -> safe fallback 0
    EXPECT_EQ(radians_per_second_to_ticks(1.0, 0.0, 2.0, 1), 0);
    EXPECT_EQ(radians_per_second_to_ticks(1.0, -0.1, 2.0, 1), 0);
    EXPECT_EQ(radians_per_second_to_ticks(1.0, 0.1, -2.0, 1), 0);
    EXPECT_EQ(radians_per_second_to_ticks(1.0, 0.1, 2.0, 0), 0);

    // Extreme tick clamping in encode_velocity_ticks (past 32767)
    auto clamped_pos = encode_velocity_ticks(50000);
    int16_t max_ticks = kStsVelocityMaxTicks;
    EXPECT_EQ(clamped_pos[0], static_cast<uint8_t>(max_ticks & 0xFF));
    EXPECT_EQ(clamped_pos[1], static_cast<uint8_t>((max_ticks >> 8) & 0x7F));

    auto clamped_neg = encode_velocity_ticks(-50000);
    EXPECT_EQ(clamped_neg[0], static_cast<uint8_t>(max_ticks & 0xFF));
    EXPECT_EQ(clamped_neg[1], static_cast<uint8_t>(((max_ticks >> 8) & 0x7F) | 0x80));
  }

  // =========================================================================
  // Position Codec & Safety Clamping Tests
  // =========================================================================

  TEST(JointCodecPosition, ClampsPositionRadiansWithinLimits)
  {
    const double min_rad = -1.5;
    const double max_rad = 1.5;

    // Within bounds
    EXPECT_DOUBLE_EQ(clamp_position_radians(0.0, min_rad, max_rad), 0.0);
    EXPECT_DOUBLE_EQ(clamp_position_radians(1.2, min_rad, max_rad), 1.2);
    EXPECT_DOUBLE_EQ(clamp_position_radians(-1.2, min_rad, max_rad), -1.2);

    // Exceeding bounds
    EXPECT_DOUBLE_EQ(clamp_position_radians(3.0, min_rad, max_rad), max_rad);
    EXPECT_DOUBLE_EQ(clamp_position_radians(-3.0, min_rad, max_rad), min_rad);
  }

  TEST(JointCodecPosition, ClampsPositionRadiansNonFiniteSafely)
  {
    const double min_rad = -1.0;
    const double max_rad = 2.0;

    // NaN / Inf safely clamped
    EXPECT_DOUBLE_EQ(clamp_position_radians(std::numeric_limits<double>::quiet_NaN(), min_rad, max_rad), 0.0);
    EXPECT_DOUBLE_EQ(clamp_position_radians(std::numeric_limits<double>::infinity(), min_rad, max_rad), 0.0);
    EXPECT_DOUBLE_EQ(clamp_position_radians(-std::numeric_limits<double>::infinity(), min_rad, max_rad), 0.0);

    // If 0.0 is outside [min_rad, max_rad]
    const double pos_min = 1.0;
    const double pos_max = 2.0;
    EXPECT_DOUBLE_EQ(clamp_position_radians(std::numeric_limits<double>::quiet_NaN(), pos_min, pos_max), pos_min);
  }

  TEST(JointCodecPosition, ConvertsRadiansToTicksAndRoundTrips)
  {
    // Center angle (0 rad) -> 2048 ticks
    EXPECT_EQ(radians_to_ticks(0.0), 2048);
    EXPECT_NEAR(ticks_to_radians(2048), 0.0, 1e-6);

    // Near +PI and -PI
    EXPECT_EQ(radians_to_ticks(M_PI), 4096);
    EXPECT_EQ(radians_to_ticks(-M_PI), 0);

    // Non-finite input falls back to center tick
    EXPECT_EQ(radians_to_ticks(std::numeric_limits<double>::quiet_NaN()), 2048);
    EXPECT_EQ(radians_to_ticks(std::numeric_limits<double>::infinity()), 2048);
  }

  TEST(JointCodecPosition, ConvertsRadiansToClampedTicksWithinHardwareLimits)
  {
    const double min_rad = -1.0; // approx 2048 - 652 = 1396 ticks
    const double max_rad = 1.0;  // approx 2048 + 652 = 2700 ticks

    const int tick_min = radians_to_clamped_ticks(min_rad, min_rad, max_rad);
    const int tick_max = radians_to_clamped_ticks(max_rad, min_rad, max_rad);
    EXPECT_GE(tick_min, 0);
    EXPECT_LE(tick_max, 4095);
    EXPECT_LT(tick_min, tick_max);

    // Out-of-bounds command (e.g. 10.0 rad) clamped to tick_max
    EXPECT_EQ(radians_to_clamped_ticks(10.0, min_rad, max_rad), tick_max);
    // Out-of-bounds command (e.g. -10.0 rad) clamped to tick_min
    EXPECT_EQ(radians_to_clamped_ticks(-10.0, min_rad, max_rad), tick_min);

    // Non-finite command clamped safely
    const int tick_nan = radians_to_clamped_ticks(std::numeric_limits<double>::quiet_NaN(), min_rad, max_rad);
    EXPECT_GE(tick_nan, tick_min);
    EXPECT_LE(tick_nan, tick_max);
  }

} // namespace lekiwi_ftservo_hardware


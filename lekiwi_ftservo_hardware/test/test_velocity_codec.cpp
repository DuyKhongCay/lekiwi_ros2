/**
 * @file test_velocity_codec.cpp
 * @brief Unit tests (L1 verification) for STS velocity codec and unit conversion logic.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <array>

#include <gtest/gtest.h>

#include "lekiwi_ftservo_hardware/velocity_codec.hpp"

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Verifies that velocity sign-magnitude encoding/decoding adheres to Feetech STS bit format.
   */
  TEST(VelocityCodec, EncodesAndDecodesSignedMagnitudeTicks)
  {
    EXPECT_EQ(encode_velocity_ticks(0), (std::array<uint8_t, 2>{0U, 0U}));
    EXPECT_EQ(encode_velocity_ticks(-42), (std::array<uint8_t, 2>{42U, 128U}));
    EXPECT_EQ(encode_velocity_ticks(42), (std::array<uint8_t, 2>{42U, 0U}));
    EXPECT_EQ(decode_velocity_ticks(42U, 128U), -42);
    EXPECT_EQ(decode_velocity_ticks(42U, 0U), 42);
  }

  /**
   * @brief Verifies velocity saturation limits (clamping) and directional polarity multiplication.
   */
  TEST(VelocityCodec, BoundsAndAppliesWheelDirection)
  {
    EXPECT_EQ(radians_per_second_to_ticks(3.0, 0.1, 2.0, 1), 20);
    EXPECT_EQ(radians_per_second_to_ticks(3.0, 0.1, 2.0, -1), -20);
    EXPECT_EQ(radians_per_second_to_ticks(-3.0, 0.1, 2.0, 1), -20);
    EXPECT_EQ(radians_per_second_to_ticks(-3.0, 0.1, 2.0, -1), 20);
  }

  /**
   * @brief Verifies robust error handling without throwing exceptions for invalid or non-finite inputs.
   */
  TEST(VelocityCodec, HandlesEdgeCasesWithoutThrowing)
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

} // namespace lekiwi_ftservo_hardware

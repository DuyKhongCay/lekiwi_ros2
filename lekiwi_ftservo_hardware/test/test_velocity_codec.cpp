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
  }

} // namespace lekiwi_ftservo_hardware

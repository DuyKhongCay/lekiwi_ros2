#include <array>

#include <gtest/gtest.h>

#include "lekiwi_driver/velocity_codec.hpp"

namespace lekiwi_driver
{

// Checks the wire format used by the STS goal-speed and present-speed registers.
TEST(VelocityCodec, EncodesAndDecodesSignedMagnitudeTicks)
{
  EXPECT_EQ(encode_velocity_ticks(0), (std::array<uint8_t, 2>{0U, 0U}));
  EXPECT_EQ(encode_velocity_ticks(-42), (std::array<uint8_t, 2>{42U, 128U}));
  EXPECT_EQ(decode_velocity_ticks(42U, 128U), -42);
  EXPECT_EQ(decode_velocity_ticks(42U, 0U), 42);
}

// Checks that the configured ROS velocity limit is applied before bus conversion.
TEST(VelocityCodec, BoundsAndAppliesWheelDirection)
{
  EXPECT_EQ(radians_per_second_to_ticks(3.0, 0.1, 2.0, 1), 20);
  EXPECT_EQ(radians_per_second_to_ticks(3.0, 0.1, 2.0, -1), -20);
}

}  // namespace lekiwi_driver

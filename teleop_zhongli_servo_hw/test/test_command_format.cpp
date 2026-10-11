/**
 * @file test_command_format.cpp
 * @brief Unit tests for Zhongli ASCII 1 Mbps protocol formatting and parsing.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include "teleop_zhongli_servo_hw/zhongli_protocol.hpp"

using namespace teleop_zhongli_servo_hw;

TEST(TestZhongliProtocolFormat, CommandFormatting)
{
  EXPECT_EQ(ZhongliProtocol::format_read_position_cmd(0), "#000PRAD!");
  EXPECT_EQ(ZhongliProtocol::format_read_position_cmd(5), "#005PRAD!");
  EXPECT_EQ(ZhongliProtocol::format_read_telemetry_cmd(1), "#001PRTV!");
  EXPECT_EQ(ZhongliProtocol::format_disable_torque_cmd(0), "#000PULK!");
  EXPECT_EQ(ZhongliProtocol::format_enable_torque_cmd(2), "#002PULR!");
  EXPECT_EQ(ZhongliProtocol::format_calibrate_midpoint_cmd(3), "#003PSCK!");
  EXPECT_EQ(ZhongliProtocol::format_set_initial_value_cmd(4), "#004PCSD!");
  EXPECT_EQ(ZhongliProtocol::format_control_servo_cmd(0, 1500, 1000), "#000P1500T1000!");
  EXPECT_EQ(ZhongliProtocol::format_control_servo_cmd(1, 2000, 0), "#001P2000!");
}

TEST(TestZhongliProtocolParse, ParsePositionResponseFullRange)
{
  int pwm = 0;

  // 4-digit PWM standard midpoints and limits
  EXPECT_TRUE(ZhongliProtocol::parse_position_response("#000P1500!", 0, &pwm));
  EXPECT_EQ(pwm, 1500);

  EXPECT_TRUE(ZhongliProtocol::parse_position_response("#005P2450!", 5, &pwm));
  EXPECT_EQ(pwm, 2450);

  // 3-digit PWM (< 1000) - previously dropped by buggy length check
  EXPECT_TRUE(ZhongliProtocol::parse_position_response("#000P500!", 0, &pwm));
  EXPECT_EQ(pwm, 500);

  EXPECT_TRUE(ZhongliProtocol::parse_position_response("#001P650!", 1, &pwm));
  EXPECT_EQ(pwm, 650);

  EXPECT_TRUE(ZhongliProtocol::parse_position_response("#002P999!", 2, &pwm));
  EXPECT_EQ(pwm, 999);

  // Padded 4-digit under 1000
  EXPECT_TRUE(ZhongliProtocol::parse_position_response("noise#002P0600!noise", 2, &pwm));
  EXPECT_EQ(pwm, 600);

  // Mismatched ID
  EXPECT_FALSE(ZhongliProtocol::parse_position_response("#001P1500!", 0, &pwm));

  // Malformed packets
  EXPECT_FALSE(ZhongliProtocol::parse_position_response("#000P!", 0, &pwm));
  EXPECT_FALSE(ZhongliProtocol::parse_position_response("#0001500!", 0, &pwm));
  EXPECT_FALSE(ZhongliProtocol::parse_position_response("None", 0, &pwm));
  EXPECT_FALSE(ZhongliProtocol::parse_position_response("#000P-10!", 0, &pwm));
}

TEST(TestZhongliProtocolParse, ParseTelemetryResponse)
{
  double temp = 0.0;
  double volt = 0.0;

  // Valid cases
  EXPECT_TRUE(ZhongliProtocol::parse_telemetry_response("#000T28.1V7.4!", 0, &temp, &volt));
  EXPECT_NEAR(temp, 28.1, 1e-4);
  EXPECT_NEAR(volt, 7.4, 1e-4);

  EXPECT_TRUE(ZhongliProtocol::parse_telemetry_response("#003T35.0V11.2!", 3, &temp, &volt));
  EXPECT_NEAR(temp, 35.0, 1e-4);
  EXPECT_NEAR(volt, 11.2, 1e-4);

  // Mismatched ID
  EXPECT_FALSE(ZhongliProtocol::parse_telemetry_response("#003T35.0V11.2!", 1, &temp, &volt));

  // Malformed cases
  EXPECT_FALSE(ZhongliProtocol::parse_telemetry_response("#000TV!", 0, &temp, &volt));
  EXPECT_FALSE(ZhongliProtocol::parse_telemetry_response("#000T28.1!", 0, &temp, &volt));
}

TEST(TestZhongliProtocolParse, ParseOkResponse)
{
  EXPECT_TRUE(ZhongliProtocol::parse_ok_response("#OK!"));
  EXPECT_TRUE(ZhongliProtocol::parse_ok_response("Prefix#OK!Suffix"));
  EXPECT_FALSE(ZhongliProtocol::parse_ok_response("#FAIL!"));
  EXPECT_FALSE(ZhongliProtocol::parse_ok_response(""));
}

TEST(TestZhongliProtocolParse, ParseVersionResponse)
{
  EXPECT_TRUE(ZhongliProtocol::parse_version_response("#000@ ZL-ZServo_AD_CBM V2.1.16STG!\r\n"));
  EXPECT_TRUE(ZhongliProtocol::parse_version_response("#000PV2.1!"));
  EXPECT_TRUE(ZhongliProtocol::parse_version_response("#000@ unknown!"));
  EXPECT_FALSE(ZhongliProtocol::parse_version_response("#000P1500!"));
  EXPECT_FALSE(ZhongliProtocol::parse_version_response("Random text"));
}

TEST(TestZhongliProtocolTiming, DefaultConfigTimingConstants)
{
  EXPECT_EQ(default_config::kTimeoutMs115200, 70);
  EXPECT_EQ(default_config::kTimeoutMs1M, 20);
  EXPECT_EQ(default_config::kEepromCommitDelayMs, 150);
  EXPECT_DOUBLE_EQ(default_config::kCalibPublishRateHz, 20.0);
  EXPECT_DOUBLE_EQ(default_config::kPublishRateHz, 50.0);
}



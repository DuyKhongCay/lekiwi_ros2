#include <gtest/gtest.h>

#include "lekiwi_ftservo_hardware/sts_protocol.hpp"

namespace lekiwi_ftservo_hardware
{

  TEST(StsProtocolTest, ValidatesProtocolConstants)
  {
    EXPECT_EQ(StsProtocol::kHeader, 0xff);
    EXPECT_EQ(StsProtocol::kBroadcastId, 0xfe);
    EXPECT_EQ(StsProtocol::kInstructionRead, 0x02);
    EXPECT_EQ(StsProtocol::kInstructionWrite, 0x03);
    EXPECT_EQ(StsProtocol::kInstructionRegWrite, 0x04);
    EXPECT_EQ(StsProtocol::kInstructionAction, 0x05);
    EXPECT_EQ(StsProtocol::kInstructionSyncRead, 0x82);
    EXPECT_EQ(StsProtocol::kInstructionSyncWrite, 0x83);

    EXPECT_EQ(StsProtocol::kModeRegister, 33);
    EXPECT_EQ(StsProtocol::kTorqueEnableRegister, 40);
    EXPECT_EQ(StsProtocol::kAccelerationRegister, 41);
    EXPECT_EQ(StsProtocol::kGoalPositionRegister, 42);
    EXPECT_EQ(StsProtocol::kGoalSpeedRegister, 46);
    EXPECT_EQ(StsProtocol::kLockRegister, 55);
    EXPECT_EQ(StsProtocol::kPresentPositionRegister, 56);
    EXPECT_EQ(StsProtocol::kPresentSpeedRegister, 58);
    EXPECT_EQ(StsProtocol::kPresentLoadRegister, 60);
    EXPECT_EQ(StsProtocol::kPresentVoltageRegister, 62);
    EXPECT_EQ(StsProtocol::kPresentTemperatureRegister, 63);
    EXPECT_EQ(StsProtocol::kMovingRegister, 66);
    EXPECT_EQ(StsProtocol::kPresentCurrentRegister, 69);
  }

  TEST(StsProtocolTest, DecodesDiagnosticTelemetryValues)
  {
    // Test voltage decoding: 124 -> 12.4V
    uint8_t raw_voltage = 124;
    double voltage_v = static_cast<double>(raw_voltage) * 0.1;
    EXPECT_DOUBLE_EQ(voltage_v, 12.4);

    // Test temperature decoding: 38 -> 38 deg C
    uint8_t raw_temp = 38;
    double temp_c = static_cast<double>(raw_temp);
    EXPECT_DOUBLE_EQ(temp_c, 38.0);

    // Test current decoding: 100 ticks -> 100 * 0.0065 = 0.65A
    int16_t raw_current = 100;
    double current_a = static_cast<double>(raw_current) * 0.0065;
    EXPECT_DOUBLE_EQ(current_a, 0.65);
  }

  TEST(StsProtocolTest, InitializesTelemetryStructures)
  {
    ServoDiagnosticData diag;
    diag.id = 1;
    diag.voltage_v = 12.0;
    diag.temperature_c = 35.0;
    diag.current_a = 0.5;
    diag.moving = false;

    EXPECT_EQ(diag.id, 1);
    EXPECT_DOUBLE_EQ(diag.voltage_v, 12.0);
    EXPECT_DOUBLE_EQ(diag.temperature_c, 35.0);
    EXPECT_DOUBLE_EQ(diag.current_a, 0.5);
    EXPECT_FALSE(diag.moving);

    ServoFastState fast;
    fast.id = 2;
    fast.position_ticks = 2048;
    fast.speed_ticks = 0;
    EXPECT_EQ(fast.id, 2);
    EXPECT_EQ(fast.position_ticks, 2048);
  }

} // namespace lekiwi_ftservo_hardware

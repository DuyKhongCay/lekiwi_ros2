/**
 * @file test_sts_protocol.cpp
 * @brief Unit tests (L1 verification) for Feetech STS serial protocol constants and telemetry conversion.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>

#include "lekiwi_ftservo_hardware/sts_protocol.hpp"

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Verifies that protocol register addresses and instruction OP codes match Feetech STS manual.
   */
  TEST(StsProtocolTest, ValidatesProtocolConstants)
  {
    EXPECT_EQ(sts::protocol::kHeader, 0xff);
    EXPECT_EQ(sts::protocol::kBroadcastId, 0xfe);
    EXPECT_EQ(sts::protocol::kInstructionRead, 0x02);
    EXPECT_EQ(sts::protocol::kInstructionWrite, 0x03);
    EXPECT_EQ(sts::protocol::kInstructionRegWrite, 0x04);
    EXPECT_EQ(sts::protocol::kInstructionAction, 0x05);
    EXPECT_EQ(sts::protocol::kInstructionSyncRead, 0x82);
    EXPECT_EQ(sts::protocol::kInstructionSyncWrite, 0x83);
    EXPECT_EQ(sts::protocol::kPreambleSearchLimit, 64U);
    EXPECT_EQ(sts::protocol::kDiagnosticPayloadSize, 15U);
    EXPECT_EQ(sts::protocol::kFastStatePayloadSize, 4U);

    EXPECT_EQ(sts::default_config::kPerServoReadTimeoutMs, 2);
    EXPECT_EQ(sts::default_config::kCommandWatchdogTimeoutMs, 100);
    EXPECT_EQ(sts::default_config::kMaxConsecutiveErrorsBeforeInvalid, 5U);
    EXPECT_EQ(sts::default_config::kMaxConsecutiveErrorsBeforeReconnect, 100U);

    EXPECT_DOUBLE_EQ(sts::default_config::kDefaultVelocityScale, 0.0015339807878856412);
    EXPECT_DOUBLE_EQ(sts::default_config::kDefaultMaxVelocity, 5.0);
    EXPECT_EQ(sts::default_config::kDefaultGoalSpeedTicks, 2400);
    EXPECT_EQ(sts::default_config::kDefaultAcceleration, 0U);
    EXPECT_DOUBLE_EQ(sts::default_config::kLowVoltageLimitV, 9.0);
    EXPECT_DOUBLE_EQ(sts::default_config::kHighVoltageLimitV, 13.5);

    EXPECT_EQ(sts::register_addr::kMode, 33);
    EXPECT_EQ(sts::register_addr::kTorqueEnable, 40);
    EXPECT_EQ(sts::register_addr::kAcceleration, 41);
    EXPECT_EQ(sts::register_addr::kGoalPosition, 42);
    EXPECT_EQ(sts::register_addr::kGoalSpeed, 46);
    EXPECT_EQ(sts::register_addr::kLock, 55);
    EXPECT_EQ(sts::register_addr::kPresentPosition, 56);
    EXPECT_EQ(sts::register_addr::kPresentSpeed, 58);
    EXPECT_EQ(sts::register_addr::kPresentLoad, 60);
    EXPECT_EQ(sts::register_addr::kPresentVoltage, 62);
    EXPECT_EQ(sts::register_addr::kPresentTemperature, 63);
    EXPECT_EQ(sts::register_addr::kMoving, 66);
    EXPECT_EQ(sts::register_addr::kPresentCurrent, 69);
  }

  /**
   * @brief Verifies raw ADC conversion factors for STS telemetry (voltage 0.1V/LSB, current 6.5mA/LSB).
   */
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

  /**
   * @brief Verifies default values and field assignment in telemetry and fast state structures.
   */
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

  /**
   * @brief Verifies argument validation in sync_write_torque for empty or mismatched vectors.
   */
  TEST(StsProtocolTest, RejectsInvalidSyncWriteTorqueArguments)
  {
    StsProtocol proto;
    std::string error;

    // 1. Empty vectors should fail
    EXPECT_FALSE(proto.sync_write_torque({}, {}, &error));
    EXPECT_FALSE(error.empty());

    // 2. Mismatched vectors should fail
    EXPECT_FALSE(proto.sync_write_torque({1, 2}, {true}, &error));
    EXPECT_FALSE(error.empty());
  }

  /**
   * @brief Verifies StsProtocol::decode_telemetry_payload parsing of Feetech STS 15-byte status payload.
   */
  TEST(StsProtocolTest, DecodesTelemetryPayloadCorrectly)
  {
    ServoDiagnosticData diag;

    // Reject payload smaller than 15 bytes
    std::vector<uint8_t> short_payload(14, 0);
    EXPECT_FALSE(StsProtocol::decode_telemetry_payload(1, short_payload, diag));

    // Construct valid 15-byte payload:
    // [0..1] pos, [2..3] speed, [4..5] load, [6] voltage (124 -> 12.4V),
    // [7] temp (38 -> 38 degC), [8..9] dummy, [10] moving (1), [11..12] dummy,
    // [13..14] current (100 -> 0.65A = 100 * 0.0065)
    std::vector<uint8_t> valid_payload = {
        0x00, 0x08, // pos: 2048
        0x64, 0x00, // speed: 100
        0x32, 0x00, // load: 50
        124,        // voltage: 12.4 V
        38,         // temperature: 38 deg C
        0x00, 0x00, // dummy
        1,          // moving: true
        0x00, 0x00, // dummy
        0x64, 0x00  // current: 100 ticks -> 0.65 A
    };

    EXPECT_TRUE(StsProtocol::decode_telemetry_payload(5, valid_payload, diag));
    EXPECT_EQ(diag.id, 5);
    EXPECT_DOUBLE_EQ(diag.voltage_v, 12.4);
    EXPECT_DOUBLE_EQ(diag.temperature_c, 38.0);
    EXPECT_TRUE(diag.moving);
    EXPECT_DOUBLE_EQ(diag.current_a, 0.65);
  }

  /**
   * @brief Verifies zero-allocation checksum computation with raw pointer and std::span overloads.
   */
  TEST(StsProtocolTest, ComputesChecksumCorrectly)
  {
    // Checksum formula: ~(sum of bytes) & 0xFF
    // Test case: Ping packet body [ID=1, Len=2, Inst=1] -> sum = 4 -> ~4 = 0xFB
    const uint8_t packet_body[] = {0x01, 0x02, 0x01};
    uint8_t expected_checksum = 0xFB;

    EXPECT_EQ(StsProtocol::compute_checksum(packet_body, sizeof(packet_body)), expected_checksum);

    // Test with std::span
    std::span<const uint8_t> span_view(packet_body);
    EXPECT_EQ(StsProtocol::compute_checksum(span_view), expected_checksum);

    // Test multi-byte payload
    const std::vector<uint8_t> vector_data = {0x02, 0x05, 0x03, 0x2A, 0x00, 0x08};
    // Sum = 2 + 5 + 3 + 42 + 0 + 8 = 60 (0x3C)
    // ~0x3C = 0xC3
    EXPECT_EQ(StsProtocol::compute_checksum(vector_data.data(), vector_data.size()), 0xC3);
    EXPECT_EQ(StsProtocol::compute_checksum(std::span<const uint8_t>(vector_data)), 0xC3);
  }

  /**
   * @brief Verifies argument validation in sync_write_position for empty or mismatched vectors.
   */
  TEST(StsProtocolTest, RejectsInvalidSyncWritePositionArguments)
  {
    StsProtocol proto;
    std::string error;

    // 1. Empty vectors should fail
    EXPECT_FALSE(proto.sync_write_position({}, {}, &error));
    EXPECT_FALSE(error.empty());

    // 2. Mismatched vectors should fail
    EXPECT_FALSE(proto.sync_write_position({1, 2}, {2048}, &error));
    EXPECT_FALSE(error.empty());
  }

  /**
   * @brief Verifies argument validation in sync_write_velocity for empty or mismatched vectors.
   */
  TEST(StsProtocolTest, RejectsInvalidSyncWriteVelocityArguments)
  {
    StsProtocol proto;
    std::string error;

    // 1. Empty vectors should fail
    EXPECT_FALSE(proto.sync_write_velocity({}, {}, &error));
    EXPECT_FALSE(error.empty());

    // 2. Mismatched vectors should fail
    EXPECT_FALSE(proto.sync_write_velocity({1, 2}, {100}, &error));
    EXPECT_FALSE(error.empty());
  }

} // namespace lekiwi_ftservo_hardware

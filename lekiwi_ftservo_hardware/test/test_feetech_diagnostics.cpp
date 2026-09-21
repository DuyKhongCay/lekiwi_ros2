/**
 * @file test_feetech_diagnostics.cpp
 * @brief Unit tests for FeetechDiagnostics health evaluation logic (L1 verification).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "lekiwi_ftservo_hardware/feetech_diagnostics.hpp"

namespace lekiwi_ftservo_hardware
{

  TEST(FeetechDiagnosticsTest, HandlesUnconfiguredOrNullSnapshot)
  {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    FeetechDiagnostics::evaluate(nullptr, "/dev/ttyUSB0", 1000000, 9, false, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_EQ(stat.message, "STS Serial bus offline or unconfigured");
  }

  TEST(FeetechDiagnosticsTest, ReportsHealthySnapshot)
  {
    JointStateSnapshot snapshot;
    snapshot.valid = true;
    snapshot.last_read_time = std::chrono::steady_clock::now();
    snapshot.update_count = 100;
    snapshot.read_error_count = 0;

    JointTelemetry joint1;
    joint1.name = "joint1";
    joint1.id = 1;
    joint1.voltage_v = 12.0;
    joint1.temperature_c = 35.0;
    joint1.current_a = 0.5;
    joint1.status_flags = 0;

    snapshot.telemetry = {joint1};

    diagnostic_updater::DiagnosticStatusWrapper stat;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat);

    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(stat.message, "All 1 STS servos healthy");
  }

  TEST(FeetechDiagnosticsTest, WarnsOnAbnormalVoltage)
  {
    JointStateSnapshot snapshot;
    snapshot.valid = true;
    snapshot.last_read_time = std::chrono::steady_clock::now();

    JointTelemetry joint;
    joint.name = "joint1";
    joint.id = 1;
    joint.temperature_c = 30.0;
    joint.status_flags = 0;

    // 1. Low voltage (< 9.0V)
    joint.voltage_v = 8.5;
    snapshot.telemetry = {joint};
    diagnostic_updater::DiagnosticStatusWrapper stat_low;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat_low);
    EXPECT_EQ(stat_low.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_NE(stat_low.message.find("Low supply voltage"), std::string::npos);

    // 2. High voltage (> 13.5V)
    joint.voltage_v = 14.2;
    snapshot.telemetry = {joint};
    diagnostic_updater::DiagnosticStatusWrapper stat_high;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat_high);
    EXPECT_EQ(stat_high.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_NE(stat_high.message.find("High supply voltage"), std::string::npos);
  }

  TEST(FeetechDiagnosticsTest, HandlesTemperatureWarningsAndErrors)
  {
    JointStateSnapshot snapshot;
    snapshot.valid = true;
    snapshot.last_read_time = std::chrono::steady_clock::now();

    JointTelemetry joint;
    joint.name = "joint1";
    joint.id = 1;
    joint.voltage_v = 12.0;
    joint.status_flags = 0;

    // 1. Warm temperature (62C >= 60C limit) -> WARN
    joint.temperature_c = 62.0;
    snapshot.telemetry = {joint};
    diagnostic_updater::DiagnosticStatusWrapper stat_warm;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat_warm);
    EXPECT_EQ(stat_warm.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

    // 2. Overheat (72C >= 70C limit) -> ERROR
    joint.temperature_c = 72.0;
    snapshot.telemetry = {joint};
    diagnostic_updater::DiagnosticStatusWrapper stat_hot;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat_hot);
    EXPECT_EQ(stat_hot.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_NE(stat_hot.message.find("Overheat"), std::string::npos);
  }

  TEST(FeetechDiagnosticsTest, FlagsHardwareErrorBitsAndStaleData)
  {
    JointStateSnapshot snapshot;
    snapshot.valid = true;
    snapshot.last_read_time = std::chrono::steady_clock::now();

    JointTelemetry joint;
    joint.name = "joint1";
    joint.id = 1;
    joint.voltage_v = 12.0;
    joint.temperature_c = 30.0;
    joint.status_flags = 0x04; // Hardware error flag

    snapshot.telemetry = {joint};
    diagnostic_updater::DiagnosticStatusWrapper stat_err;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat_err);
    EXPECT_EQ(stat_err.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_NE(stat_err.message.find("Hardware Flag"), std::string::npos);

    // Test stale data (> 200 ms)
    joint.status_flags = 0;
    snapshot.telemetry = {joint};
    snapshot.last_read_time = std::chrono::steady_clock::now() - std::chrono::milliseconds(300);
    diagnostic_updater::DiagnosticStatusWrapper stat_stale;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat_stale);
    EXPECT_EQ(stat_stale.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_NE(stat_stale.message.find("stale"), std::string::npos);
  }

  TEST(FeetechDiagnosticsTest, CombinesVoltageAndTemperatureWarnings)
  {
    JointStateSnapshot snapshot;
    snapshot.valid = true;
    snapshot.last_read_time = std::chrono::steady_clock::now();

    JointTelemetry joint;
    joint.name = "joint1";
    joint.id = 1;
    joint.voltage_v = 8.5;      // Low voltage (< 9.0V)
    joint.temperature_c = 62.0; // Warm (>= 60C)
    joint.status_flags = 0;

    snapshot.telemetry = {joint};
    diagnostic_updater::DiagnosticStatusWrapper stat;
    FeetechDiagnostics::evaluate(&snapshot, "/dev/ttyUSB0", 1000000, 1, true, stat);
    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_NE(stat.message.find("Low supply voltage"), std::string::npos);
    EXPECT_NE(stat.message.find("high temperature"), std::string::npos);
  }

} // namespace lekiwi_ftservo_hardware

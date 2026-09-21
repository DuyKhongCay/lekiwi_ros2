/**
 * @file feetech_diagnostics.cpp
 * @brief Implementation of FeetechDiagnostics health evaluation.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_ftservo_hardware/feetech_diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

namespace lekiwi_ftservo_hardware
{

  void FeetechDiagnostics::evaluate(
      const JointStateSnapshot *snapshot,
      const std::string &usb_port,
      const int baud_rate,
      const size_t configured_joints_count,
      const bool is_worker_running,
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (snapshot == nullptr || !snapshot->valid || snapshot->telemetry.empty())
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "STS Serial bus offline or unconfigured");
      stat.add("USB Port", usb_port);
      stat.add("Baud Rate", baud_rate);
      stat.add("Worker Running", is_worker_running ? "True" : "False");
      return;
    }

    const auto &telem_list = snapshot->telemetry;
    const uint64_t total_updates = snapshot->update_count;
    const uint64_t total_errors = snapshot->read_error_count;
    const auto last_read = snapshot->last_read_time;

    // Check freshness of last read
    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count();
    const bool is_stale = (age_ms > sts::default_config::kTelemetryStaleTimeoutMs);

    // Telemetry aggregations
    double min_v = 999.0;
    double max_v = 0.0;
    double max_temp = 0.0;
    std::string hottest_joint;
    double max_curr = 0.0;
    std::string high_curr_joint;
    std::vector<std::string> error_joints;
    std::vector<std::string> warn_joints;

    for (const auto &telem : telem_list)
    {
      if (telem.voltage_v > 0.1)
      {
        min_v = std::min(min_v, telem.voltage_v);
        max_v = std::max(max_v, telem.voltage_v);
      }
      if (telem.temperature_c > max_temp)
      {
        max_temp = telem.temperature_c;
        hottest_joint = std::string(telem.name);
      }
      if (telem.current_a > max_curr)
      {
        max_curr = telem.current_a;
        high_curr_joint = std::string(telem.name);
      }

      // Check temperature limits
      if (telem.temperature_c >= sts::default_config::kServoTempErrorLimitC)
      {
        error_joints.push_back(std::string(telem.name) + " (Overheat: " + std::to_string(static_cast<int>(telem.temperature_c)) + "C)");
      }
      else if (telem.temperature_c >= sts::default_config::kServoTempWarnLimitC)
      {
        warn_joints.push_back(std::string(telem.name) + " (Warm: " + std::to_string(static_cast<int>(telem.temperature_c)) + "C)");
      }

      // Check hardware protection flags
      if (telem.status_flags != 0)
      {
        error_joints.push_back(std::string(telem.name) + " (Hardware Flag: 0x" + std::to_string(telem.status_flags) + ")");
      }
    }

    // Check voltage safety (Nominal STS3215 operating range)
    const bool low_voltage = (min_v < sts::default_config::kLowVoltageLimitV && min_v > 1.0);
    const bool high_voltage = (max_v > sts::default_config::kHighVoltageLimitV);

    // 1. Overall Status Evaluation
    if (is_stale)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "STS telemetry stale (last read %lld ms ago)", static_cast<long long>(age_ms));
    }
    else if (!error_joints.empty())
    {
      std::string err_str;
      for (size_t i = 0; i < error_joints.size(); ++i)
      {
        err_str += (i > 0 ? ", " : "") + error_joints[i];
      }
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Servo errors: %s", err_str.c_str());
    }
    else if (low_voltage || high_voltage || !warn_joints.empty())
    {
      std::string warn_msg;
      if (low_voltage)
      {
        char v_buf[64];
        std::snprintf(v_buf, sizeof(v_buf), "Low supply voltage: %.1f V", min_v);
        warn_msg = v_buf;
      }
      else if (high_voltage)
      {
        char v_buf[64];
        std::snprintf(v_buf, sizeof(v_buf), "High supply voltage: %.1f V", max_v);
        warn_msg = v_buf;
      }

      if (!warn_joints.empty())
      {
        if (!warn_msg.empty())
        {
          warn_msg += "; ";
        }
        warn_msg += "Servos operating at high temperature";
      }
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);
    }
    else
    {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "All %zu STS servos healthy", telem_list.size());
    }

    // 2. Add Key-Value Metrics
    stat.add("Serial Port", usb_port);
    stat.add("Baud Rate", baud_rate);
    stat.add("Active Servos Count", std::to_string(telem_list.size()) + " / " + std::to_string(configured_joints_count));
    stat.add("Worker I/O Loops", total_updates);
    stat.add("Worker Read Errors", total_errors);

    if (total_updates + total_errors > 0)
    {
      const double err_rate = (static_cast<double>(total_errors) / (total_updates + total_errors)) * 100.0;
      stat.addf("Serial Error Rate (%)", "%.2f", err_rate);
    }

    if (min_v < 900.0)
    {
      stat.addf("Bus Voltage (Min / Max)", "%.2f V / %.2f V", min_v, max_v);
    }
    stat.addf("Max Temperature", "%.1f C (%s)", max_temp, hottest_joint.c_str());
    stat.addf("Max Current", "%.2f A (%s)", max_curr, high_curr_joint.c_str());
  }

} // namespace lekiwi_ftservo_hardware

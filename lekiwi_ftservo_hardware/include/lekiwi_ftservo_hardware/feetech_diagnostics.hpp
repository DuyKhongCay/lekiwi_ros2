/**
 * @file feetech_diagnostics.hpp
 * @brief Health evaluation and diagnostic reporting for Feetech STS servo bus.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cstddef>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "lekiwi_ftservo_hardware/joint_state_snapshot.hpp"
#include "lekiwi_ftservo_hardware/sts_constants.hpp"

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Evaluates servo hardware telemetry and produces structured ROS 2 diagnostic status.
   */
  class FeetechDiagnostics
  {
  public:
    /**
     * @brief Evaluates telemetry snapshot and formats diagnostic key-values and status summary.
     *
     * @param[in] snapshot Pointer to latest telemetry snapshot (can be null or invalid).
     * @param[in] usb_port Configured serial device path.
     * @param[in] baud_rate Configured serial baud rate.
     * @param[in] configured_joints_count Number of joints configured from URDF.
     * @param[in] is_worker_running True if the background I/O thread is actively spinning.
     * @param[out] stat Diagnostic wrapper populated with status and telemetry metrics.
     */
    static void evaluate(
        const JointStateSnapshot *snapshot,
        const std::string &usb_port,
        int baud_rate,
        size_t configured_joints_count,
        bool is_worker_running,
        diagnostic_updater::DiagnosticStatusWrapper &stat);
  };

} // namespace lekiwi_ftservo_hardware

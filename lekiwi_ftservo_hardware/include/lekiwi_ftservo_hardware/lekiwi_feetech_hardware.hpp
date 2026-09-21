/**
 * @file lekiwi_feetech_hardware.hpp
 * @brief ros2_control SystemInterface hardware plugin adapter for Feetech STS servos.
 *
 * Thin Façade pattern delegating URDF parsing to JointConfigParser, background serial
 * communication and real-time buffering to FeetechBusWorker, and health telemetry
 * to FeetechDiagnostics.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lekiwi_ftservo_hardware/feetech_bus_worker.hpp"
#include "lekiwi_ftservo_hardware/feetech_diagnostics.hpp"
#include "lekiwi_ftservo_hardware/joint_config_parser.hpp"
#include "lekiwi_ftservo_hardware/joint_state_snapshot.hpp"
#include "lekiwi_ftservo_hardware/sts_constants.hpp"

namespace lekiwi_ftservo_hardware
{

    /**
     * @brief Hardware plugin connecting Feetech STS serial servos to ros2_control.
     *
     * Conforms to the `hardware_interface::SystemInterface` specification.
     * Maintains real-time latency strictly bounded (< 10 µs) with zero dynamic allocation
     * and delegates all blocking I/O to FeetechBusWorker.
     */
    class LeKiwiFeetechHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        ~LeKiwiFeetechHardwareInterface() override;

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief Provides a copy of the latest joint telemetry snapshot.
         */
        std::vector<JointTelemetry> get_telemetry() const;

    private:
        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

        JointTopology topology_;
        std::unique_ptr<FeetechBusWorker> worker_;
        std::string usb_port_;
        int baud_rate_{sts::default_config::kDefaultBaudRate};
        int timeout_ms_{sts::default_config::kDefaultTimeoutMs};

        std::shared_ptr<diagnostic_updater::Updater> updater_;
    };

} // namespace lekiwi_ftservo_hardware

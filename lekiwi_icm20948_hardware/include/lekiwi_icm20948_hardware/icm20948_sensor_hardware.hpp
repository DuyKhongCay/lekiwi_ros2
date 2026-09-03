/**
 * @file icm20948_sensor_hardware.hpp
 * @brief ros2_control SensorInterface hardware plugin for the ICM-20948 9-DoF IMU.
 *
 * Exposes 10 state interfaces:
 * - `orientation.x`, `orientation.y`, `orientation.z`, `orientation.w` (Quaternion, default identity)
 * - `angular_velocity.x`, `angular_velocity.y`, `angular_velocity.z` ($rad/s$, REP-103)
 * - `linear_acceleration.x`, `linear_acceleration.y`, `linear_acceleration.z` ($m/s^2$, REP-103)
 *
 * Implements axis sign remapping, static bias subtraction, online gyro auto-calibration,
 * and diagnostic telemetry updating.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "lekiwi_icm20948_hardware/icm20948_driver.hpp"

namespace lekiwi_icm20948_hardware
{

    /**
     * @brief Sensor hardware interface plugin for ICM-20948 9-Axis IMU.
     *
     * Adheres to the `hardware_interface::SensorInterface` contract for ros2_control.
     */
    class ICM20948SensorHardware : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948SensorHardware)

        /**
         * @brief Initializes hardware parameters from the URDF `<ros2_control>` XML description.
         *
         * Parses I2C bus number, address, calibration offsets, axis sign directions,
         * and registers the 10 IMU state interfaces.
         *
         * @param[in] params Hardware component parameters passed by the resource manager.
         * @return hardware_interface::CallbackReturn SUCCESS if parameters and interface contracts match, ERROR otherwise.
         */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        /**
         * @brief Configures I2C communication and programs sensor registers.
         *
         * Opens `/dev/i2c-X`, verifies WHO_AM_I, and initializes DLPF, sample rate, and AK09916 auxiliary bus.
         *
         * @param[in] previous_state Lifecycle state prior to transition (Unconfigured).
         * @return hardware_interface::CallbackReturn SUCCESS on device ready, ERROR on communication failure.
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Activates the IMU sensor and starts the zero-motion gyro calibration sequence.
         *
         * @param[in] previous_state Lifecycle state prior to transition (Inactive).
         * @return hardware_interface::CallbackReturn SUCCESS on successful activation.
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Deactivates the IMU and puts the device into low-power sleep mode.
         *
         * @param[in] previous_state Lifecycle state prior to transition (Active).
         * @return hardware_interface::CallbackReturn SUCCESS on deactivation.
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Reads fresh IMU burst sample and updates ros2_control state interface buffers.
         *
         * @note **Real-Time Safety:** In hardware mode, this performs an I2C burst read over `/dev/i2c-X`
         * which consumes ~1.0 ms at 400 kHz Fast Mode. Ensure the controller manager update rate accounts for this timing.
         *
         * @param[in] time Current ROS system time.
         * @param[in] period Control loop cycle duration since last call.
         * @return hardware_interface::return_type OK on success, ERROR if consecutive bus failures exceed threshold.
         */
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        ICM20948Driver driver_;
        DriverConfig driver_config_;
        std::string sensor_name_{"icm20948_imu"};
        bool mock_sensor_{false};

        // Sensor Calibration & Remapping
        std::array<double, 3> accel_bias_{0.0, 0.0, 0.0};
        std::array<double, 3> gyro_bias_{0.0, 0.0, 0.0};

        std::array<int, 3> accel_axis_sign_{1, 1, 1};
        std::array<int, 3> gyro_axis_sign_{1, 1, 1};
        std::array<int, 3> mag_axis_sign_{1, 1, 1};

        // Static Gyro Bias Auto-Calibration
        bool auto_calibrate_gyro_{true};
        int gyro_calib_samples_{500};
        int current_calib_count_{0};
        std::array<double, 3> gyro_bias_sum_{0.0, 0.0, 0.0};
        bool gyro_calibrated_{false};

        // Error & Reliability tracking
        int consecutive_errors_{0};
        int max_consecutive_errors_{10};
        uint64_t total_reads_{0};
        uint64_t failed_reads_{0};
        SensorData last_valid_data_;

        // Diagnostics
        std::shared_ptr<diagnostic_updater::Updater> updater_;
        /**
         * @brief Publishes IMU hardware diagnostics and communication metrics.
         *
         * @param[out] stat Status wrapper populated with diagnostic metrics.
         */
        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    };

} // namespace lekiwi_icm20948_hardware

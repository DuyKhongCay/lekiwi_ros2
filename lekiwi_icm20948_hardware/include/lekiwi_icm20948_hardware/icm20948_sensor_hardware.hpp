#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "lekiwi_icm20948_hardware/icm20948_driver.hpp"

namespace lekiwi_icm20948_hardware
{

    class ICM20948SensorHardware : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948SensorHardware)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

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
        std::array<double, 3> mag_bias_{0.0, 0.0, 0.0};
        std::array<double, 3> mag_scale_{1.0, 1.0, 1.0};

        std::array<int, 3> accel_axis_sign_{1, 1, 1};
        std::array<int, 3> gyro_axis_sign_{1, 1, 1};
        std::array<int, 3> mag_axis_sign_{1, 1, 1};

        // Static Gyro Bias Auto-Calibration
        bool auto_calibrate_gyro_{true};
        int gyro_calib_samples_{500};
        int current_calib_count_{0};
        std::array<double, 3> gyro_bias_sum_{0.0, 0.0, 0.0};
        bool gyro_calibrated_{false};

        // Error tracking
        int consecutive_errors_{0};
        int max_consecutive_errors_{10};
        SensorData last_valid_data_;
    };

} // namespace lekiwi_icm20948_hardware

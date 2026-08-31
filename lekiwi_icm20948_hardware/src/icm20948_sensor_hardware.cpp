#include "lekiwi_icm20948_hardware/icm20948_sensor_hardware.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lekiwi_icm20948_hardware
{

    hardware_interface::CallbackReturn ICM20948SensorHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SensorInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.sensors.empty())
        {
            RCLCPP_FATAL(get_logger(), "No sensor tag defined in URDF for ICM20948");
            return hardware_interface::CallbackReturn::ERROR;
        }

        sensor_name_ = info_.sensors[0].name;

        // Parse mock_sensor
        auto it_mock = info_.hardware_parameters.find("mock_sensor");
        if (it_mock != info_.hardware_parameters.end())
        {
            mock_sensor_ = (it_mock->second == "true" || it_mock->second == "True" || it_mock->second == "1");
        }

        // Parse I2C bus
        auto it_bus = info_.hardware_parameters.find("i2c_bus");
        if (it_bus != info_.hardware_parameters.end())
        {
            try
            {
                driver_config_.i2c_bus = std::stoi(it_bus->second);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid i2c_bus param '%s': %s", it_bus->second.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Parse I2C address (supports hex "0x68" or decimal)
        auto it_addr = info_.hardware_parameters.find("i2c_address");
        if (it_addr != info_.hardware_parameters.end())
        {
            try
            {
                driver_config_.i2c_address = static_cast<uint8_t>(std::stoul(it_addr->second, nullptr, 0));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid i2c_address param '%s': %s", it_addr->second.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Parse Accel Range
        auto it_arange = info_.hardware_parameters.find("accel_range");
        if (it_arange != info_.hardware_parameters.end())
        {
            if (it_arange->second == "2G" || it_arange->second == "2g")
            {
                driver_config_.accel_range = AccelRange::RANGE_2G;
            }
            else if (it_arange->second == "4G" || it_arange->second == "4g")
            {
                driver_config_.accel_range = AccelRange::RANGE_4G;
            }
            else if (it_arange->second == "8G" || it_arange->second == "8g")
            {
                driver_config_.accel_range = AccelRange::RANGE_8G;
            }
            else if (it_arange->second == "16G" || it_arange->second == "16g")
            {
                driver_config_.accel_range = AccelRange::RANGE_16G;
            }
        }

        // Parse Gyro Range
        auto it_grange = info_.hardware_parameters.find("gyro_range");
        if (it_grange != info_.hardware_parameters.end())
        {
            if (it_grange->second == "250DPS" || it_grange->second == "250dps")
            {
                driver_config_.gyro_range = GyroRange::RANGE_250DPS;
            }
            else if (it_grange->second == "500DPS" || it_grange->second == "500dps")
            {
                driver_config_.gyro_range = GyroRange::RANGE_500DPS;
            }
            else if (it_grange->second == "1000DPS" || it_grange->second == "1000dps")
            {
                driver_config_.gyro_range = GyroRange::RANGE_1000DPS;
            }
            else if (it_grange->second == "2000DPS" || it_grange->second == "2000dps")
            {
                driver_config_.gyro_range = GyroRange::RANGE_2000DPS;
            }
        }

        // Parse DLPF
        auto it_dlpf = info_.hardware_parameters.find("dlpf_config");
        if (it_dlpf != info_.hardware_parameters.end())
        {
            try
            {
                driver_config_.dlpf_config = static_cast<uint8_t>(std::stoi(it_dlpf->second));
            }
            catch (...)
            {
            }
        }

        // Doc cau hinh tu dong can chinh Gyro Bias (Auto-calibration on startup)
        auto it_calib = info_.hardware_parameters.find("auto_calibrate_gyro");
        if (it_calib != info_.hardware_parameters.end())
        {
            auto_calibrate_gyro_ = (it_calib->second == "true" || it_calib->second == "True" || it_calib->second == "1");
        }

        auto it_samples = info_.hardware_parameters.find("gyro_calib_samples");
        if (it_samples != info_.hardware_parameters.end())
        {
            try
            {
                gyro_calib_samples_ = std::max(10, std::stoi(it_samples->second));
            }
            catch (...)
            {
                RCLCPP_WARN(get_logger(), "Invalid gyro_calib_samples '%s', using default %d",
                            it_samples->second.c_str(), gyro_calib_samples_);
            }
        }

        // Doc gia tri gyro bias co dinh (neu khong dung auto calibration)
        auto it_gbx = info_.hardware_parameters.find("gyro_bias_x");
        if (it_gbx != info_.hardware_parameters.end())
        {
            try
            {
                gyro_bias_[0] = std::stod(it_gbx->second);
            }
            catch (...)
            {
            }
        }
        auto it_gby = info_.hardware_parameters.find("gyro_bias_y");
        if (it_gby != info_.hardware_parameters.end())
        {
            try
            {
                gyro_bias_[1] = std::stod(it_gby->second);
            }
            catch (...)
            {
            }
        }
        auto it_gbz = info_.hardware_parameters.find("gyro_bias_z");
        if (it_gbz != info_.hardware_parameters.end())
        {
            try
            {
                gyro_bias_[2] = std::stod(it_gbz->second);
            }
            catch (...)
            {
            }
        }

        // Verify exported state interfaces
        const std::vector<std::string> expected_interfaces = {
            "orientation.x", "orientation.y", "orientation.z", "orientation.w",
            "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
            "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
            "magnetic_field.x", "magnetic_field.y", "magnetic_field.z"};

        for (const auto &if_name : expected_interfaces)
        {
            bool found = false;
            for (const auto &state_if : info_.sensors[0].state_interfaces)
            {
                if (state_if.name == if_name)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                RCLCPP_WARN(get_logger(), "Expected state interface '%s' not explicitly declared in URDF sensor tag", if_name.c_str());
            }
        }

        RCLCPP_INFO(get_logger(), "Initialized ICM20948SensorHardware for sensor '%s' on /dev/i2c-%d (addr 0x%02X)",
                    sensor_name_.c_str(), driver_config_.i2c_bus, driver_config_.i2c_address);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ICM20948SensorHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (mock_sensor_)
        {
            RCLCPP_INFO(get_logger(), "ICM20948 running in MOCK mode (simulating stationary sensor values).");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        RCLCPP_INFO(get_logger(), "Configuring ICM20948 sensor on /dev/i2c-%d...", driver_config_.i2c_bus);

        std::string err;
        if (!driver_.open_bus(driver_config_, &err))
        {
            RCLCPP_ERROR(get_logger(), "Failed to open I2C bus: %s", err.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!driver_.configure_device(&err))
        {
            RCLCPP_ERROR(get_logger(), "Failed to configure ICM20948 sensor: %s", err.c_str());
            driver_.close_bus();
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Successfully configured ICM20948 and AK09916 magnetometer.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ICM20948SensorHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating ICM20948SensorHardware...");
        consecutive_errors_ = 0;

        // Reset trang thai auto-calibration cho Gyro
        gyro_bias_sum_ = {0.0, 0.0, 0.0};
        current_calib_count_ = 0;
        gyro_calibrated_ = !auto_calibrate_gyro_;
        if (auto_calibrate_gyro_ && !mock_sensor_)
        {
            RCLCPP_INFO(get_logger(), "Starting Gyro auto-calibration (%d samples)... Keep robot stationary!", gyro_calib_samples_);
        }

        // Dat gia tri khoi tao: quaternion dong nhat (0,0,0,1), gia toc trong truong +Z = 9.81 m/s^2
        set_state(sensor_name_ + "/orientation.x", 0.0);
        set_state(sensor_name_ + "/orientation.y", 0.0);
        set_state(sensor_name_ + "/orientation.z", 0.0);
        set_state(sensor_name_ + "/orientation.w", 1.0);

        set_state(sensor_name_ + "/angular_velocity.x", 0.0);
        set_state(sensor_name_ + "/angular_velocity.y", 0.0);
        set_state(sensor_name_ + "/angular_velocity.z", 0.0);

        set_state(sensor_name_ + "/linear_acceleration.x", 0.0);
        set_state(sensor_name_ + "/linear_acceleration.y", 0.0);
        set_state(sensor_name_ + "/linear_acceleration.z", GRAVITY_EARTH);

        set_state(sensor_name_ + "/magnetic_field.x", 2.0e-5);
        set_state(sensor_name_ + "/magnetic_field.y", 0.0);
        set_state(sensor_name_ + "/magnetic_field.z", 4.0e-5);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Xu ly khi deactivate sensor
    hardware_interface::CallbackReturn ICM20948SensorHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating ICM20948SensorHardware...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Chu ky RT read(): doc cam bien, ap dung calib va cap nhat 13 state interfaces
    hardware_interface::return_type ICM20948SensorHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Truong hop mock mode: cap nhat du lieu tinh gia lap
        if (mock_sensor_)
        {
            set_state(sensor_name_ + "/orientation.x", 0.0);
            set_state(sensor_name_ + "/orientation.y", 0.0);
            set_state(sensor_name_ + "/orientation.z", 0.0);
            set_state(sensor_name_ + "/orientation.w", 1.0);

            set_state(sensor_name_ + "/angular_velocity.x", 0.0);
            set_state(sensor_name_ + "/angular_velocity.y", 0.0);
            set_state(sensor_name_ + "/angular_velocity.z", 0.0);

            set_state(sensor_name_ + "/linear_acceleration.x", 0.0);
            set_state(sensor_name_ + "/linear_acceleration.y", 0.0);
            set_state(sensor_name_ + "/linear_acceleration.z", GRAVITY_EARTH);

            set_state(sensor_name_ + "/magnetic_field.x", 2.0e-5);
            set_state(sensor_name_ + "/magnetic_field.y", 0.0);
            set_state(sensor_name_ + "/magnetic_field.z", 4.0e-5);

            return hardware_interface::return_type::OK;
        }

        SensorData data;
        std::string err;

        // Doc du lieu tu phan cung I2C, xu ly loi thoang qua (transient noise)
        if (!driver_.read_sensor_data(data, &err))
        {
            consecutive_errors_++;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "ICM20948 I2C read failed: %s (consecutive errors: %d)",
                err.c_str(), consecutive_errors_);

            // Tra ve ERROR neu loi lien tiep vuot nguong toi da
            if (consecutive_errors_ > max_consecutive_errors_)
            {
                RCLCPP_ERROR(get_logger(), "ICM20948 exceeded max consecutive errors (%d), reporting failure",
                             max_consecutive_errors_);
                return hardware_interface::return_type::ERROR;
            }
            // Giu nguyen gia tri hop le truoc do khi bi loi nhe
            return hardware_interface::return_type::OK;
        }

        consecutive_errors_ = 0;
        last_valid_data_ = data;

        // Ap dung bu tru bias, he so ti le va dinh huong truc theo chuan REP-103
        const double ax = (data.accel_m_s2[0] - accel_bias_[0]) * accel_axis_sign_[0];
        const double ay = (data.accel_m_s2[1] - accel_bias_[1]) * accel_axis_sign_[1];
        const double az = (data.accel_m_s2[2] - accel_bias_[2]) * accel_axis_sign_[2];

        // Xu ly tinh toan Gyro bias auto-calibration
        double gx = 0.0;
        double gy = 0.0;
        double gz = 0.0;

        if (auto_calibrate_gyro_ && !gyro_calibrated_)
        {
            gyro_bias_sum_[0] += data.gyro_rad_s[0];
            gyro_bias_sum_[1] += data.gyro_rad_s[1];
            gyro_bias_sum_[2] += data.gyro_rad_s[2];
            current_calib_count_++;

            if (current_calib_count_ >= gyro_calib_samples_)
            {
                gyro_bias_[0] = gyro_bias_sum_[0] / static_cast<double>(gyro_calib_samples_);
                gyro_bias_[1] = gyro_bias_sum_[1] / static_cast<double>(gyro_calib_samples_);
                gyro_bias_[2] = gyro_bias_sum_[2] / static_cast<double>(gyro_calib_samples_);
                gyro_calibrated_ = true;
                RCLCPP_INFO(get_logger(), "Gyro bias calibration complete (%d samples)! Bias: [gx: %.6f, gy: %.6f, gz: %.6f] rad/s",
                            gyro_calib_samples_, gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
            }
            // Trong qua trinh lay mau calibration, giu gia tri toc do goc = 0
            gx = 0.0;
            gy = 0.0;
            gz = 0.0;
        }
        else
        {
            gx = (data.gyro_rad_s[0] - gyro_bias_[0]) * gyro_axis_sign_[0];
            gy = (data.gyro_rad_s[1] - gyro_bias_[1]) * gyro_axis_sign_[1];
            gz = (data.gyro_rad_s[2] - gyro_bias_[2]) * gyro_axis_sign_[2];
        }

        double mx = (data.mag_tesla[0] - mag_bias_[0]) * mag_scale_[0] * mag_axis_sign_[0];
        double my = (data.mag_tesla[1] - mag_bias_[1]) * mag_scale_[1] * mag_axis_sign_[1];
        double mz = (data.mag_tesla[2] - mag_bias_[2]) * mag_scale_[2] * mag_axis_sign_[2];

        // Raw IMU exports identity quaternion (covariance[0] = -1.0 in controller)
        set_state(sensor_name_ + "/orientation.x", 0.0);
        set_state(sensor_name_ + "/orientation.y", 0.0);
        set_state(sensor_name_ + "/orientation.z", 0.0);
        set_state(sensor_name_ + "/orientation.w", 1.0);

        set_state(sensor_name_ + "/angular_velocity.x", gx);
        set_state(sensor_name_ + "/angular_velocity.y", gy);
        set_state(sensor_name_ + "/angular_velocity.z", gz);

        set_state(sensor_name_ + "/linear_acceleration.x", ax);
        set_state(sensor_name_ + "/linear_acceleration.y", ay);
        set_state(sensor_name_ + "/linear_acceleration.z", az);

        if (data.mag_valid)
        {
            set_state(sensor_name_ + "/magnetic_field.x", mx);
            set_state(sensor_name_ + "/magnetic_field.y", my);
            set_state(sensor_name_ + "/magnetic_field.z", mz);
        }

        return hardware_interface::return_type::OK;
    }

} // namespace lekiwi_icm20948_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lekiwi_icm20948_hardware::ICM20948SensorHardware,
    hardware_interface::SensorInterface)

/**
 * @file icm20948_sensor_hardware.cpp
 * @brief Implementation of the ros2_control ICM20948SensorHardware plugin.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

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

        // Cache state interface names to avoid heap allocations in read()
        for (size_t i = 0; i < expected_interfaces.size() && i < state_interface_names_.size(); ++i)
        {
            state_interface_names_[i] = sensor_name_ + "/" + expected_interfaces[i];
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
        }
        else
        {
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
        }

        if (get_node())
        {
            updater_ = std::make_shared<diagnostic_updater::Updater>(get_node());
            updater_->setHardwareID(sensor_name_);
            updater_->add(
                sensor_name_ + "_status", this,
                &ICM20948SensorHardware::produce_diagnostics);
            RCLCPP_INFO(get_logger(), "Diagnostic updater initialized for %s", sensor_name_.c_str());
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Default node is not available. Diagnostic updater will not be published.");
        }

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

        // Initialize shared state buffer
        {
            std::lock_guard<std::mutex> lock(shared_state_.mutex);
            shared_state_.orientation = {0.0, 0.0, 0.0, 1.0};
            shared_state_.angular_velocity = {0.0, 0.0, 0.0};
            shared_state_.linear_acceleration = {0.0, 0.0, GRAVITY_EARTH};
            shared_state_.magnetic_field = {2.0e-5, 0.0, 4.0e-5};
            shared_state_.mag_valid = true;
            shared_state_.valid = true;
            shared_state_.last_read_time = std::chrono::steady_clock::now();
        }

        // Dat gia tri khoi tao qua ten interface da duoc cache (khong cap phat heap)
        set_state(state_interface_names_[0], 0.0);
        set_state(state_interface_names_[1], 0.0);
        set_state(state_interface_names_[2], 0.0);
        set_state(state_interface_names_[3], 1.0);

        set_state(state_interface_names_[4], 0.0);
        set_state(state_interface_names_[5], 0.0);
        set_state(state_interface_names_[6], 0.0);

        set_state(state_interface_names_[7], 0.0);
        set_state(state_interface_names_[8], 0.0);
        set_state(state_interface_names_[9], GRAVITY_EARTH);

        set_state(state_interface_names_[10], 2.0e-5);
        set_state(state_interface_names_[11], 0.0);
        set_state(state_interface_names_[12], 4.0e-5);

        // Khoi dong Async Worker Thread khi chay phan cung thật
        if (!mock_sensor_)
        {
            io_running_ = true;
            io_worker_thread_ = std::thread(&ICM20948SensorHardware::io_worker_loop, this);
            RCLCPP_INFO(get_logger(), "Started ICM20948 Async I/O Worker Thread (100 Hz).");
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Xu ly khi deactivate sensor: dung worker thread an toan
    hardware_interface::CallbackReturn ICM20948SensorHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating ICM20948SensorHardware...");
        io_running_ = false;
        if (io_worker_thread_.joinable())
        {
            io_worker_thread_.join();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Chu ky RT read(): khong blocking I/O, khong cap phat dong tren heap, thoi gian thuc thi < 5 us
    hardware_interface::return_type ICM20948SensorHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (mock_sensor_)
        {
            total_reads_++;
            set_state(state_interface_names_[0], 0.0);
            set_state(state_interface_names_[1], 0.0);
            set_state(state_interface_names_[2], 0.0);
            set_state(state_interface_names_[3], 1.0);

            set_state(state_interface_names_[4], 0.0);
            set_state(state_interface_names_[5], 0.0);
            set_state(state_interface_names_[6], 0.0);

            set_state(state_interface_names_[7], 0.0);
            set_state(state_interface_names_[8], 0.0);
            set_state(state_interface_names_[9], GRAVITY_EARTH);

            set_state(state_interface_names_[10], 2.0e-5);
            set_state(state_interface_names_[11], 0.0);
            set_state(state_interface_names_[12], 4.0e-5);
            return hardware_interface::return_type::OK;
        }

        // Doc nhanh tu shared state buffer (< 5 us)
        std::array<double, 3> w;
        std::array<double, 3> a;
        std::array<double, 3> m;
        bool mag_valid = false;
        {
            std::lock_guard<std::mutex> lock(shared_state_.mutex);
            if (!shared_state_.valid)
            {
                return hardware_interface::return_type::OK;
            }
            w = shared_state_.angular_velocity;
            a = shared_state_.linear_acceleration;
            m = shared_state_.magnetic_field;
            mag_valid = shared_state_.mag_valid;
        }

        // Cap nhat 13 state interfaces qua ten da cache san, loai bo toan bo malloc/free
        set_state(state_interface_names_[0], 0.0);
        set_state(state_interface_names_[1], 0.0);
        set_state(state_interface_names_[2], 0.0);
        set_state(state_interface_names_[3], 1.0);

        set_state(state_interface_names_[4], w[0]);
        set_state(state_interface_names_[5], w[1]);
        set_state(state_interface_names_[6], w[2]);

        set_state(state_interface_names_[7], a[0]);
        set_state(state_interface_names_[8], a[1]);
        set_state(state_interface_names_[9], a[2]);

        if (mag_valid)
        {
            set_state(state_interface_names_[10], m[0]);
            set_state(state_interface_names_[11], m[1]);
            set_state(state_interface_names_[12], m[2]);
        }

        return hardware_interface::return_type::OK;
    }

    void ICM20948SensorHardware::io_worker_loop()
    {
        using namespace std::chrono_literals;
        constexpr auto loop_period = 10ms; // 100 Hz polling rate

        while (io_running_)
        {
            const auto start_time = std::chrono::steady_clock::now();
            total_reads_++;
            SensorData data;
            std::string err;

            if (!driver_.read_sensor_data(data, &err))
            {
                consecutive_errors_++;
                failed_reads_++;
            }
            else
            {
                consecutive_errors_ = 0;
                last_valid_data_ = data;

                const double ax = (data.accel_m_s2[0] - accel_bias_[0]) * accel_axis_sign_[0];
                const double ay = (data.accel_m_s2[1] - accel_bias_[1]) * accel_axis_sign_[1];
                const double az = (data.accel_m_s2[2] - accel_bias_[2]) * accel_axis_sign_[2];

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

                double mx = data.mag_tesla[0] * mag_axis_sign_[0];
                double my = data.mag_tesla[1] * mag_axis_sign_[1];
                double mz = data.mag_tesla[2] * mag_axis_sign_[2];

                {
                    std::lock_guard<std::mutex> lock(shared_state_.mutex);
                    shared_state_.angular_velocity = {gx, gy, gz};
                    shared_state_.linear_acceleration = {ax, ay, az};
                    if (data.mag_valid)
                    {
                        shared_state_.magnetic_field = {mx, my, mz};
                        shared_state_.mag_valid = true;
                    }
                    shared_state_.valid = true;
                    shared_state_.last_read_time = std::chrono::steady_clock::now();
                }
            }

            const auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < loop_period)
            {
                std::this_thread::sleep_for(loop_period - elapsed);
            }
        }
    }

    void ICM20948SensorHardware::produce_diagnostics(
        diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (mock_sensor_)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running in MOCK mode");
            stat.add("Mock Mode", "True");
            stat.add("Total Reads", total_reads_);
            return;
        }

        // 1. Danh gia trang thai suc khoe tong the
        const int err_count = consecutive_errors_.load();
        if (err_count > max_consecutive_errors_)
        {
            stat.summaryf(
                diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "I2C communication failure (%d consecutive errors)", err_count);
        }
        else if (auto_calibrate_gyro_ && !gyro_calibrated_.load())
        {
            stat.summaryf(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "Calibrating Gyro Bias (%d/%d samples)", current_calib_count_, gyro_calib_samples_);
        }
        else if (err_count > 0)
        {
            stat.summaryf(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "Transient I2C read errors (%d consecutive)", err_count);
        }
        else
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "IMU hardware OK");
        }

        // 2. Metrics toi thieu khong trung lap voi broadcaster
        std::ostringstream addr_ss;
        addr_ss << "/dev/i2c-" << driver_config_.i2c_bus << " (0x"
                << std::hex << std::uppercase << static_cast<int>(driver_config_.i2c_address) << ")";
        stat.add("I2C Device", addr_ss.str());

        stat.add("Consecutive Read Errors", static_cast<int>(consecutive_errors_.load()));
        stat.add("Total Reads", static_cast<uint64_t>(total_reads_.load()));
        stat.add("Failed Reads", static_cast<uint64_t>(failed_reads_.load()));

        const uint64_t reads = total_reads_.load();
        const uint64_t fails = failed_reads_.load();
        if (reads > 0)
        {
            double error_rate = (static_cast<double>(fails) / static_cast<double>(reads)) * 100.0;
            stat.addf("Error Rate (%)", "%.2f", error_rate);
        }
        else
        {
            stat.add("Error Rate (%)", "0.00");
        }

        if (auto_calibrate_gyro_)
        {
            stat.add("Gyro Calibrated", gyro_calibrated_ ? "Yes" : "In Progress");
            stat.addf(
                "Calculated Gyro Bias (rad/s)", "[%.6f, %.6f, %.6f]",
                gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
        }
        else
        {
            stat.add("Gyro Calibrated", "Manual Bias");
            stat.addf(
                "Configured Gyro Bias (rad/s)", "[%.6f, %.6f, %.6f]",
                gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
        }
    }

} // namespace lekiwi_icm20948_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lekiwi_icm20948_hardware::ICM20948SensorHardware,
    hardware_interface::SensorInterface)

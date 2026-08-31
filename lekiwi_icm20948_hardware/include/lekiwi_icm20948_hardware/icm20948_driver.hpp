#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lekiwi_icm20948_hardware/icm20948_defs.hpp"

namespace lekiwi_icm20948_hardware
{

    struct SensorData
    {
        double accel_m_s2[3]{0.0, 0.0, 0.0}; // x, y, z in m/s^2 (REP-103)
        double gyro_rad_s[3]{0.0, 0.0, 0.0}; // x, y, z in rad/s (REP-103)
        double mag_tesla[3]{0.0, 0.0, 0.0};  // x, y, z in Tesla (REP-103)
        bool mag_valid{false};
    };

    enum class AccelRange : uint8_t
    {
        RANGE_2G = 0,
        RANGE_4G = 1,
        RANGE_8G = 2,
        RANGE_16G = 3
    };

    enum class GyroRange : uint8_t
    {
        RANGE_250DPS = 0,
        RANGE_500DPS = 1,
        RANGE_1000DPS = 2,
        RANGE_2000DPS = 3
    };

    struct DriverConfig
    {
        int i2c_bus{1};
        uint8_t i2c_address{ICM20948_I2C_ADDR_DEFAULT};
        AccelRange accel_range{AccelRange::RANGE_4G};
        GyroRange gyro_range{GyroRange::RANGE_1000DPS};
        uint16_t accel_sample_rate_div{10}; // ~100 Hz internal
        uint8_t gyro_sample_rate_div{10};   // ~100 Hz internal
        bool dlpf_enabled{true};
        uint8_t dlpf_config{3}; // DLPF ~50 Hz bandwidth
    };

    class ICM20948Driver
    {
    public:
        ICM20948Driver() = default;
        ~ICM20948Driver();

        // Opens I2C bus and verifies device presence via WHO_AM_I
        bool open_bus(const DriverConfig &config, std::string *error_msg = nullptr);
        // Initializes registers, power management, accelerometer, gyroscope, and magnetometer
        bool configure_device(std::string *error_msg = nullptr);
        // Reads accelerometer, gyroscope, and magnetometer data in a single burst
        bool read_sensor_data(SensorData &out_data, std::string *error_msg = nullptr);
        // Closes I2C bus
        void close_bus();

        bool is_open() const { return fd_ >= 0; }
        const DriverConfig &get_config() const { return config_; }

        // Conversion Helpers (public for testing)
        static double calculate_accel_scale(AccelRange range);
        static double calculate_gyro_scale(GyroRange range);

    private:
        int fd_{-1};
        DriverConfig config_;
        uint8_t current_bank_{0xFF};
        double accel_scale_{0.0};
        double gyro_scale_{0.0};

        bool set_bank(uint8_t bank);
        bool write_byte(uint8_t bank, uint8_t reg, uint8_t val);
        bool read_byte(uint8_t bank, uint8_t reg, uint8_t &val);
        bool write_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool bit_val);
        bool read_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool &bit_val);
        bool read_block(uint8_t bank, uint8_t start_reg, uint8_t *buf, size_t length);

        // Magnetometer helpers
        bool write_mag_byte(uint8_t mag_reg, uint8_t val);
        bool read_mag_byte(uint8_t mag_reg, uint8_t &val);
        bool init_magnetometer(std::string *error_msg = nullptr);
    };

} // namespace lekiwi_icm20948_hardware

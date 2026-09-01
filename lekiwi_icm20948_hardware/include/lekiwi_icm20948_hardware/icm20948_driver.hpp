/**
 * @file icm20948_driver.hpp
 * @brief Low-level I2C hardware driver for ICM-20948 9-Axis IMU and AK09916 magnetometer.
 *
 * Implements I2C register configuration, user bank switching, burst sample reads,
 * and physical unit scaling (REP-103 standard) directly against the Linux `/dev/i2c-X` interface.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lekiwi_icm20948_hardware/icm20948_defs.hpp"

namespace lekiwi_icm20948_hardware
{

    /**
     * @brief Calibrated 9-DoF sensor reading in standard ROS REP-103 units.
     */
    struct SensorData
    {
        /// Linear acceleration vector [x, y, z] in $m/s^2$ (ENU frame: +Z points up when level).
        double accel_m_s2[3]{0.0, 0.0, 0.0};
        /// Angular velocity vector [x, y, z] in $rad/s$ (Right-hand rule).
        double gyro_rad_s[3]{0.0, 0.0, 0.0};
        /// Magnetic field vector [x, y, z] in Tesla ($T$).
        double mag_tesla[3]{0.0, 0.0, 0.0};
        /// True if magnetometer data was refreshed without DRDY timeout or magnetic overflow.
        bool mag_valid{false};
    };

    /**
     * @brief Full-scale acceleration measurement range options.
     */
    enum class AccelRange : uint8_t
    {
        /// $\pm 2g$ range (Sensitivity: 16,384 LSB/g).
        RANGE_2G = 0,
        /// $\pm 4g$ range (Sensitivity: 8,192 LSB/g).
        RANGE_4G = 1,
        /// $\pm 8g$ range (Sensitivity: 4,096 LSB/g).
        RANGE_8G = 2,
        /// $\pm 16g$ range (Sensitivity: 2,048 LSB/g).
        RANGE_16G = 3
    };

    /**
     * @brief Full-scale gyroscope measurement range options.
     */
    enum class GyroRange : uint8_t
    {
        /// $\pm 250^\circ/s$ range (Sensitivity: 131.0 LSB/(°/s)).
        RANGE_250DPS = 0,
        /// $\pm 500^\circ/s$ range (Sensitivity: 65.5 LSB/(°/s)).
        RANGE_500DPS = 1,
        /// $\pm 1000^\circ/s$ range (Sensitivity: 32.8 LSB/(°/s)).
        RANGE_1000DPS = 2,
        /// $\pm 2000^\circ/s$ range (Sensitivity: 16.4 LSB/(°/s)).
        RANGE_2000DPS = 3
    };

    /**
     * @brief Runtime hardware configuration parameters for the ICM-20948 driver.
     */
    struct DriverConfig
    {
        /// Linux I2C bus index (e.g., 1 for `/dev/i2c-1`).
        int i2c_bus{1};
        /// Primary 7-bit I2C device address (0x68 or 0x69).
        uint8_t i2c_address{ICM20948_I2C_ADDR_DEFAULT};
        /// Full-scale range for accelerometer channels.
        AccelRange accel_range{AccelRange::RANGE_4G};
        /// Full-scale range for gyroscope channels.
        GyroRange gyro_range{GyroRange::RANGE_1000DPS};
        /// Accelerometer internal sample rate divider (1125Hz / (1 + div)).
        uint16_t accel_sample_rate_div{10};
        /// Gyroscope internal sample rate divider (1100Hz / (1 + div)).
        uint8_t gyro_sample_rate_div{10};
        /// Enable hardware Digital Low Pass Filter (DLPF).
        bool dlpf_enabled{true};
        /// DLPF bandwidth configuration code (0..7).
        uint8_t dlpf_config{3};
    };

    /**
     * @brief Direct I2C driver for InvenSense ICM-20948 and AK09916 magnetometer.
     *
     * Handles file descriptor management, bank switching caching, register read/write,
     * auxiliary I2C master configuration for AK09916, and unit conversion.
     */
    class ICM20948Driver
    {
    public:
        /// Default constructor.
        ICM20948Driver() = default;
        /// Destructor. Automatically closes the I2C file descriptor if open.
        ~ICM20948Driver();

        /**
         * @brief Opens the Linux I2C bus device file and verifies chip identity.
         *
         * @param[in] config Target configuration specifying bus number and device address.
         * @param[out] error_msg Optional string capturing diagnostics if open or WHO_AM_I check fails.
         * @return true If bus opened and device returned expected WHO_AM_I (0xEA), false otherwise.
         */
        bool open_bus(const DriverConfig &config, std::string *error_msg = nullptr);

        /**
         * @brief Configures power management, clock source, DLPF, scale ranges, and magnetometer.
         *
         * @param[out] error_msg Optional string capturing error details on failure.
         * @return true If all registers were initialized and magnetometer responded, false otherwise.
         */
        bool configure_device(std::string *error_msg = nullptr);

        /**
         * @brief Reads accelerometer, gyroscope, and magnetometer in a single contiguous burst.
         *
         * Reads 12 bytes of Accel+Gyro data from Bank 0, plus 9 bytes of AK09916 data from external sensor buffers.
         * Decodes Big-Endian (Accel/Gyro) and Little-Endian (Mag) binary fields into SI units.
         *
         * @param[out] out_data Output data structure receiving converted readings.
         * @param[out] error_msg Optional string capturing diagnostics on communication failure.
         * @return true If burst read succeeded, false otherwise.
         */
        bool read_sensor_data(SensorData &out_data, std::string *error_msg = nullptr);

        /**
         * @brief Closes the I2C bus file descriptor.
         */
        void close_bus();

        /**
         * @brief Checks if the I2C device is currently open.
         * @return true If @ref fd_ is valid (>= 0), false otherwise.
         */
        bool is_open() const { return fd_ >= 0; }

        /**
         * @brief Returns active driver configuration.
         * @return const DriverConfig& Active configuration reference.
         */
        const DriverConfig &get_config() const { return config_; }

        /**
         * @brief Calculates scaling multiplier from raw 16-bit integer to $m/s^2$.
         *
         * @param[in] range Selected accelerometer full-scale range.
         * @return double Conversion factor ($m/s^2 / \text{LSB}$).
         */
        static double calculate_accel_scale(AccelRange range);

        /**
         * @brief Calculates scaling multiplier from raw 16-bit integer to $rad/s$.
         *
         * @param[in] range Selected gyroscope full-scale range.
         * @return double Conversion factor ($rad/s / \text{LSB}$).
         */
        static double calculate_gyro_scale(GyroRange range);

    private:
        int fd_{-1};
        DriverConfig config_;
        uint8_t current_bank_{0xFF};
        double accel_scale_{0.0};
        double gyro_scale_{0.0};

        /**
         * @brief Switches the active register bank (0..3) if different from @ref current_bank_.
         *
         * @param[in] bank Target bank constant (BANK_0, BANK_1, BANK_2, or BANK_3).
         * @return true If bank switch succeeded or was already active, false on I2C error.
         */
        bool set_bank(uint8_t bank);

        /**
         * @brief Writes a single byte to a register in the specified bank.
         *
         * @param[in] bank Target register bank.
         * @param[in] reg Register address.
         * @param[in] val Byte value to write.
         * @return true On success, false on error.
         */
        bool write_byte(uint8_t bank, uint8_t reg, uint8_t val);

        /**
         * @brief Reads a single byte from a register in the specified bank.
         *
         * @param[in] bank Target register bank.
         * @param[in] reg Register address.
         * @param[out] val Output reference receiving the read byte.
         * @return true On success, false on error.
         */
        bool read_byte(uint8_t bank, uint8_t reg, uint8_t &val);

        /**
         * @brief Modifies a single bit in a register while preserving other bits.
         */
        bool write_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool bit_val);

        /**
         * @brief Reads a specific bit from a register.
         */
        bool read_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool &bit_val);

        /**
         * @brief Reads a contiguous block of bytes from starting register.
         */
        bool read_block(uint8_t bank, uint8_t start_reg, uint8_t *buf, size_t length);

        // Auxiliary Magnetometer Helpers
        /**
         * @brief Writes a byte to AK09916 via ICM-20948 I2C Slave 4 one-shot peripheral.
         */
        bool write_mag_byte(uint8_t mag_reg, uint8_t val);

        /**
         * @brief Reads a byte from AK09916 via ICM-20948 I2C Slave 4 peripheral.
         */
        bool read_mag_byte(uint8_t mag_reg, uint8_t &val);

        /**
         * @brief Initializes AK09916 auxiliary I2C master slave mappings and continuous mode.
         */
        bool init_magnetometer(std::string *error_msg = nullptr);
    };

} // namespace lekiwi_icm20948_hardware

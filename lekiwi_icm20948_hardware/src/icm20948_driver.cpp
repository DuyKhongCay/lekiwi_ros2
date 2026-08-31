#include "lekiwi_icm20948_hardware/icm20948_driver.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace lekiwi_icm20948_hardware
{

    ICM20948Driver::~ICM20948Driver()
    {
        close_bus();
    }

    double ICM20948Driver::calculate_accel_scale(AccelRange range)
    {
        switch (range)
        {
        case AccelRange::RANGE_2G:
            return GRAVITY_EARTH / 16384.0;
        case AccelRange::RANGE_4G:
            return GRAVITY_EARTH / 8192.0;
        case AccelRange::RANGE_8G:
            return GRAVITY_EARTH / 4096.0;
        case AccelRange::RANGE_16G:
            return GRAVITY_EARTH / 2048.0;
        default:
            return GRAVITY_EARTH / 8192.0;
        }
    }

    double ICM20948Driver::calculate_gyro_scale(GyroRange range)
    {
        switch (range)
        {
        case GyroRange::RANGE_250DPS:
            return (1.0 / 131.0) * DEG_TO_RAD;
        case GyroRange::RANGE_500DPS:
            return (1.0 / 65.5) * DEG_TO_RAD;
        case GyroRange::RANGE_1000DPS:
            return (1.0 / 32.8) * DEG_TO_RAD;
        case GyroRange::RANGE_2000DPS:
            return (1.0 / 16.4) * DEG_TO_RAD;
        default:
            return (1.0 / 32.8) * DEG_TO_RAD;
        }
    }

    void ICM20948Driver::close_bus()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
        current_bank_ = 0xFF;
    }

    bool ICM20948Driver::open_bus(const DriverConfig &config, std::string *error_msg)
    {
        close_bus();
        config_ = config;
        accel_scale_ = calculate_accel_scale(config_.accel_range);
        gyro_scale_ = calculate_gyro_scale(config_.gyro_range);

        const std::string bus_path = "/dev/i2c-" + std::to_string(config_.i2c_bus);
        fd_ = ::open(bus_path.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            if (error_msg)
            {
                *error_msg = "Cannot open " + bus_path + ": " + std::strerror(errno);
            }
            return false;
        }

        if (::ioctl(fd_, I2C_SLAVE, config_.i2c_address) < 0)
        {
            if (error_msg)
            {
                *error_msg = "Cannot set I2C slave address 0x" + std::to_string(config_.i2c_address) + ": " + std::strerror(errno);
            }
            close_bus();
            return false;
        }

        current_bank_ = 0xFF;
        return true;
    }

    bool ICM20948Driver::set_bank(uint8_t bank)
    {
        if (fd_ < 0)
            return false;
        if (current_bank_ == bank)
            return true;

        uint8_t buf[2] = {REG_BANK_SEL, bank};
        if (::write(fd_, buf, 2) != 2)
        {
            return false;
        }
        current_bank_ = bank;
        return true;
    }

    bool ICM20948Driver::write_byte(uint8_t bank, uint8_t reg, uint8_t val)
    {
        if (!set_bank(bank))
            return false;
        uint8_t buf[2] = {reg, val};
        return (::write(fd_, buf, 2) == 2);
    }

    bool ICM20948Driver::read_byte(uint8_t bank, uint8_t reg, uint8_t &val)
    {
        return read_block(bank, reg, &val, 1);
    }

    bool ICM20948Driver::write_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool bit_val)
    {
        uint8_t prev_val = 0;
        if (!read_byte(bank, reg, prev_val))
            return false;
        uint8_t new_val = (prev_val & ~(1 << bit_pos)) | ((bit_val ? 1 : 0) << bit_pos);
        return write_byte(bank, reg, new_val);
    }

    bool ICM20948Driver::read_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool &bit_val)
    {
        uint8_t val = 0;
        if (!read_byte(bank, reg, val))
            return false;
        bit_val = ((val >> bit_pos) & 0x01) != 0;
        return true;
    }

    bool ICM20948Driver::read_block(uint8_t bank, uint8_t start_reg, uint8_t *buf, size_t length)
    {
        if (!set_bank(bank))
            return false;

        struct i2c_msg msgs[2];
        msgs[0].addr = config_.i2c_address;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &start_reg;

        msgs[1].addr = config_.i2c_address;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = static_cast<__u16>(length);
        msgs[1].buf = buf;

        struct i2c_rdwr_ioctl_data rdwr;
        rdwr.msgs = msgs;
        rdwr.nmsgs = 2;

        return (::ioctl(fd_, I2C_RDWR, &rdwr) >= 0);
    }

    bool ICM20948Driver::write_mag_byte(uint8_t mag_reg, uint8_t val)
    {
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_ADDR, AK09916_MAG_I2C_ADDR))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_REG, mag_reg))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_DO, val))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_CTRL, 0x80))
            return false;

        bool done = false;
        for (int i = 0; i < 20; ++i)
        {
            if (read_bit(BANK_0, REG_B0_I2C_MST_STATUS, 6, done) && done)
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }

    bool ICM20948Driver::read_mag_byte(uint8_t mag_reg, uint8_t &val)
    {
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_ADDR, AK09916_MAG_I2C_ADDR | 0x80))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_REG, mag_reg))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV4_CTRL, 0x80))
            return false;

        bool done = false;
        for (int i = 0; i < 20; ++i)
        {
            if (read_bit(BANK_0, REG_B0_I2C_MST_STATUS, 6, done) && done)
            {
                return read_byte(BANK_3, REG_B3_I2C_SLV4_DI, val);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }

    bool ICM20948Driver::init_magnetometer(std::string *error_msg)
    {
        // Disable I2C master bypass
        if (!write_bit(BANK_0, REG_B0_INT_PIN_CFG, 1, false))
        {
            if (error_msg)
                *error_msg = "Failed to disable I2C bypass";
            return false;
        }

        // Set I2C master clock (~345.6 kHz)
        if (!write_byte(BANK_3, REG_B3_I2C_MST_CTRL, 0x17))
        {
            if (error_msg)
                *error_msg = "Failed to configure I2C master clock";
            return false;
        }

        // Enable I2C master in USER_CTRL
        if (!write_bit(BANK_0, REG_B0_USER_CTRL, 5, true))
        {
            if (error_msg)
                *error_msg = "Failed to enable I2C master mode";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Reset AK09916
        write_mag_byte(REG_AK09916_CNTL3, AK09916_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Verify AK09916 WIA2
        uint8_t mag_id = 0;
        bool mag_detected = false;
        for (int i = 0; i < 5; ++i)
        {
            if (read_mag_byte(REG_AK09916_WIA2, mag_id) && mag_id == AK09916_WIA2_VALUE)
            {
                mag_detected = true;
                break;
            }
            // Reset chip I2C master if communication fails
            write_bit(BANK_0, REG_B0_USER_CTRL, 1, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        if (!mag_detected)
        {
            if (error_msg)
            {
                *error_msg = "AK09916 magnetometer not detected (WIA2=0x" + std::to_string(mag_id) + ", expected 0x09)";
            }
            return false;
        }

        // Set continuous measurement mode (100 Hz)
        if (!write_mag_byte(REG_AK09916_CNTL2, AK09916_MODE_POWER_DOWN))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (!write_mag_byte(REG_AK09916_CNTL2, AK09916_MODE_CONT_100HZ))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        // Configure SLV0 to automatically read 9 bytes starting from ST1 (0x10) to ST2 (0x18)
        if (!write_byte(BANK_3, REG_B3_I2C_SLV0_ADDR, AK09916_MAG_I2C_ADDR | 0x80))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV0_REG, REG_AK09916_ST1))
            return false;
        if (!write_byte(BANK_3, REG_B3_I2C_SLV0_CTRL, 0x89))
            return false; // Enable + 9 bytes

        return true;
    }

    bool ICM20948Driver::configure_device(std::string *error_msg)
    {
        if (fd_ < 0)
        {
            if (error_msg)
                *error_msg = "I2C bus not open";
            return false;
        }

        // 1. Verify WHO_AM_I
        uint8_t who_am_i = 0;
        if (!read_byte(BANK_0, REG_B0_WHO_AM_I, who_am_i) || who_am_i != ICM20948_WHO_AM_I_VALUE)
        {
            if (error_msg)
            {
                *error_msg = "Invalid WHO_AM_I: 0x" + std::to_string(who_am_i) + " (expected 0xEA)";
            }
            return false;
        }

        // 2. Soft Reset
        if (!write_bit(BANK_0, REG_B0_PWR_MGMT_1, 7, true))
        {
            if (error_msg)
                *error_msg = "Failed to issue soft reset";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool resetting = true;
        for (int i = 0; i < 20; ++i)
        {
            if (read_bit(BANK_0, REG_B0_PWR_MGMT_1, 7, resetting) && !resetting)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        current_bank_ = BANK_0;

        // 3. Wake up device and select best clock (Auto PLL)
        if (!write_byte(BANK_0, REG_B0_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO))
        {
            if (error_msg)
                *error_msg = "Failed to wake up ICM20948";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 4. Configure Accelerometer
        const uint8_t accel_msb = static_cast<uint8_t>((config_.accel_sample_rate_div >> 8) & 0x0F);
        const uint8_t accel_lsb = static_cast<uint8_t>(config_.accel_sample_rate_div & 0xFF);
        if (!write_byte(BANK_2, REG_B2_ACCEL_SMPLRT_DIV_1, accel_msb) ||
            !write_byte(BANK_2, REG_B2_ACCEL_SMPLRT_DIV_2, accel_lsb))
        {
            if (error_msg)
                *error_msg = "Failed to set accelerometer sample rate divider";
            return false;
        }

        const uint8_t accel_cfg = (static_cast<uint8_t>(config_.accel_range) << 1) |
                                  (config_.dlpf_config << 3) |
                                  (config_.dlpf_enabled ? 0x01 : 0x00);
        if (!write_byte(BANK_2, REG_B2_ACCEL_CONFIG, accel_cfg))
        {
            if (error_msg)
                *error_msg = "Failed to set accelerometer range and DLPF";
            return false;
        }

        // 5. Configure Gyroscope
        if (!write_byte(BANK_2, REG_B2_GYRO_SMPLRT_DIV, config_.gyro_sample_rate_div))
        {
            if (error_msg)
                *error_msg = "Failed to set gyroscope sample rate divider";
            return false;
        }

        const uint8_t gyro_cfg = (static_cast<uint8_t>(config_.gyro_range) << 1) |
                                 (config_.dlpf_config << 3) |
                                 (config_.dlpf_enabled ? 0x01 : 0x00);
        if (!write_byte(BANK_2, REG_B2_GYRO_CONFIG_1, gyro_cfg))
        {
            if (error_msg)
                *error_msg = "Failed to set gyroscope range and DLPF";
            return false;
        }

        // 6. Initialize Magnetometer
        if (!init_magnetometer(error_msg))
        {
            return false;
        }

        return true;
    }

    bool ICM20948Driver::read_sensor_data(SensorData &out_data, std::string *error_msg)
    {
        if (fd_ < 0)
        {
            if (error_msg)
                *error_msg = "I2C bus not open";
            return false;
        }

        // Burst read 23 bytes: Accel (6) + Gyro (6) + Temp (2) + Mag SLV0 Buffer (9)
        uint8_t buf[23];
        if (!read_block(BANK_0, REG_B0_ACCEL_XOUT_H, buf, 23))
        {
            if (error_msg)
                *error_msg = "I2C burst read failed: " + std::string(std::strerror(errno));
            return false;
        }

        // Big-endian 16-bit accel
        const int16_t raw_ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
        const int16_t raw_ay = static_cast<int16_t>((buf[2] << 8) | buf[3]);
        const int16_t raw_az = static_cast<int16_t>((buf[4] << 8) | buf[5]);

        // Big-endian 16-bit gyro
        const int16_t raw_gx = static_cast<int16_t>((buf[6] << 8) | buf[7]);
        const int16_t raw_gy = static_cast<int16_t>((buf[8] << 8) | buf[9]);
        const int16_t raw_gz = static_cast<int16_t>((buf[10] << 8) | buf[11]);

        out_data.accel_m_s2[0] = static_cast<double>(raw_ax) * accel_scale_;
        out_data.accel_m_s2[1] = static_cast<double>(raw_ay) * accel_scale_;
        out_data.accel_m_s2[2] = static_cast<double>(raw_az) * accel_scale_;

        out_data.gyro_rad_s[0] = static_cast<double>(raw_gx) * gyro_scale_;
        out_data.gyro_rad_s[1] = static_cast<double>(raw_gy) * gyro_scale_;
        out_data.gyro_rad_s[2] = static_cast<double>(raw_gz) * gyro_scale_;

        // Magnetometer buffer (offset 14: ST1 at buf[14], HXL at buf[15]..ST2 at buf[22])
        const uint8_t st1 = buf[14];
        const uint8_t st2 = buf[22];
        const bool overflow = (st2 & 0x08) != 0;
        const bool data_ready = (st1 & 0x01) != 0;

        if (data_ready && !overflow)
        {
            // Little-endian 16-bit mag
            const int16_t raw_mx = static_cast<int16_t>(buf[15] | (buf[16] << 8));
            const int16_t raw_my = static_cast<int16_t>(buf[17] | (buf[18] << 8));
            const int16_t raw_mz = static_cast<int16_t>(buf[19] | (buf[20] << 8));

            out_data.mag_tesla[0] = static_cast<double>(raw_mx) * MAG_LSB_TO_TESLA;
            out_data.mag_tesla[1] = static_cast<double>(raw_my) * MAG_LSB_TO_TESLA;
            out_data.mag_tesla[2] = static_cast<double>(raw_mz) * MAG_LSB_TO_TESLA;
            out_data.mag_valid = true;
        }
        else
        {
            out_data.mag_valid = false;
        }

        return true;
    }

} // namespace lekiwi_icm20948_hardware

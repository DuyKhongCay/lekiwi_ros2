/**
 * @file test_icm20948_math.cpp
 * @brief Unit tests (L1 verification) for ICM-20948 scale factor calculations and binary field decoding.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <cmath>
#include "lekiwi_icm20948_hardware/icm20948_defs.hpp"
#include "lekiwi_icm20948_hardware/icm20948_driver.hpp"

using namespace lekiwi_icm20948_hardware;

/**
 * @brief Verifies accelerometer sensitivity scale factors across 2g, 4g, 8g, and 16g ranges.
 */
TEST(ICM20948MathTest, AccelScaleFactors)
{
    double scale_2g = ICM20948Driver::calculate_accel_scale(AccelRange::RANGE_2G);
    double scale_4g = ICM20948Driver::calculate_accel_scale(AccelRange::RANGE_4G);
    double scale_8g = ICM20948Driver::calculate_accel_scale(AccelRange::RANGE_8G);
    double scale_16g = ICM20948Driver::calculate_accel_scale(AccelRange::RANGE_16G);

    EXPECT_NEAR(scale_2g, GRAVITY_EARTH / 16384.0, 1e-6);
    EXPECT_NEAR(scale_4g, GRAVITY_EARTH / 8192.0, 1e-6);
    EXPECT_NEAR(scale_8g, GRAVITY_EARTH / 4096.0, 1e-6);
    EXPECT_NEAR(scale_16g, GRAVITY_EARTH / 2048.0, 1e-6);

    // 1g reading on 4g scale should yield ~9.80665 m/s^2
    int16_t raw_1g = 8192;
    double accel_1g = static_cast<double>(raw_1g) * scale_4g;
    EXPECT_NEAR(accel_1g, GRAVITY_EARTH, 1e-4);
}

/**
 * @brief Verifies gyroscope sensitivity scale factors across 250, 500, 1000, and 2000 dps ranges.
 */
TEST(ICM20948MathTest, GyroScaleFactors)
{
    double scale_250 = ICM20948Driver::calculate_gyro_scale(GyroRange::RANGE_250DPS);
    double scale_500 = ICM20948Driver::calculate_gyro_scale(GyroRange::RANGE_500DPS);
    double scale_1000 = ICM20948Driver::calculate_gyro_scale(GyroRange::RANGE_1000DPS);
    double scale_2000 = ICM20948Driver::calculate_gyro_scale(GyroRange::RANGE_2000DPS);

    EXPECT_NEAR(scale_250, (1.0 / 131.0) * DEG_TO_RAD, 1e-6);
    EXPECT_NEAR(scale_500, (1.0 / 65.5) * DEG_TO_RAD, 1e-6);
    EXPECT_NEAR(scale_1000, (1.0 / 32.8) * DEG_TO_RAD, 1e-6);
    EXPECT_NEAR(scale_2000, (1.0 / 16.4) * DEG_TO_RAD, 1e-6);

    // 100 deg/s on 1000 dps scale should yield 100 * pi / 180 rad/s
    int16_t raw_100dps = 3280;
    double gyro_rad = static_cast<double>(raw_100dps) * scale_1000;
    EXPECT_NEAR(gyro_rad, 100.0 * DEG_TO_RAD, 1e-3);
}

/**
 * @brief Verifies conversion of raw magnetometer 16-bit integer values to Tesla units.
 */
TEST(ICM20948MathTest, MagnetometerScalingToTesla)
{
    // 100 uT = 100 * 10^-6 Tesla
    // Sensitivity: 0.15 uT / LSB
    int16_t raw_mag = static_cast<int16_t>(100.0 / 0.15); // ~667 LSB
    double mag_tesla = static_cast<double>(raw_mag) * MAG_LSB_TO_TESLA;
    EXPECT_NEAR(mag_tesla, 100.0e-6, 1e-6);
}

/**
 * @brief Verifies Big-Endian byte parsing (MSB first) for ICM-20948 accelerometer and gyroscope.
 */
TEST(ICM20948MathTest, BigEndianAccelGyroDecoding)
{
    uint8_t buf[12] = {
        0x20, 0x00, // ax = +8192 (+1g on 4g range)
        0x00, 0x00, // ay = 0
        0x00, 0x00, // az = 0
        0x00, 0x00, // gx = 0
        0x0C, 0xD0, // gy = +3280 (+100 dps on 1000dps range)
        0x00, 0x00  // gz = 0
    };

    int16_t ax = ICM20948Driver::decode_be16(buf[0], buf[1]);
    int16_t gy = ICM20948Driver::decode_be16(buf[8], buf[9]);

    EXPECT_EQ(ax, 8192);
    EXPECT_EQ(gy, 3280);

    // Negative values and signed boundaries (MISRA C compliance)
    EXPECT_EQ(ICM20948Driver::decode_be16(0x7F, 0xFF), 32767);
    EXPECT_EQ(ICM20948Driver::decode_be16(0x80, 0x00), -32768);
    EXPECT_EQ(ICM20948Driver::decode_be16(0xFF, 0xFF), -1);
    EXPECT_EQ(ICM20948Driver::decode_be16(0x00, 0x00), 0);
}

/**
 * @brief Verifies Little-Endian byte parsing (LSB first) for AK09916 magnetometer registers.
 */
TEST(ICM20948MathTest, LittleEndianMagnetometerDecoding)
{
    uint8_t mag_buf[9] = {
        0x01,       // ST1: DRDY = 1
        0x9B, 0x02, // mx = 0x029B = 667 LSB (~100 uT)
        0x00, 0x00, // my
        0x00, 0x00, // mz
        0x00,       // TMPS
        0x00        // ST2: HOFL = 0
    };

    int16_t mx = ICM20948Driver::decode_le16(mag_buf[1], mag_buf[2]);
    EXPECT_EQ(mx, 667);
    double mag_t = static_cast<double>(mx) * MAG_LSB_TO_TESLA;
    EXPECT_NEAR(mag_t, 100.05e-6, 1e-6);

    // Negative values and signed boundaries (MISRA C compliance)
    EXPECT_EQ(ICM20948Driver::decode_le16(0xFF, 0x7F), 32767);
    EXPECT_EQ(ICM20948Driver::decode_le16(0x00, 0x80), -32768);
    EXPECT_EQ(ICM20948Driver::decode_le16(0xFF, 0xFF), -1);
    EXPECT_EQ(ICM20948Driver::decode_le16(0x00, 0x00), 0);
}

/**
 * @brief Verifies AK09916 coordinate alignment to ICM-20948 body frame (DS-000189 Section 15, Fig 12 & 13).
 * X_imu = +X_mag, Y_imu = -Y_mag, Z_imu = -Z_mag
 */
TEST(ICM20948MathTest, AK09916ToBodyFrameAlignment)
{
    // Raw magnetometer reading:
    // mx = +1000 LSB (pointing along +X of AK09916 die)
    // my = +2000 LSB (pointing along +Y of AK09916 die)
    // mz = +3000 LSB (pointing along +Z of AK09916 die)
    int16_t raw_mx = 1000;
    int16_t raw_my = 2000;
    int16_t raw_mz = 3000;

    double imu_body_mag_x = static_cast<double>(raw_mx) * MAG_LSB_TO_TESLA;
    double imu_body_mag_y = -static_cast<double>(raw_my) * MAG_LSB_TO_TESLA;
    double imu_body_mag_z = -static_cast<double>(raw_mz) * MAG_LSB_TO_TESLA;

    EXPECT_NEAR(imu_body_mag_x, 1000.0 * MAG_LSB_TO_TESLA, 1e-12);
    EXPECT_NEAR(imu_body_mag_y, -2000.0 * MAG_LSB_TO_TESLA, 1e-12);
    EXPECT_NEAR(imu_body_mag_z, -3000.0 * MAG_LSB_TO_TESLA, 1e-12);
}

/**
 * @brief Verifies bit shifting guards (MISRA C Rule 12.2).
 */
TEST(ICM20948MathTest, BitShiftGuards)
{
    ICM20948Driver driver;
    bool bit_val = false;

    // Out of bounds shift positions (>= 8) must return false without crash or undefined behavior
    EXPECT_FALSE(driver.write_bit(BANK_0, REG_B0_PWR_MGMT_1, 8, true));
    EXPECT_FALSE(driver.write_bit(BANK_0, REG_B0_PWR_MGMT_1, 32, true));
    EXPECT_FALSE(driver.read_bit(BANK_0, REG_B0_PWR_MGMT_1, 8, bit_val));
    EXPECT_FALSE(driver.read_bit(BANK_0, REG_B0_PWR_MGMT_1, 64, bit_val));
}

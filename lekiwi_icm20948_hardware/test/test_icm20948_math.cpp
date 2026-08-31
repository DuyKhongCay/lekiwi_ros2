#include <gtest/gtest.h>
#include <cmath>
#include "lekiwi_icm20948_hardware/icm20948_defs.hpp"
#include "lekiwi_icm20948_hardware/icm20948_driver.hpp"

using namespace lekiwi_icm20948_hardware;

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

TEST(ICM20948MathTest, MagnetometerScalingToTesla)
{
    // 100 uT = 100 * 10^-6 Tesla
    // Sensitivity: 0.15 uT / LSB
    int16_t raw_mag = static_cast<int16_t>(100.0 / 0.15); // ~667 LSB
    double mag_tesla = static_cast<double>(raw_mag) * MAG_LSB_TO_TESLA;
    EXPECT_NEAR(mag_tesla, 100.0e-6, 1e-6);
}

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

    int16_t ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    int16_t gy = static_cast<int16_t>((buf[8] << 8) | buf[9]);

    EXPECT_EQ(ax, 8192);
    EXPECT_EQ(gy, 3280);
}

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

    int16_t mx = static_cast<int16_t>(mag_buf[1] | (mag_buf[2] << 8));
    EXPECT_EQ(mx, 667);
    double mag_t = static_cast<double>(mx) * MAG_LSB_TO_TESLA;
    EXPECT_NEAR(mag_t, 100.05e-6, 1e-6);
}

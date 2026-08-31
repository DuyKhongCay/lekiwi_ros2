#pragma once

#include <cstdint>

namespace lekiwi_icm20948_hardware
{

    // I2C Addresses
    constexpr uint8_t ICM20948_I2C_ADDR_DEFAULT = 0x68; // AD0 = 0
    constexpr uint8_t ICM20948_I2C_ADDR_ALT = 0x69;     // AD0 = 1
    constexpr uint8_t AK09916_MAG_I2C_ADDR = 0x0C;      // Magnetometer auxiliary I2C address

    // Register Bank Selection
    constexpr uint8_t REG_BANK_SEL = 0x7F;
    constexpr uint8_t BANK_0 = 0x00;
    constexpr uint8_t BANK_1 = 0x10;
    constexpr uint8_t BANK_2 = 0x20;
    constexpr uint8_t BANK_3 = 0x30;

    // Bank 0 Registers
    constexpr uint8_t REG_B0_WHO_AM_I = 0x00;
    constexpr uint8_t REG_B0_USER_CTRL = 0x03;
    constexpr uint8_t REG_B0_PWR_MGMT_1 = 0x06;
    constexpr uint8_t REG_B0_PWR_MGMT_2 = 0x07;
    constexpr uint8_t REG_B0_INT_PIN_CFG = 0x0F;
    constexpr uint8_t REG_B0_I2C_MST_STATUS = 0x17;
    constexpr uint8_t REG_B0_ACCEL_XOUT_H = 0x2D;
    constexpr uint8_t REG_B0_ACCEL_XOUT_L = 0x2E;
    constexpr uint8_t REG_B0_ACCEL_YOUT_H = 0x2F;
    constexpr uint8_t REG_B0_ACCEL_YOUT_L = 0x30;
    constexpr uint8_t REG_B0_ACCEL_ZOUT_H = 0x31;
    constexpr uint8_t REG_B0_ACCEL_ZOUT_L = 0x32;
    constexpr uint8_t REG_B0_GYRO_XOUT_H = 0x33;
    constexpr uint8_t REG_B0_GYRO_XOUT_L = 0x34;
    constexpr uint8_t REG_B0_GYRO_YOUT_H = 0x35;
    constexpr uint8_t REG_B0_GYRO_YOUT_L = 0x36;
    constexpr uint8_t REG_B0_GYRO_ZOUT_H = 0x37;
    constexpr uint8_t REG_B0_GYRO_ZOUT_L = 0x38;
    constexpr uint8_t REG_B0_TEMP_OUT_H = 0x39;
    constexpr uint8_t REG_B0_TEMP_OUT_L = 0x3A;
    constexpr uint8_t REG_B0_EXT_SLV_SENS_DATA = 0x3B; // 24-byte external sensor buffer (0x3B - 0x52)

    // Bank 2 Registers
    constexpr uint8_t REG_B2_GYRO_SMPLRT_DIV = 0x00;
    constexpr uint8_t REG_B2_GYRO_CONFIG_1 = 0x01;
    constexpr uint8_t REG_B2_ACCEL_SMPLRT_DIV_1 = 0x10;
    constexpr uint8_t REG_B2_ACCEL_SMPLRT_DIV_2 = 0x11;
    constexpr uint8_t REG_B2_ACCEL_CONFIG = 0x14;

    // Bank 3 Registers
    constexpr uint8_t REG_B3_I2C_MST_CTRL = 0x01;
    constexpr uint8_t REG_B3_I2C_SLV0_ADDR = 0x03;
    constexpr uint8_t REG_B3_I2C_SLV0_REG = 0x04;
    constexpr uint8_t REG_B3_I2C_SLV0_CTRL = 0x05;
    constexpr uint8_t REG_B3_I2C_SLV4_ADDR = 0x13;
    constexpr uint8_t REG_B3_I2C_SLV4_REG = 0x14;
    constexpr uint8_t REG_B3_I2C_SLV4_CTRL = 0x15;
    constexpr uint8_t REG_B3_I2C_SLV4_DO = 0x16;
    constexpr uint8_t REG_B3_I2C_SLV4_DI = 0x17;

    // ICM-20948 Constants & Bitfields
    constexpr uint8_t ICM20948_WHO_AM_I_VALUE = 0xEA;
    constexpr uint8_t PWR_MGMT_1_RESET = 0x80;
    constexpr uint8_t PWR_MGMT_1_SLEEP = 0x40;
    constexpr uint8_t PWR_MGMT_1_CLKSEL_AUTO = 0x01;
    constexpr uint8_t USER_CTRL_I2C_MST_EN = 0x20;
    constexpr uint8_t USER_CTRL_I2C_MST_RST = 0x02;
    constexpr uint8_t INT_PIN_CFG_BYPASS_EN = 0x02;

    // AK09916 Magnetometer Registers
    constexpr uint8_t REG_AK09916_WIA2 = 0x01;
    constexpr uint8_t REG_AK09916_ST1 = 0x10;
    constexpr uint8_t REG_AK09916_HXL = 0x11;
    constexpr uint8_t REG_AK09916_ST2 = 0x18;
    constexpr uint8_t REG_AK09916_CNTL2 = 0x31;
    constexpr uint8_t REG_AK09916_CNTL3 = 0x32;

    // AK09916 Constants
    constexpr uint8_t AK09916_WIA2_VALUE = 0x09;
    constexpr uint8_t AK09916_MODE_POWER_DOWN = 0x00;
    constexpr uint8_t AK09916_MODE_CONT_10HZ = 0x02;
    constexpr uint8_t AK09916_MODE_CONT_20HZ = 0x04;
    constexpr uint8_t AK09916_MODE_CONT_50HZ = 0x06;
    constexpr uint8_t AK09916_MODE_CONT_100HZ = 0x08;
    constexpr uint8_t AK09916_RESET = 0x01;

    // Standard Physical Conversion Constants
    constexpr double GRAVITY_EARTH = 9.80665;           // 1g in m/s^2
    constexpr double DEG_TO_RAD = 0.017453292519943295; // pi / 180
    constexpr double MAG_LSB_TO_TESLA = 0.15e-6;        // 0.15 uT / LSB -> Tesla

} // namespace lekiwi_icm20948_hardware

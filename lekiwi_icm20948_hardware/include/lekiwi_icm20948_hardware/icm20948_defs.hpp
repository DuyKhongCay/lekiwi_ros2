/**
 * @file icm20948_defs.hpp
 * @brief Register maps, bank selection addresses, and physical constants for ICM-20948 and AK09916.
 *
 * Provides symbolic constants for register addresses across Banks 0..3 of the TDK InvenSense
 * ICM-20948 9-Axis MotionTracking device and its embedded Asahi Kasei AK09916 magnetometer.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cstdint>

namespace lekiwi_icm20948_hardware
{

    /// Default primary I2C 7-bit slave address when AD0 pin is tied LOW (GND).
    constexpr uint8_t ICM20948_I2C_ADDR_DEFAULT = 0x68;
    /// Alternate I2C 7-bit slave address when AD0 pin is tied HIGH (VCC).
    constexpr uint8_t ICM20948_I2C_ADDR_ALT = 0x69;
    /// Dedicated auxiliary I2C 7-bit slave address for embedded AK09916 magnetometer.
    constexpr uint8_t AK09916_MAG_I2C_ADDR = 0x0C;

    /// Register Bank Selection Register (Common to all banks at address 0x7F).
    constexpr uint8_t REG_BANK_SEL = 0x7F;
    /// Value written to REG_BANK_SEL to select User Bank 0.
    constexpr uint8_t BANK_0 = 0x00;
    /// Value written to REG_BANK_SEL to select User Bank 1.
    constexpr uint8_t BANK_1 = 0x10;
    /// Value written to REG_BANK_SEL to select User Bank 2.
    constexpr uint8_t BANK_2 = 0x20;
    /// Value written to REG_BANK_SEL to select User Bank 3.
    constexpr uint8_t BANK_3 = 0x30;

    // --- Bank 0 Registers ---
    /// Bank 0: Device identification register (Expected value: 0xEA).
    constexpr uint8_t REG_B0_WHO_AM_I = 0x00;
    /// Bank 0: User control register (I2C Master enable/reset, DMP control).
    constexpr uint8_t REG_B0_USER_CTRL = 0x03;
    /// Bank 0: Power Management 1 (Device reset, Sleep mode, Clock source selection).
    constexpr uint8_t REG_B0_PWR_MGMT_1 = 0x06;
    /// Bank 0: Power Management 2 (Individual Accel / Gyro axis disable).
    constexpr uint8_t REG_B0_PWR_MGMT_2 = 0x07;
    /// Bank 0: Interrupt pin and I2C bypass configuration (Bypass enables direct host access to aux bus).
    constexpr uint8_t REG_B0_INT_PIN_CFG = 0x0F;
    /// Bank 0: I2C Master status register.
    constexpr uint8_t REG_B0_I2C_MST_STATUS = 0x17;
    /// Bank 0: Accelerometer X-axis high byte.
    constexpr uint8_t REG_B0_ACCEL_XOUT_H = 0x2D;
    /// Bank 0: Accelerometer X-axis low byte.
    constexpr uint8_t REG_B0_ACCEL_XOUT_L = 0x2E;
    /// Bank 0: Accelerometer Y-axis high byte.
    constexpr uint8_t REG_B0_ACCEL_YOUT_H = 0x2F;
    /// Bank 0: Accelerometer Y-axis low byte.
    constexpr uint8_t REG_B0_ACCEL_YOUT_L = 0x30;
    /// Bank 0: Accelerometer Z-axis high byte.
    constexpr uint8_t REG_B0_ACCEL_ZOUT_H = 0x31;
    /// Bank 0: Accelerometer Z-axis low byte.
    constexpr uint8_t REG_B0_ACCEL_ZOUT_L = 0x32;
    /// Bank 0: Gyroscope X-axis high byte.
    constexpr uint8_t REG_B0_GYRO_XOUT_H = 0x33;
    /// Bank 0: Gyroscope X-axis low byte.
    constexpr uint8_t REG_B0_GYRO_XOUT_L = 0x34;
    /// Bank 0: Gyroscope Y-axis high byte.
    constexpr uint8_t REG_B0_GYRO_YOUT_H = 0x35;
    /// Bank 0: Gyroscope Y-axis low byte.
    constexpr uint8_t REG_B0_GYRO_YOUT_L = 0x36;
    /// Bank 0: Gyroscope Z-axis high byte.
    constexpr uint8_t REG_B0_GYRO_ZOUT_H = 0x37;
    /// Bank 0: Gyroscope Z-axis low byte.
    constexpr uint8_t REG_B0_GYRO_ZOUT_L = 0x38;
    /// Bank 0: Internal temperature sensor high byte.
    constexpr uint8_t REG_B0_TEMP_OUT_H = 0x39;
    /// Bank 0: Internal temperature sensor low byte.
    constexpr uint8_t REG_B0_TEMP_OUT_L = 0x3A;
    /// Bank 0: Starting register for 24-byte external sensor data buffer (0x3B - 0x52).
    constexpr uint8_t REG_B0_EXT_SLV_SENS_DATA = 0x3B;

    // --- Bank 2 Registers ---
    /// Bank 2: Gyroscope sample rate divider.
    constexpr uint8_t REG_B2_GYRO_SMPLRT_DIV = 0x00;
    /// Bank 2: Gyroscope configuration 1 (DLPF filter configuration & Full Scale Range).
    constexpr uint8_t REG_B2_GYRO_CONFIG_1 = 0x01;
    /// Bank 2: Accelerometer sample rate divider (MSB).
    constexpr uint8_t REG_B2_ACCEL_SMPLRT_DIV_1 = 0x10;
    /// Bank 2: Accelerometer sample rate divider (LSB).
    constexpr uint8_t REG_B2_ACCEL_SMPLRT_DIV_2 = 0x11;
    /// Bank 2: Accelerometer configuration (DLPF filter configuration & Full Scale Range).
    constexpr uint8_t REG_B2_ACCEL_CONFIG = 0x14;

    // --- Bank 3 Registers (I2C Master Control) ---
    /// Bank 3: I2C Master control (clock frequency, delay settings).
    constexpr uint8_t REG_B3_I2C_MST_CTRL = 0x01;
    /// Bank 3: I2C Slave 0 physical target address.
    constexpr uint8_t REG_B3_I2C_SLV0_ADDR = 0x03;
    /// Bank 3: I2C Slave 0 target starting register address.
    constexpr uint8_t REG_B3_I2C_SLV0_REG = 0x04;
    /// Bank 3: I2C Slave 0 control (Enable, byte count to read/write).
    constexpr uint8_t REG_B3_I2C_SLV0_CTRL = 0x05;
    /// Bank 3: I2C Slave 4 physical address (used for one-shot direct register I/O).
    constexpr uint8_t REG_B3_I2C_SLV4_ADDR = 0x13;
    /// Bank 3: I2C Slave 4 target register address.
    constexpr uint8_t REG_B3_I2C_SLV4_REG = 0x14;
    /// Bank 3: I2C Slave 4 control register.
    constexpr uint8_t REG_B3_I2C_SLV4_CTRL = 0x15;
    /// Bank 3: I2C Slave 4 data out register (write payload).
    constexpr uint8_t REG_B3_I2C_SLV4_DO = 0x16;
    /// Bank 3: I2C Slave 4 data in register (read payload).
    constexpr uint8_t REG_B3_I2C_SLV4_DI = 0x17;

    // --- ICM-20948 Constants & Bitfields ---
    /// Expected device ID returned by ICM-20948 WHO_AM_I register (0xEA).
    constexpr uint8_t ICM20948_WHO_AM_I_VALUE = 0xEA;
    /// Bit 7 of PWR_MGMT_1: Initiates software reset of all registers.
    constexpr uint8_t PWR_MGMT_1_RESET = 0x80;
    /// Bit 6 of PWR_MGMT_1: Puts device into low-power sleep mode.
    constexpr uint8_t PWR_MGMT_1_SLEEP = 0x40;
    /// Bits [2:0] of PWR_MGMT_1: Automatically select best available clock source (PLL).
    constexpr uint8_t PWR_MGMT_1_CLKSEL_AUTO = 0x01;
    /// Bit 5 of USER_CTRL: Enables auxiliary I2C master peripheral mode.
    constexpr uint8_t USER_CTRL_I2C_MST_EN = 0x20;
    /// Bit 1 of USER_CTRL: Resets auxiliary I2C master logic.
    constexpr uint8_t USER_CTRL_I2C_MST_RST = 0x02;
    /// Bit 1 of INT_PIN_CFG: Enables direct host-to-auxiliary I2C bypass multiplexer.
    constexpr uint8_t INT_PIN_CFG_BYPASS_EN = 0x02;

    // --- AK09916 Magnetometer Registers ---
    /// AK09916 Company / Device ID 2 register (Expected value: 0x09).
    constexpr uint8_t REG_AK09916_WIA2 = 0x01;
    /// AK09916 Status 1 register (Data Ready bit DRDY at bit 0).
    constexpr uint8_t REG_AK09916_ST1 = 0x10;
    /// AK09916 Measurement data X-axis low byte (Little-Endian start).
    constexpr uint8_t REG_AK09916_HXL = 0x11;
    /// AK09916 Status 2 register (Magnetic sensor overflow flag HOFL at bit 3).
    constexpr uint8_t REG_AK09916_ST2 = 0x18;
    /// AK09916 Control 2 register (Operating mode configuration).
    constexpr uint8_t REG_AK09916_CNTL2 = 0x31;
    /// AK09916 Control 3 register (Software reset trigger).
    constexpr uint8_t REG_AK09916_CNTL3 = 0x32;

    // --- AK09916 Mode Constants ---
    /// Expected device ID returned by AK09916 WIA2 register (0x09).
    constexpr uint8_t AK09916_WIA2_VALUE = 0x09;
    /// AK09916 Power-down mode (quiescent state).
    constexpr uint8_t AK09916_MODE_POWER_DOWN = 0x00;
    /// AK09916 Continuous measurement mode 1 (10 Hz).
    constexpr uint8_t AK09916_MODE_CONT_10HZ = 0x02;
    /// AK09916 Continuous measurement mode 2 (20 Hz).
    constexpr uint8_t AK09916_MODE_CONT_20HZ = 0x04;
    /// AK09916 Continuous measurement mode 3 (50 Hz).
    constexpr uint8_t AK09916_MODE_CONT_50HZ = 0x06;
    /// AK09916 Continuous measurement mode 4 (100 Hz).
    constexpr uint8_t AK09916_MODE_CONT_100HZ = 0x08;
    /// Bit 0 of CNTL3: Initiates AK09916 software reset.
    constexpr uint8_t AK09916_RESET = 0x01;

    // --- Standard Physical Conversion Constants ---
    /// Standard Earth gravitational acceleration in SI units ($m/s^2$) per REP-103.
    constexpr double GRAVITY_EARTH = 9.80665;
    /// Conversion constant from degrees to radians ($\pi / 180$).
    constexpr double DEG_TO_RAD = 0.017453292519943295;
    /// AK09916 magnetometer sensitivity scale factor: $0.15\,\mu\text{T} / \text{LSB} = 0.15 \times 10^{-6}\,\text{Tesla}$.
    constexpr double MAG_LSB_TO_TESLA = 0.15e-6;

} // namespace lekiwi_icm20948_hardware

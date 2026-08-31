# `lekiwi_icm20948_hardware`

`ros2_control` C++ sensor interface plugin (`SensorInterface`) interfacing the TDK InvenSense ICM-20948 9-DoF IMU (accelerometer, gyroscope, and magnetometer) over I2C on Raspberry Pi 5.

---

## ⚡ Features

- **I2C Communication**: Directly reads raw registers from `/dev/i2c-1` at address `0x68`.
- **ros2_control State Interfaces (13 States)**:
  - `orientation.x`, `orientation.y`, `orientation.z`, `orientation.w`
  - `angular_velocity.x`, `angular_velocity.y`, `angular_velocity.z`
  - `linear_acceleration.x`, `linear_acceleration.y`, `linear_acceleration.z`
  - `magnetic_field.x`, `magnetic_field.y`, `magnetic_field.z`
- **Automatic Gyroscope Calibration**: Computes zero-rate offsets across a configurable sample window at startup (`imu_gyro_calib_samples: 500`).
- **Async Execution**: Supports asynchronous non-blocking sensor updates.

---

## ⚙️ Parameters (URDF Xacro)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mock_sensor` | `bool` | `false` | When `true`, produces synthetic sensor readings without I2C hardware. |
| `i2c_bus` | `int` | `1` | Linux I2C adapter number (`/dev/i2c-1`). |
| `i2c_address` | `string` | `"0x68"` | ICM-20948 7-bit I2C slave address. |
| `auto_calibrate_gyro` | `bool` | `true` | Calibrate gyroscope bias during `on_activate`. |
| `gyro_calib_samples` | `int` | `500` | Number of samples collected during calibration. |

---

## 🧪 Testing

```bash
colcon test --packages-select lekiwi_icm20948_hardware --event-handlers console_direct+
colcon test-result --verbose
```


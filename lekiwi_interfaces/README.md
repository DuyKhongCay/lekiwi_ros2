# `lekiwi_interfaces`

ROS 2 message (`.msg`) and service (`.srv`) definitions shared across the LeKiwi robot software stack.

---

## 📋 Message Definitions (`msg/`)

### 1. `CameraMode.msg`
Defines the four discrete operational modes of the robot perception and control finite-state machine:

| Constant | Value | Description |
|---|---|---|
| `STANDBY` | `0` | Default idle state; all camera streams and Hailo inference pipelines are gated off. |
| `NAVIGATING` | `1` | Mobile base navigation mode; active cameras: `stereo_left`. |
| `CHESS_THINKING` | `2` | Chess perception mode; Hailo-8 NPU inference active on `stereo_left`. |
| `MANIPULATION_LEROBOT` | `3` | Robotic arm manipulation mode; active cameras: `stereo_right`, `usb_wrist`, `usb_side`. |

- **Field**: `uint8 value`

---

### 2. `DriveStatus.msg`
Health, error counters, and communication status of the mobile base and motor bus:

- **Fields**:
  - `std_msgs/Header header`
  - `uint8 state` (`UNKNOWN=0`, `INACTIVE=1`, `ACTIVE=2`, `ERROR=3`)
  - `bool bus_connected`
  - `bool drive_enabled`
  - `bool watchdog_expired`
  - `uint32 serial_error_count`
  - `uint32 consecutive_error_count`
  - `uint32 retry_count`
  - `float32 command_age_sec`
  - `string last_error`

---

### 3. `ServoTelemetry.msg`
High-frequency diagnostic telemetry for the 3 omnidirectional base wheel servos (Left, Back, Right):

- **Fields**:
  - `std_msgs/Header header`
  - `uint8[3] id`
  - `int32[3] raw_position`
  - `int32[3] raw_velocity`
  - `int32[3] raw_load`
  - `float64[3] position_rad`
  - `float64[3] velocity_rad_s`
  - `float32[3] current_ma`
  - `float32[3] voltage_v`
  - `uint8[3] temperature_c`
  - `uint8[3] status`
  - `bool[3] moving`
  - `uint8[3] servo_error`
  - `bool[3] online`

---

## 🛠️ Service Definitions (`srv/`)

### 1. `SetCamMode.srv`
Request a mode transition in the task orchestrator or camera streamer components:
- **Request**: `CameraMode requested_mode`
- **Response**: `bool success`, `CameraMode applied_mode`, `string message`

### 2. `ResetMotorBus.srv`
Restart the serial communication bus and re-enable motor torque without restarting the ROS stack:
- **Request**: (empty)
- **Response**: `bool success`, `string message`

### 3. `SetTorqueEnabled.srv`
Enable or disable motor torque for the entire robot, arm only, or base only:
- **Constants**:
  - `uint8 TARGET_ALL = 0`
  - `uint8 TARGET_ARM = 1`
  - `uint8 TARGET_BASE = 2`
- **Request**:
  - `uint8 target`
  - `bool enabled`
- **Response**:
  - `bool success`
  - `string message`

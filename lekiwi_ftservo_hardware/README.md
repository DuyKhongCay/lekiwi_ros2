# `lekiwi_ftservo_hardware`

`ros2_control` C++ hardware interface plugin (`SystemInterface`) providing real-time communication with 9 Feetech STS serial bus servos over a single UART bus (`/dev/lekiwi_serial`) at 1,000,000 baud.

---

## ⚡ Key Features

- **Asynchronous I/O Worker Thread**:
  - `read()` and `write()` calls from the 50 Hz `ros2_control` controller loop interact only with thread-safe shared double-buffers (`SharedState`, `SharedCommand`) in `< 10 µs`.
  - A background worker thread (`io_worker_thread_`) continuously executes packet serialization, sync read, and write commands over the 1 Mbps serial line.
- **Hybrid Joint Control**:
  - **6 Robotic Arm Joints** (`STS3215` in position mode): `arm_shoulder_pan`, `arm_shoulder_lift`, `arm_elbow_flex`, `arm_wrist_flex`, `arm_wrist_roll`, `arm_gripper`.
  - **3 Omnidirectional Base Wheels** (`STS3215` in continuous velocity mode): `base_left_wheel`, `base_back_wheel`, `base_right_wheel`.
- **Diagnostic Telemetry & Safety Watchdogs**:
  - Real-time voltage, current, temperature, moving state, and load published via `diagnostic_updater`.
  - Watchdog checks automatically disable motor torque if serial communication fails.

---

## 🔌 Hardware Configuration

Joint limits, velocity scaling, and homing offsets are configured in [`lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml):

```yaml
joints:
  arm_shoulder_pan:
    id: 1
    drive_mode: 0 # 0: Position mode
    homing_offset: -316
    range_min: 844
    range_max: 3345
  base_left_wheel:
    id: 7
    drive_mode: 1 # 1: Velocity mode
    velocity_radians_per_second_per_tick: 0.07665486
    max_velocity_radians_per_second: 2.0
```

---

## 🧪 Testing

Run GoogleTest unit tests for STS packet serialization and velocity codec math:

```bash
colcon test --packages-select lekiwi_ftservo_hardware --event-handlers console_direct+
colcon test-result --verbose
```


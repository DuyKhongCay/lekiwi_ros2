# External Integrations

## Core Sections (Required)

### 1) Integration Inventory

| System | Type (API/DB/Queue/etc) | Purpose | Auth model | Criticality | Evidence |
|--------|---------------------------|---------|------------|-------------|----------|
| **Hailo-8 / Hailo-8L NPU** | Hardware Accelerator (PCIe / M.2) | Real-time neural network inference (YOLO chess detection / segmentation) via HailoRT & TAPPAS | System driver / PCIe access | High | [`lekiwi_perception/CMakeLists.txt:33-36`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L33-L36), [`lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp) |
| **Feetech STS Servos Bus** | Hardware Serial Bus (UART) | Real-time actuation of 6 arm joints (STS3215) and 3 omni wheels (STS3215 in velocity mode) at 1 Mbps | `/dev/lekiwi_serial` via udev rule | High | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp), [`lekiwi_bringup/udev/99-lekiwi.rules`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules) |
| **Raspberry Pi CSI Cameras (IMX219)** | Hardware CSI Camera (RP1 PCIe) | Dual stereo camera capture via GStreamer `libcamerasrc` | Linux kernel device driver | High | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:16,33`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L16) |
| **USB V4L2 Cameras** | Hardware USB Cameras | Wrist and side perspective camera capture via GStreamer `v4l2src` | `/dev/usb_wrist`, `/dev/usb_side` via udev | Medium | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:50,68`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L50) |
| **ICM-20948 9-DoF IMU** | Hardware Sensor (I2C) | 9-DoF orientation, linear acceleration, and magnetometer feedback | `/dev/i2c-1` @ address `0x68` | Medium | [`lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp) |
| **LeRobot Bridge** | Robotics Policy Middleware (ROS 2 Action/Topic) | Transmitting arm actions and joint observations to LeRobot imitation learning policies | ROS 2 Topics & Actions | Medium | [`lekiwi_control/lekiwi_control/lerobot_arm_bridge.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/lerobot_arm_bridge.py) |
| **Gamepad Teleop** | USB / Bluetooth Input Device | Manual mobile base and arm velocity teleoperation | `/dev/gamepad` via udev | Low | [`lekiwi_bringup/launch/teleop.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/teleop.launch.py) |

---

### 2) Data Stores & Calibration Assets

| Store | Role | Access layer | Key risk | Evidence |
|-------|------|--------------|----------|----------|
| **Camera Calibration Files** (`*.yaml`) | Intrinsic and distortion matrices for left/right stereo and USB cameras | Loaded via `camera_info_manager` | Mismatched calibration causing projection errors in chess square mapping | [`lekiwi_bringup/config/perception/calibration/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/calibration) |
| **Joint Calibration File** (`lekiwi_joints.yaml`) | Joint limits, homing offsets, PID coefficients, and velocity scale factors | Loaded at launch by `lekiwi_ftservo_hardware` and `lerobot_arm_bridge` | Incorrect homing offsets leading to joint collision | [`lekiwi_bringup/config/hardware/lekiwi_joints.yaml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/hardware/lekiwi_joints.yaml) |
| **HEF Neural Models** (`*.hef`) | Compiled Hailo neural network binaries for YOLO piece detection and segmentation | Loaded by HailoRT runtime in `lekiwi_perception` | Model mismatch with target TAPPAS/HailoRT driver version | [`lekiwi_perception/resources/models/yolo11n.hef`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/resources/models/yolo11n.hef) |
| **Piece Sprite Assets** (`*.png`) | 2D transparent piece sprites for GUI overlay generation | Loaded by OpenCV in `ChessVisualizerComponent` | Missing sprite path causing visualization failure | [`lekiwi_perception/resources/pieces/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/resources/pieces) |

---

### 3) Secrets and Credentials Handling

- **Credential sources**: No external cloud tokens, API keys, or database credentials are used. All hardware access is local via Linux device paths (`/dev/*`).
- **Hardcoding checks**: Device identifiers are bound via standard Linux udev rules ([`lekiwi_bringup/udev/99-lekiwi.rules`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules)) and parameterized via launch files.
- **Rotation / Lifecycle**: Not applicable (local hardware bus).

---

### 4) Reliability and Failure Behavior

- **Serial Bus Watchdog & Error Recovery**:
  - `LeKiwiFeetechHardwareInterface` tracks consecutive read/write failures.
  - If serial communication drops, motor torque is safely disabled and watchdog flags are raised on `/lekiwi_base/drive_status`.
  - Dedicated service `/lekiwi_base/reset_motor_bus` allows restarting the serial bus without restarting the full ROS stack ([`lekiwi_interfaces/srv/ResetMotorBus.srv`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_interfaces/srv/ResetMotorBus.srv)).
- **GStreamer Pipeline Error Recovery**:
  - `CameraStreamerComponent` monitors the GStreamer bus on a periodic timer (`poll_bus_errors()`), capturing pipeline error messages and resetting GStreamer state when camera hardware disconnects.
- **Hailo NPU Lifecycle Recovery**:
  - If inference fails, `HailoChessInferenceComponent` publishes `PIPELINE_ERROR` state and transitions into an error state handled by `TaskOrchestratorNode`.

---

### 5) Observability for Integrations

- **Diagnostic Updater (`diagnostic_updater`)**:
  - Periodically publishes hardware and pipeline diagnostics to `/diagnostics`.
  - Monitored metrics include: servo voltage, temperature, current, GStreamer FPS, latency, and bus error counters.
- **Status Topics**:
  - `/hailo_chess_inference/status`: Pipeline state (`PIPELINE_RUNNING`, `PIPELINE_ERROR`), current FPS, last error.
  - `/lekiwi_base/drive_status`: Bus connection state, watchdog status, error counts.
  - `/lekiwi_base/servo_telemetry`: Per-joint voltage, temperature, load, and position feedback.

---

### 6) Evidence

- [`lekiwi_bringup/udev/99-lekiwi.rules`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules)
- [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)
- [`lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp)
- [`lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp)
- [`lekiwi_bringup/config/perception/gscam_cameras.yaml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml)
- [`lekiwi_interfaces/msg/ServoTelemetry.msg`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_interfaces/msg/ServoTelemetry.msg)


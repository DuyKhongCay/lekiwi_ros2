# External Integrations

## Core Sections (Required)

### 1) Integration Inventory

| System | Type (API/DB/Queue/etc) | Purpose | Auth model | Criticality | Evidence |
|--------|---------------------------|---------|------------|-------------|----------|
| **Hailo-8 / Hailo-8L NPU** | Hardware Accelerator (PCIe / M.2) | Real-time neural network inference (YOLO chess detection / segmentation) via HailoRT & TAPPAS | System driver / PCIe access | High | [`lekiwi_perception/CMakeLists.txt:33-36`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L33-L36), [`lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp) |
| **AprilTag Fiducial Markers** | Vision Target (36h11 family) | Chessboard arena boundary detection and 6-DoF spatial pose estimation | Vision marker geometry | High | [`lekiwi_tag_localization/config/chessboard_tags.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/config/chessboard_tags.yaml), [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp) |
| **Feetech STS Servos Bus** | Hardware Serial Bus (UART) | Real-time actuation of 6 follower arm joints (STS3215) and 3 omni wheels (STS3215 in velocity mode) at 1 Mbps | `/dev/lekiwi_serial` via udev rule | High | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp), [`lekiwi_bringup/udev/99-lekiwi.rules`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules) |
| **uArm / Zhongli Leader Arm** | Hardware Serial Bus (UART) | Low-latency unpowered leader arm teleoperation (reading servo feedback angles at 50 Hz) | `/dev/uarm_leader` via udev rule | Medium | [`teleop_zhongli_servo_hw/src/zhongli_protocol.cpp`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/zhongli_protocol.cpp), [`lekiwi_bringup/udev/99-teleop-lekiwi.rules`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-teleop-lekiwi.rules) |
| **Raspberry Pi CSI Cameras (IMX219)** | Hardware CSI Camera (RP1 PCIe) | Dual stereo camera capture via GStreamer `libcamerasrc` with PiSP tuning config | Linux kernel device driver | High | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:16,33`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L16), [`lekiwi_bringup/config/perception/pisp_tuning/imx219_noir.json`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/pisp_tuning/imx219_noir.json) |
| **USB V4L2 Cameras** | Hardware USB Cameras | Wrist and side perspective camera capture via GStreamer `v4l2src` | `/dev/usb_wrist`, `/dev/usb_side` via udev | Medium | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:50,68`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L50) |
| **ICM-20948 9-DoF IMU** | Hardware Sensor (I2C) | 9-DoF orientation, linear acceleration, and magnetometer feedback | `/dev/i2c-1` @ address `0x68` | Medium | [`lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp) |
| **LeRobot Bridge** | Robotics Policy Middleware (ROS 2 Action/Topic) | Transmitting arm actions and joint observations to LeRobot imitation learning policies | ROS 2 Topics & Actions | Medium | [`lekiwi_control/lekiwi_control/lerobot_arm_bridge.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/lerobot_arm_bridge.py) |
| **Gamepad Teleop** | USB / Bluetooth Input Device | Manual mobile base velocity teleoperation via `joy_linux` | `/dev/gamepad` via udev | Low | [`lekiwi_bringup/launch/teleop_gamepad.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/teleop_gamepad.launch.py) |

---

### 2) Data Stores & Calibration Assets

| Store | Role | Access layer | Key risk | Evidence |
|-------|------|--------------|----------|----------|
| **AprilTag & Chessboard Geometry Config** (`chessboard_tags.yaml`) | Tag layout, IDs, dimensions, and board boundary physical parameters | Loaded by `chessboard_pose_estimator_node` | Incorrect tag coordinates causing orientation and position errors | [`lekiwi_tag_localization/config/chessboard_tags.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/config/chessboard_tags.yaml) |
| **PiSP Tuning JSON** (`imx219_noir.json`) | ISP pipeline sensor tuning matrix for IMX219 NoIR CSI cameras on Raspberry Pi 5 | Exported via `LIBCAMERA_RPI_TUNING_FILE` environment variable | Poor image exposure/white balance degrading detection accuracy | [`lekiwi_bringup/config/perception/pisp_tuning/imx219_noir.json`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/pisp_tuning/imx219_noir.json) |
| **Arena Static Map** (`chessboard_arena.yaml`, `.pgm`) | 2.0m x 2.0m occupancy grid with 0.45m x 0.45m forbidden zone | Loaded by `nav2_map_server` | Incorrect resolution or origin causing robot localization drift | [`lekiwi_navigation/maps/chessboard_arena.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/maps/chessboard_arena.yaml) |
| **uArm Teleop Calibration File** (`uarm_teleop_calib.yaml`) | Leader-follower joint mapping, midpoint positions, direction inversion, and scaling | Loaded by `teleop_uarm_node` and generated by `calibrate_uarm_node` | Incorrect midpoint or direction causing erratic follower motion | [`lekiwi_bringup/config/servos/uarm_teleop_calib.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/servos/uarm_teleop_calib.yaml) |
| **Joint Calibration File** (`lekiwi_arm_calib.yaml`) | Robot follower arm joint limits, homing offsets, PID coefficients, and velocity scale factors | Loaded at launch by `lekiwi_ftservo_hardware` and `lerobot_arm_bridge` | Incorrect homing offsets leading to joint collision | [`lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml) |
| **Camera Calibration Files** (`*.yaml`) | Intrinsic and distortion matrices for left/right stereo and USB cameras | Loaded via `camera_info_manager` | Mismatched calibration causing projection errors in chess square mapping | [`lekiwi_bringup/config/perception/camera_info/`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/camera_info) |
| **HEF Neural Models** (`*.hef`) | Compiled Hailo neural network binaries for YOLO piece detection and segmentation | Loaded by HailoRT runtime in `lekiwi_perception` | Model mismatch with target TAPPAS/HailoRT driver version | [`lekiwi_perception/resources/models/yolo11n.hef`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/resources/models/yolo11n.hef) |

---

### 3) Secrets and Credentials Handling

- **Credential sources**: No external cloud tokens, API keys, or database credentials are used. All hardware access is local via Linux device paths (`/dev/*`).
- **Hardcoding checks**: Device identifiers are bound via standard Linux udev rules ([`lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules)) and parameterized via launch files.
- **Rotation / Lifecycle**: Not applicable (local hardware bus).

---

### 4) Reliability and Failure Behavior

- **Command Velocity Arbitration & Emergency Stop (`twist_mux`)**:
  - `twist_mux` automatically locks `/cmd_vel` output when `/safety/estop_active` is asserted (Priority 255).
  - Manual gamepad teleoperation (`/cmd_vel_teleop`, Priority 100) instantly overrides autonomous Nav2 path tracking (`/cmd_vel_nav`, Priority 10).
- **Serial Bus Watchdog & Error Recovery**:
  - `LeKiwiFeetechHardwareInterface` tracks consecutive read/write failures.
  - If serial communication drops, motor torque is safely disabled and watchdog flags are raised on `/lekiwi_base/drive_status`.
  - Dedicated service `/lekiwi_base/reset_motor_bus` allows restarting the serial bus without restarting the full ROS stack ([`lekiwi_interfaces/srv/ResetMotorBus.srv`](file:///root/docker_ws/lekiwi_ros2/lekiwi_interfaces/srv/ResetMotorBus.srv)).
- **uArm Leader Teleoperation Resilience**:
  - `teleop_uarm_node` reconnects automatically upon serial disconnection and flags connection state to diagnostics.
- **GStreamer Pipeline Error Recovery**:
  - `CameraStreamerComponent` monitors the GStreamer bus on a periodic timer (`poll_bus_errors()`), capturing pipeline error messages and resetting GStreamer state when camera hardware disconnects.
- **Hailo NPU Lifecycle Recovery**:
  - If inference fails, `HailoChessInferenceComponent` publishes `PIPELINE_ERROR` state and transitions into an error state handled by `TaskOrchestratorNode`.

---

### 5) Observability for Integrations

- **Diagnostic Updater & Aggregator (`diagnostic_updater` / `diagnostic_aggregator`)**:
  - Periodically publishes hardware, teleop, system resources, and pipeline diagnostics to `/diagnostics` and `/diagnostics_agg`.
  - Monitored metrics include: servo voltage, temperature, current, uArm leader connection & frequency, CPU/RAM/Disk loads, GStreamer FPS, latency, and bus error counters.
- **Status Topics**:
  - `/hailo_chess_inference/status`: Pipeline state (`PIPELINE_RUNNING`, `PIPELINE_ERROR`), current FPS, last error.
  - `/lekiwi_base/drive_status`: Mobile base motor bus state, drive enable status, and watchdog fault counters.
  - `/safety/estop_active`: Software emergency stop status.

---

### 6) Evidence

- [`lekiwi_tag_localization/config/chessboard_tags.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/config/chessboard_tags.yaml)
- [`lekiwi_navigation/config/twist_mux.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/config/twist_mux.yaml)
- [`lekiwi_bringup/config/perception/pisp_tuning/imx219_noir.json`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/pisp_tuning/imx219_noir.json)
- [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)
- [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp)
- [`lekiwi_bringup/launch/robot.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)

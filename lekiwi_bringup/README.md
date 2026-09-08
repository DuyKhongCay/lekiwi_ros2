# `lekiwi_bringup`

Central orchestration package for the LeKiwi robot, containing subsystem and full system launch files, runtime parameter YAMLs, controller configurations, calibration matrices, and Linux udev rules.

---

## 🚀 Launch Architecture

```text
robot.launch.py (Top-level System Entrypoint)
├── description.launch.py    -> robot_state_publisher + ros2_control controller manager + spawners
├── cameras.launch.py        -> lekiwi_perception_container (4x CameraStreamer + HailoChessInference)
├── control.launch.py        -> task_orchestrator + LeRobotArmBridge + camera_mode_demo
├── imu.launch.py            -> ICM-20948 broadcaster + Madgwick filter + TF2 transformer
├── teleop_gamepad.launch.py -> joy_linux_node + joy_teleop (Base joystick teleop)
├── teleop_uarm.launch.py    -> teleop_uarm_node (uArm leader arm teleop)
└── diagnostics.launch.py    -> diagnostic_aggregator (/diagnostics_agg) + system resource monitors (CPU, RAM, Disk)
```

---

## 🛠️ Launch Commands

### 1. Full Robot Stack & Master Switches

`robot.launch.py` cung cấp các biến Master Switch quan trọng nhất để bật/tắt toàn bộ hoặc từng phần của robot:

| Master Variable | Mặc Định | Chức Năng |
|---|---|---|
| `enable_description` | `true` | Bật/tắt URDF, robot_state_publisher và ros2_control manager |
| `enable_perception` | `false` | Bật/tắt cụm 4 Camera GStreamer + Hailo-8 NPU perception |
| `enable_control` | `true` | Bật/tắt tầng điều khiển cấp cao (`lekiwi_control`) |
| `enable_imu_pipeline` | `true` | Bật/tắt lọc Madgwick IMU và biến đổi TF2 base_footprint |
| `start_gamepad_teleop` | `false` | Bật/tắt điều khiển mobile base bằng tay cầm joystick |
| `start_uarm_teleop` | `false` | Bật/tắt điều khiển cánh tay bằng uArm leader |
| `enable_diagnostics` | `true` | Bật/tắt cây chẩn đoán diagnostic_aggregator & monitors |

```bash
# Chạy mặc định (Mock mode an toàn cho dev, diagnostics & imu bật)
ros2 launch lekiwi_bringup robot.launch.py

# Chạy trên robot thật với toàn bộ controllers & teleop:
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=real \
  imu_hardware_type:=real \
  start_controller_manager:=true \
  activate_controllers:=true \
  start_gamepad_teleop:=true \
  start_uarm_teleop:=true

# Kích hoạt riêng arm_controller (dành cho teleop hoặc điều khiển cánh tay độc lập):
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=real \
  start_controller_manager:=true \
  start_arm_controller:=true \
  start_uarm_teleop:=true

# Chạy cùng LeRobot ML Bridge:
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=real \
  start_controller_manager:=true \
  start_arm_controller:=true \
  start_lerobot_bridge:=true
```

### 2. Isolated Subsystem Debugging
```bash
# Description and controllers only:
ros2 launch lekiwi_description description.launch.py hardware_type:=mock
```

# Controllers only:
ros2 launch lekiwi_bringup controllers.launch.py hardware_type:=mock

# Composed perception container only:
ros2 launch lekiwi_bringup cameras.launch.py use_test_sources:=true

# Control orchestrator and LeRobot bridge only:
ros2 launch lekiwi_bringup control.launch.py start_lerobot_bridge:=true

# IMU filtering pipeline:
ros2 launch lekiwi_bringup imu.launch.py

# Standalone Gamepad teleoperation:
ros2 launch lekiwi_bringup teleop_gamepad.launch.py

# Standalone uArm leader arm teleoperation:
ros2 launch lekiwi_bringup teleop_uarm.launch.py arm_mode:=joint_trajectory

# Diagnostics aggregator and system monitors only:
ros2 launch lekiwi_bringup diagnostics.launch.py

# View diagnostics tree in GUI:
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

## ⚙️ Configuration Directories (`config/`)

- `config/control/`: High-level FSM orchestrator, gamepad teleop, and uArm parameters.
- `config/controllers/`: `ros2_control` controller configurations (`arm_controller`, `omni_base_controller`, `lekiwi_imu_broadcaster`, `joint_state_broadcaster`).
- `config/diagnostics/`: Diagnostic analyzer grouping hierarchy for `diagnostic_aggregator`.
- `config/localization/`: EKF robot localization (`ekf.yaml`) and AprilTag chessboard layout parameters (`chessboard_tags.yaml`, `chessboard_tags_calib.yaml`).
- `config/navigation/`: Nav2 stack parameter configuration (`nav2_params.yaml`).
- `config/perception/`: GStreamer pipelines (`gscam_cameras.yaml`) and intrinsic camera calibration matrices.
- `config/sensors/`: IMU Madgwick orientation estimation parameters (`imu_filter.yaml`).

---

## 🔌 Udev Rules

Install udev rules to create persistent symlinks for hardware buses:

```bash
sudo cp udev/99-lekiwi.rules /etc/udev/rules.d/
sudo cp udev/99-teleop-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

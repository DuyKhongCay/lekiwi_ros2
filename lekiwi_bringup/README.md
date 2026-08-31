# `lekiwi_bringup`

Central orchestration package for the LeKiwi robot, containing subsystem and full system launch files, runtime parameter YAMLs, controller configurations, calibration matrices, and Linux udev rules.

---

## 🚀 Launch Architecture

```text
robot.launch.py (Top-level System Entrypoint)
├── description.launch.py  -> robot_state_publisher + ros2_control controller manager
├── cameras.launch.py      -> lekiwi_perception_container (4x CameraStreamer + HailoChessInference)
├── control.launch.py      -> task_orchestrator + LeRobotArmBridge
├── imu.launch.py          -> ICM-20948 broadcaster + Madgwick filter + TF2 transformer
└── teleop.launch.py       -> joy_node + teleop_twist_joy (Optional Gamepad control)
```

---

## 🛠️ Launch Commands

### 1. Full Robot Stack
```bash
# Mock mode (Default, safe for development without hardware)
ros2 launch lekiwi_bringup robot.launch.py

# Real hardware execution (Raspberry Pi 5 + Feetech Servos + Hailo NPU)
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=real \
  imu_hardware_type:=real \
  start_controller_manager:=true \
  activate_controllers:=true
```

### 2. Isolated Subsystem Debugging
```bash
# Description and controllers only:
ros2 launch lekiwi_bringup description.launch.py hardware_type:=mock start_controller_manager:=true activate_controllers:=true

# Composed perception container only:
ros2 launch lekiwi_bringup cameras.launch.py use_test_sources:=true

# Control orchestrator and LeRobot bridge only:
ros2 launch lekiwi_bringup control.launch.py start_lerobot_bridge:=true

# IMU filtering pipeline:
ros2 launch lekiwi_bringup imu.launch.py

# Gamepad teleoperation:
ros2 launch lekiwi_bringup teleop.launch.py start_teleop:=true
```

---

## ⚙️ Configuration Directories (`config/`)

- `config/controllers/lekiwi_controllers.yaml`: `ros2_control` controller configurations (`arm_controller`, `omni_base_controller`, `lekiwi_imu_broadcaster`, `joint_state_broadcaster`).
- `config/hardware/lekiwi_joints.yaml`: Servo IDs, homing offsets, limits, and velocity scale factors.
- `config/perception/cameras.yaml`: Parameters for `HailoChessInferenceComponent`.
- `config/perception/gscam_cameras.yaml`: GStreamer pipeline declarations for all 4 camera endpoints with valve gating.
- `config/perception/calibration/`: Intrinsic camera calibration matrices for stereo and USB cameras.
- `config/sensors/imu_filter.yaml`: IMU Madgwick orientation estimation and covariance tuning.
- `config/control/teleop_gamepad.yaml`: Button and axis mappings for joystick teleoperation.

---

## 🔌 Udev Rules

Install udev rules to create persistent symlinks for hardware buses:

```bash
sudo cp udev/99-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

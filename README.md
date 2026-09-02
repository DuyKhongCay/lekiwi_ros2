# LeKiwi ROS 2 Workspace

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-22314E.svg?logo=ros)](https://docs.ros.org/)
[![C++](https://img.shields.io/badge/C++-17-00599C.svg?logo=c%2B%2B)](https://en.cppreference.com/)
[![Python](https://img.shields.io/badge/Python-3.10%2B%20%7C%203.12-3776AB.svg?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Production ROS 2 workspace for the **LeKiwi Robot**, featuring an omnidirectional mobile base, 6-DoF robotic arm, dual CSI stereo perception with Hailo-8/8L NPU hardware acceleration, LeRobot imitation learning integration, and `ros2_control` hardware abstraction.

---

## 🏛️ System Architecture Overview

```text
                                 +-----------------------------------+
                                 |       Task Orchestrator FSM       |
                                 |     (lekiwi_control / rclpy)      |
                                 +-----------------+-----------------+
                                                   | /camera_mode
                 +---------------------------------+---------------------------------+
                 |                                                                   |
                 v                                                                   v
+---------------------------------+                               +-----------------------------------+
|    lekiwi_perception Container  |                               |    ros2_control Controller Mgr    |
|   (GStreamer + HailoRT + OCV)   |                               |  (lekiwi_bringup / ros2_control)  |
+---------------------------------+                               +-----------------+-----------------+
| • CameraStreamerComponent (x4)  |                                                 |
|   (Dynamic GStreamer Valve Gate)|                        +------------------------+-----------------------+
| • HailoChessInferenceComponent  |                        |                                                |
|   (Hailo-8/8L NPU YOLO & FEN)   |                        v                                                v
| • ChessVisualizerComponent      |        +-------------------------------+                +-------------------------------+
+---------------------------------+        |   LeKiwiFeetechHardware       |                |      ICM20948SensorHardware   |
                                           |  (Async I/O Worker Thread)    |                |   (I2C 9-DoF IMU Broadcaster) |
                                           +---------------+---------------+                +---------------+---------------+
                                                           | /dev/lekiwi_serial                             | /dev/i2c-1
                                                           v                                                v
                                           +-------------------------------+                +-------------------------------+
                                           | 9x Feetech STS3215 Servos     |                | ICM-20948 9-DoF IMU Sensor    |
                                           | (6-DoF Arm + 3-Wheel Omni)    |                +-------------------------------+
                                           +-------------------------------+
```

---

## 📦 Packages Summary

| Package | Language / Type | Description |
|:---|:---:|:---|
| [`lekiwi_interfaces`](lekiwi_interfaces/) | ROS 2 Interfaces | Custom messages (`CameraMode`, `HailoInferenceStatus`, `DriveStatus`, `ServoTelemetry`) and services (`SetCamMode`, `ResetMotorBus`, `SetDriveEnabled`). |
| [`lekiwi_perception`](lekiwi_perception/) | C++ Components | Lifecycle GStreamer camera streamer components, HailoRT NPU YOLO inference component, chessboard detection, FEN generator, and visualizer. |
| [`lekiwi_control`](lekiwi_control/) | Python / `rclpy` | Four-mode camera/task finite-state machine (FSM), lifecycle boot orchestrator, and LeRobot arm trajectory bridge. |
| [`lekiwi_ftservo_hardware`](lekiwi_ftservo_hardware/) | C++ `ros2_control` | `SystemInterface` hardware plugin for 9 Feetech STS servos on a 1 Mbps serial bus with asynchronous I/O worker thread. |
| [`lekiwi_icm20948_hardware`](lekiwi_icm20948_hardware/) | C++ `ros2_control` | `SensorInterface` hardware plugin for ICM-20948 9-DoF IMU communicating over I2C (`/dev/i2c-1`). |
| [`lekiwi_description`](lekiwi_description/) | URDF / Xacro | Kinematic robot description, CAD STL meshes, joint limits, transmissions, and `ros2_control` macros. |
| [`lekiwi_bringup`](lekiwi_bringup/) | Launch & Config | System launch compositions (`robot.launch.py`), YAML configs, controllers, calibration files, and udev rules. |
| [`lekiwi_navigation`](lekiwi_navigation/) | Roadmap / Nav2 | Nav2 integration package for autonomous omni-wheel navigation (in development). |
| [`teleop_zhongli_servo_hw`](teleop_zhongli_servo_hw/) | C++ Driver / Remapper | Dedicated driver, interactive calibration CLI, and ROS 2 publisher for Zhongli / uArm leader teleoperation. |
| [`scripts`](scripts/) | Python Utilities | Developer diagnostics and image capture tools (`analyzing_traces.py`, `save_image.py`). |
| [`deprecated`](deprecated/) | Archived | Legacy nodes and backups (`hailo_perception_node`, `lekiwi_hardwares`), ignored via `COLCON_IGNORE`. |

---

## 🚀 Quick Start

### 1. Build the Workspace

```bash
# Clone and build with symlink install
colcon build --symlink-install

# Or build specific packages
colcon build --packages-select lekiwi_interfaces lekiwi_perception lekiwi_control lekiwi_ftservo_hardware

# Source workspace setup
source install/setup.bash
```

### 2. Run Tests

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### 3. Launching the Robot

```bash
# Launch entire robot stack in default mock mode (safe, no serial bus required)
ros2 launch lekiwi_bringup robot.launch.py

# Launch on real robot hardware (Raspberry Pi 5 + Hailo-8 + Feetech Servos)
ros2 launch lekiwi_bringup robot.launch.py hardware_type:=real imu_hardware_type:=real start_controller_manager:=true activate_controllers:=true

# Run individual subsystems for isolated debugging:
ros2 launch lekiwi_bringup description.launch.py hardware_type:=mock
ros2 launch lekiwi_bringup cameras.launch.py use_test_sources:=true
ros2 launch lekiwi_bringup control.launch.py start_lerobot_bridge:=true
ros2 launch lekiwi_bringup imu.launch.py
ros2 launch lekiwi_bringup teleop.launch.py start_teleop:=true
```

---

## 🔌 Hardware Setup & Device Rules

Install the udev rules to symlink serial devices and configure proper access permissions:

```bash
sudo cp lekiwi_bringup/udev/99-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Device symlinks configured:
- `/dev/lekiwi_serial`: Feetech STS motor bus (1,000,000 baud).
- `/dev/i2c-1`: Raspberry Pi 5 I2C bus for ICM-20948 IMU (`0x68`).
- `/dev/usb_wrist`, `/dev/usb_side`: USB camera nodes.
- `/dev/gamepad`: Teleoperation joystick.

---

## 📚 Standard Codebase Documentation

For in-depth architectural specifications and coding standards, refer to the [Standard Codebase Docs](docs/codebase/):

- [`STACK.md`](docs/codebase/STACK.md) — Tech stack, compilers, and production dependencies.
- [`STRUCTURE.md`](docs/codebase/STRUCTURE.md) — Directory layout, entry points, and module boundaries.
- [`ARCHITECTURE.md`](docs/codebase/ARCHITECTURE.md) — Design patterns, component composition, and data flow.
- [`CONVENTIONS.md`](docs/codebase/CONVENTIONS.md) — Coding styles, naming rules, and abbreviation standards.
- [`INTEGRATIONS.md`](docs/codebase/INTEGRATIONS.md) — Hardware drivers, sensors, and external systems.
- [`TESTING.md`](docs/codebase/TESTING.md) — Test suites, GTest/Pytest configurations, and mock strategies.
- [`CONCERNS.md`](docs/codebase/CONCERNS.md) — Technical debt, known risks, and performance notes.
# Technology Stack

## Core Sections (Required)

### 1) Runtime Summary

| Area | Value | Evidence |
|------|-------|----------|
| Primary languages | C++ (C++17 & C++20 standard) & Python (3.10+ / 3.12) | [`lekiwi_perception/CMakeLists.txt:5-7`]($HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L5-L7), [`teleop_zhongli_servo_hw/CMakeLists.txt:25`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L25), [`lekiwi_control/package.xml:18-19`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/package.xml#L18-L19) |
| Runtime + version | ROS 2 (Humble / Jazzy compatible, Linux aarch64/x86_64) | [`AGENTS.md:5`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md#L5), [`lekiwi_bringup/package.xml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/package.xml) |
| Package manager | `apt` / `rosdep` (ROS packages), `pip` | [`lekiwi_control/package.xml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/package.xml), [`teleop_zhongli_servo_hw/package.xml`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/package.xml) |
| Module/build system | `colcon` build tool, `ament_cmake` and `ament_cmake_python` | [`AGENTS.md:5`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md#L5), [`lekiwi_perception/CMakeLists.txt:1-14`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L1-L14), [`teleop_zhongli_servo_hw/CMakeLists.txt:1-10`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L1-L10), [`lekiwi_control/CMakeLists.txt:1-8`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt#L1-L8) |

### 2) Production Frameworks and Dependencies

List only high-impact production dependencies (frameworks, data, transport, auth).

| Dependency | Version | Role in system | Evidence |
|------------|---------|----------------|----------|
| `rclcpp` / `rclcpp_components` / `rclcpp_lifecycle` | ROS 2 core | C++ Node, Lifecycle Node, and Component composition framework | [`lekiwi_perception/package.xml:19-21`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/package.xml#L19-L21), [`teleop_zhongli_servo_hw/package.xml:13`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/package.xml#L13) |
| `rclpy` | ROS 2 core | Python orchestration node runtime and async event loop | [`lekiwi_control/package.xml:19`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/package.xml#L19) |
| `hardware_interface` / `pluginlib` | ROS 2 ros2_control | Hardware abstraction interface for Feetech STS motor bus & ICM-20948 IMU sensor | [`lekiwi_ftservo_hardware/package.xml:13-14`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/package.xml#L13-L14), [`lekiwi_icm20948_hardware/package.xml:13-14`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/package.xml#L13-L14) |
| `gstreamer-1.0` (`app`, `video`) | 1.0+ | Video streaming, zero-copy valve gating, format conversion (`libcamerasrc`, `v4l2src`) | [`lekiwi_perception/CMakeLists.txt:28-32`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L28-L32), [`lekiwi_bringup/config/perception/gscam_cameras.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml) |
| `hailo-tappas-core` / `gsthailometa` | TAPPAS 3.x+ | Hailo-8 / Hailo-8L NPU hardware-accelerated deep learning inference and post-processing | [`lekiwi_perception/CMakeLists.txt:33-36`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L33-L36), [`lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/hailo/hailo_gst_pipeline.cpp) |
| `OpenCV` (`core`, `imgproc`, `dnn`, `highgui`) | 4.x | Image manipulation, perspective transformation, GUI debug rendering | [`lekiwi_perception/CMakeLists.txt:26`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L26), [`lekiwi_perception/src/chess_visualizer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/chess_visualizer_component.cpp) |
| `libserial-dev` / `libserial` | System library | Low-latency serial bus communication with Feetech STS servos and uArm / Zhongli leader servos | [`lekiwi_ftservo_hardware/package.xml:19`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/package.xml#L19), [`teleop_zhongli_servo_hw/package.xml:18`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/package.xml#L18), [`teleop_zhongli_servo_hw/src/zhongli_protocol.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/zhongli_protocol.cpp) |
| `diagnostic_updater` / `diagnostic_aggregator` | ROS 2 | Real-time hardware health, frame rate, latency, system CPU/RAM/Disk, and bus error diagnostics | [`lekiwi_ftservo_hardware/package.xml:12`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/package.xml#L12), [`teleop_zhongli_servo_hw/package.xml:15`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/package.xml#L15), [`lekiwi_bringup/launch/diagnostics.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/diagnostics.launch.py) |
| `cv_bridge` | ROS 2 | Conversion between ROS `sensor_msgs/Image` and OpenCV `cv::Mat` | [`lekiwi_perception/package.xml:14`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/package.xml#L14) |
| `yaml-cpp` / `PyYAML` | 0.7+ / 6.0+ | Configuration file parsing for joint calibration, offsets, and limits | [`lekiwi_ftservo_hardware/package.xml:18`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/package.xml#L18), [`teleop_zhongli_servo_hw/package.xml:17`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/package.xml#L17), [`lekiwi_control/lekiwi_control/lerobot_arm_bridge.py:9`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/lerobot_arm_bridge.py#L9) |

### 3) Development Toolchain

| Tool | Purpose | Evidence |
|------|---------|----------|
| `ament_cmake_gtest` | C++ unit and component testing (GoogleTest framework) | [`lekiwi_perception/CMakeLists.txt:139-150`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L139-L150), [`teleop_zhongli_servo_hw/CMakeLists.txt:93-100`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L93-L100), [`lekiwi_ftservo_hardware/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/CMakeLists.txt) |
| `ament_cmake_pytest` / `pytest` | Python unit testing for FSM state transitions and orchestrator logic | [`lekiwi_control/CMakeLists.txt:18-21`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt#L18-L21) |
| `GCC` / `Clang` compiler flags | Enforcing strict C++ standard: `-Wall -Wextra -Wpedantic` | [`lekiwi_perception/CMakeLists.txt:9-11`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L9-L11), [`teleop_zhongli_servo_hw/CMakeLists.txt:4-6`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L4-L6), [`AGENTS.md:45`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md#L45) |
| `rosidl_default_generators` | Generation of C++ and Python message / service bindings from `.msg` and `.srv` | [`lekiwi_interfaces/package.xml:11`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_interfaces/package.xml#L11) |

### 4) Key Commands

```bash
# Build entire workspace with symlink install
colcon build --symlink-install

# Build single focused package
colcon build --packages-select lekiwi_perception
colcon build --packages-select lekiwi_control
colcon build --packages-select lekiwi_ftservo_hardware
colcon build --packages-select teleop_zhongli_servo_hw

# Source workspace environment
source install/setup.bash

# Run unit tests across workspace
colcon test --event-handlers console_direct+
colcon test-result --verbose

# Run main robot stack launch entrypoint
ros2 launch lekiwi_bringup robot.launch.py

# Run standalone component debugging launch files
ros2 launch lekiwi_bringup description.launch.py hardware_type:=mock start_controller_manager:=true activate_controllers:=true
ros2 launch lekiwi_bringup cameras.launch.py use_test_sources:=true
ros2 launch lekiwi_bringup control.launch.py start_lerobot_bridge:=true
ros2 launch lekiwi_bringup teleop_uarm.launch.py port:=/dev/uarm_leader
ros2 launch lekiwi_bringup teleop_gamepad.launch.py
ros2 launch lekiwi_bringup diagnostics.launch.py
```

### 5) Environment and Config

- Config sources:
  - [`lekiwi_bringup/config/controllers/lekiwi_controllers.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/controllers/lekiwi_controllers.yaml) (ros2_control controller manager configurations)
  - [`lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml) (Joint limits, drive mode, homing offset, velocity ratios)
  - [`lekiwi_bringup/config/servos/uarm_teleop_calib.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/servos/uarm_teleop_calib.yaml) (uArm leader arm physical calibration and joint remapping configs)
  - [`lekiwi_bringup/config/control/uarm_teleop.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/control/uarm_teleop.yaml) (uArm leader node runtime parameters)
  - [`lekiwi_bringup/config/control/gamepad_base_teleop.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/control/gamepad_base_teleop.yaml) (Gamepad axes & buttons mapping)
  - [`lekiwi_bringup/config/perception/cameras.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/cameras.yaml) (Hailo inference parameters, model dimensions, timeout)
  - [`lekiwi_bringup/config/perception/gscam_cameras.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml) (GStreamer pipelines for 4 camera endpoints with valve gating)
  - [`lekiwi_bringup/config/sensors/imu_filter.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/sensors/imu_filter.yaml) (Madgwick / complementary filter parameters)
  - [`lekiwi_bringup/config/diagnostics/lekiwi_analyzers.yaml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/diagnostics/lekiwi_analyzers.yaml) (Diagnostic aggregator tree configuration)
  - [`lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules) (Device symlinks: `/dev/lekiwi_serial`, `/dev/uarm_leader`, `/dev/usb_wrist`, `/dev/usb_side`, `/dev/gamepad`)
- Required env vars / Device paths:
  - `/dev/lekiwi_serial` (Serial port for Feetech STS servos bus on robot)
  - `/dev/uarm_leader` (Serial port for uArm / Zhongli teleop leader arm)
  - `/dev/i2c-1` (I2C device bus for ICM-20948 IMU)
  - `/dev/gamepad` (Linux joystick device for base teleop)
  - `/dev/usb_wrist`, `/dev/usb_side` (V4L2 camera devices)
  - `/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10` (Raspberry Pi 5 CSI IMX219 stereo camera devices)
- Deployment/runtime constraints:
  - Raspberry Pi 5 single-board computer running Linux (aarch64) with Hailo-8 / Hailo-8L M.2 HAT.
  - HailoRT runtime drivers and TAPPAS libraries installed in system (`/usr/include/hailo`).
  - GStreamer 1.0 with `libcamerasrc` (libcamera) and `v4l2src` plugin support.

### 6) Evidence

- [`lekiwi_interfaces/package.xml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_interfaces/package.xml)
- [`teleop_zhongli_servo_hw/package.xml`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/package.xml)
- [`teleop_zhongli_servo_hw/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt)
- [`lekiwi_perception/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt)
- [`lekiwi_control/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt)
- [`lekiwi_ftservo_hardware/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/CMakeLists.txt)
- [`lekiwi_icm20948_hardware/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/CMakeLists.txt)
- [`lekiwi_bringup/launch/robot.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_bringup/launch/teleop_uarm.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/teleop_uarm.launch.py)
- [`lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules)
- [`AGENTS.md`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md)

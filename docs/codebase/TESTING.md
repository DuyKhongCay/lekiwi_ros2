# Testing Patterns

## Core Sections (Required)

### 1) Test Stack and Commands

- **Primary test frameworks**:
  - C++: GoogleTest (GTest) integrated via `ament_cmake_gtest`.
  - Python: `pytest` integrated via `ament_cmake_pytest`.
- **Assertion / mocking tools**: Standard GTest assertions (`EXPECT_EQ`, `ASSERT_TRUE`, `EXPECT_FALSE`), Python `assert`, GStreamer `videotestsrc` mock elements, and `mock_components/GenericSystem` in ros2_control.
- **Commands**:

```bash
# Run all unit and integration tests across the workspace
colcon test --event-handlers console_direct+

# Run tests for a specific package
colcon test --packages-select teleop_zhongli_servo_hw --event-handlers console_direct+
colcon test --packages-select lekiwi_perception --event-handlers console_direct+
colcon test --packages-select lekiwi_control --event-handlers console_direct+
colcon test --packages-select lekiwi_ftservo_hardware --event-handlers console_direct+
colcon test --packages-select lekiwi_icm20948_hardware --event-handlers console_direct+

# Inspect test results and failures
colcon test-result --verbose
```

---

### 2) Test Layout

- **Test file placement**: Co-located within the `test/` directory of each ROS 2 package.
- **Naming convention**: `test_<feature>.cpp` for C++ tests, `test_<feature>.py` for Python tests.
- **Setup files**:
  - [`teleop_zhongli_servo_hw/test/test_command_format.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/test/test_command_format.cpp)
  - [`teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp)
  - [`lekiwi_perception/test/test_camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_camera_streamer_component.cpp)
  - [`lekiwi_perception/test/test_hailo_gst_pipeline.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_hailo_gst_pipeline.cpp)
  - [`lekiwi_control/test/test_fsm.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/test/test_fsm.py)
  - [`lekiwi_control/test/test_task_orchestrator.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/test/test_task_orchestrator.py)
  - [`lekiwi_ftservo_hardware/test/test_sts_protocol.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/test/test_sts_protocol.cpp)
  - [`lekiwi_ftservo_hardware/test/test_velocity_codec.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/test/test_velocity_codec.cpp)
  - [`lekiwi_icm20948_hardware/test/test_icm20948_math.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/test/test_icm20948_math.cpp)

---

### 3) Test Scope Matrix

| Scope | Covered? | Typical target | Notes |
|-------|----------|----------------|-------|
| **Unit** | Yes | Feetech & Zhongli packet formatting, checksum calculation, kinematic remapping calculations, FSM transitions, IMU filtering | Fully isolated in-memory unit tests; executes deterministically on CI and developer machines. |
| **Integration** | Yes | `CameraStreamerComponent` lifecycle transitions (`configure`, `activate`, `deactivate`, `cleanup`), dynamic valve gating, orchestrator client/service messaging | Uses synthetic GStreamer sources (`videotestsrc`) without physical camera hardware. |
| **E2E / HIL** | Manual / Configured | Full robot motion via `ros2 launch lekiwi_bringup robot.launch.py hardware_type:=real` | Requires physical LeKiwi robot, Hailo-8 NPU, and serial motor buses. |

---

### 4) Mocking and Isolation Strategy

- **Camera Hardware Mocking**:
  - Tests substitute physical camera sources (`libcamerasrc`, `v4l2src`) with GStreamer synthetic generators: `videotestsrc is-live=true ! ...` ([`lekiwi_perception/test/test_camera_streamer_component.cpp:38`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_camera_streamer_component.cpp#L38)).
- **Hardware Interface Mocking**:
  - `ros2_control` Xacro templates include condition branches for `mock_components/GenericSystem` when `hardware_type == 'mock'`, enabling full controller stack startup without serial hardware ([`lekiwi_description/urdf/ros2_control.xacro:30-32`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_description/urdf/ros2_control.xacro#L30-L32)).
- **Isolation Guarantees**:
  - `rclcpp::init()` and `rclcpp::shutdown()` are managed cleanly in GTest fixtures (`SetUp`/`TearDown`) to avoid state leakage between test cases.

---

### 5) Coverage and Quality Signals

- **Coverage tool**: `lcov` / `pytest-cov` (configured on demand via colcon).
- **Current status**:
  - Core protocol codecs (`zhongli_protocol`, `sts_protocol`, `velocity_codec`, `fsm`, `kinematic_remapper`) have robust unit test coverage.
  - Teleop kinematic conversion verifies joint clamping, scale factors, and direction inversions.
- **Known gaps / test risks**:
  - Tests for HailoRT hardware inference execution require the physical Hailo-8 NPU device driver and cannot execute on machines without the Hailo M.2 HAT.

---

### 6) Evidence

- [`teleop_zhongli_servo_hw/CMakeLists.txt:92-105`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L92-L105)
- [`lekiwi_perception/CMakeLists.txt:138-150`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L138-L150)
- [`lekiwi_control/CMakeLists.txt:17-21`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt#L17-L21)
- [`lekiwi_ftservo_hardware/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/CMakeLists.txt)
- [`lekiwi_icm20948_hardware/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/CMakeLists.txt)
- [`teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp)
- [`lekiwi_perception/test/test_camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_camera_streamer_component.cpp)
- [`lekiwi_control/test/test_fsm.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/test/test_fsm.py)

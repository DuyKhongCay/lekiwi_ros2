# Testing Patterns

## Core Sections (Required)

### 1) Test Stack and Commands

- **Primary test frameworks**:
  - C++: GoogleTest (GTest) integrated via `ament_cmake_gtest`.
  - Python: `pytest` integrated via `ament_cmake_pytest`.
- **Assertion / mocking tools**: Standard GTest assertions (`EXPECT_EQ`, `ASSERT_TRUE`, `EXPECT_FALSE`, `EXPECT_NEAR`), Python `assert`, GStreamer `videotestsrc` mock elements, and `mock_components/GenericSystem` in ros2_control.
- **Commands**:

```bash
# Run all unit and integration tests across the workspace
colcon test --event-handlers console_direct+

# Run tests for a specific package
colcon test --packages-select lekiwi_tag_localization --event-handlers console_direct+
colcon test --packages-select teleop_zhongli_servo_hw --event-handlers console_direct+
colcon test --packages-select lekiwi_perception --event-handlers console_direct+
colcon test --packages-select lekiwi_control --event-handlers console_direct+
colcon test --packages-select lekiwi_ftservo_hardware --event-handlers console_direct+
colcon test --packages-select lekiwi_icm20948_hardware --event-handlers console_direct+

# Inspect test results and verbose failure logs
colcon test-result --verbose
```

---

### 2) Test Layout

- **Test file placement**: Co-located within the `test/` directory of each ROS 2 package.
- **Naming convention**: `test_<feature>.cpp` for C++ tests, `test_<feature>.py` for Python tests.
- **Setup files**:
  - [`lekiwi_tag_localization/test/test_pose_solver.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/test/test_pose_solver.cpp) (AprilTag SolvePnP mathematical correctness & board corner projection tests)
  - [`teleop_zhongli_servo_hw/test/test_command_format.cpp`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/test/test_command_format.cpp)
  - [`teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp)
  - [`lekiwi_perception/test/test_camera_streamer_component.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_camera_streamer_component.cpp)
  - [`lekiwi_perception/test/test_hailo_gst_pipeline.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_hailo_gst_pipeline.cpp)
  - [`lekiwi_control/test/test_fsm.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_control/test/test_fsm.py)
  - [`lekiwi_control/test/test_task_orchestrator.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_control/test/test_task_orchestrator.py)
  - [`lekiwi_ftservo_hardware/test/test_sts_protocol.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/test/test_sts_protocol.cpp)
  - [`lekiwi_ftservo_hardware/test/test_velocity_codec.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/test/test_velocity_codec.cpp)
  - [`lekiwi_icm20948_hardware/test/test_icm20948_math.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/test/test_icm20948_math.cpp)

---

### 3) Test Scope Matrix

| Scope | Covered? | Typical target | Notes |
|-------|----------|----------------|-------|
| **Unit** | Yes | SolvePnP math, Feetech & Zhongli packet formatting, checksum calculation, kinematic remapping calculations, FSM transitions, IMU filtering | Fully isolated in-memory unit tests; executes deterministically on CI and developer machines. |
| **Integration** | Yes | `CameraStreamerComponent` lifecycle transitions (`configure`, `activate`, `deactivate`, `cleanup`), dynamic valve gating, orchestrator client/service messaging | Uses synthetic GStreamer sources (`videotestsrc`) without physical camera hardware. |
| **E2E / HIL** | Manual / Configured | Full robot motion & Nav2 navigation via `ros2 launch lekiwi_bringup robot.launch.py hardware_type:=real enable_navigation:=true` | Requires physical LeKiwi robot, Hailo-8 NPU, and serial motor buses. |

---

### 4) Mocking and Isolation Strategy

- **Pose Solver Geometric Math Isolation**:
  - `lekiwi_tag_localization/test/test_pose_solver.cpp` verifies corner transformations, synthetic 3D-2D reprojections, and yaw offset rotations without requiring live camera frames or AprilTag detector nodes.
- **Camera Hardware Mocking**:
  - Tests substitute physical camera sources (`libcamerasrc`, `v4l2src`) with GStreamer synthetic generators: `videotestsrc is-live=true ! ...` ([`lekiwi_perception/test/test_camera_streamer_component.cpp:38`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_camera_streamer_component.cpp#L38)).
- **ros2_control Hardware Simulation**:
  - `description.launch.py` and `robot.launch.py` provide `hardware_type:=mock` launching `mock_components/GenericSystem` plugin to test controller loops without physical servos.

---

### 5) Coverage and Quality Signals

- Coverage tool: Standard GCC/Clang `lcov` / `gcov` compatible.
- Quality signals:
  - Strict compiler warning flags (`-Wall -Wextra -Wpedantic`) enabled across all CMakeLists.
  - Zero compiler warnings in production packages.

---

### 6) Evidence

- [`lekiwi_tag_localization/CMakeLists.txt`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/CMakeLists.txt)
- [`lekiwi_tag_localization/test/test_pose_solver.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/test/test_pose_solver.cpp)
- [`lekiwi_perception/test/test_camera_streamer_component.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/test/test_camera_streamer_component.cpp)
- [`teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/test/test_kinematic_conversion.cpp)
- [`lekiwi_ftservo_hardware/test/test_sts_protocol.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/test/test_sts_protocol.cpp)

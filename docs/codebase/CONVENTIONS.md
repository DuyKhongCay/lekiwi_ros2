# Coding Conventions

## Core Sections (Required)

### 1) Naming Rules

| Item | Rule | Example | Evidence |
|------|------|---------|----------|
| Source Files (C++) | `snake_case` with `.cpp` and `.hpp` | `camera_streamer_component.cpp`, `sts_protocol.hpp`, `teleop_uarm_node.cpp` | [`lekiwi_perception/src/camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp), [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp) |
| Source Files (Python) | `snake_case` with `.py` | `task_orchestrator.py`, `lerobot_arm_bridge.py` | [`lekiwi_control/lekiwi_control/task_orchestrator.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py) |
| ROS 2 Messages / Services | `PascalCase` with `.msg` and `.srv` | `CameraMode.msg`, `HailoInferenceStatus.msg`, `SetCamMode.srv` | [`lekiwi_interfaces/msg/CameraMode.msg`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_interfaces/msg/CameraMode.msg) |
| C++ Types / Classes | `PascalCase` | `HailoChessInferenceComponent`, `LeKiwiFeetechHardwareInterface`, `TeleopUarmNode` | [`lekiwi_perception/include/hailo_chess_inference_component.hpp:31`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/include/hailo_chess_inference_component.hpp#L31), [`teleop_zhongli_servo_hw/include/teleop_zhongli_servo_hw/teleop_uarm_node.hpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/include/teleop_zhongli_servo_hw/teleop_uarm_node.hpp) |
| Functions & Methods | `snake_case` or `camelCase` (with trailing `_` for C++ private members) | `handle_sample()`, `connect_hardware()`, `gscam_config_`, `port_` | [`lekiwi_control/lekiwi_control/fsm.py:37`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/fsm.py#L37), [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp:40`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp#L40) |
| Constants / Enums | `UPPER_SNAKE_CASE` | `ALLOWED_TRANSITIONS`, `RAW_POSITION_SPAN`, `STANDBY` | [`lekiwi_control/lekiwi_control/fsm.py:7-27`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/fsm.py#L7-L27), [`lekiwi_interfaces/msg/CameraMode.msg:3-6`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_interfaces/msg/CameraMode.msg#L3-L6) |
| Topics and Services | `snake_case` starting with `/` | `/camera_mode`, `/leader/joint_states`, `/arm_trajectory_controller/joint_trajectory`, `/omni_base_controller/cmd_vel` | [`lekiwi_control/lekiwi_control/task_orchestrator.py:27-30`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py#L27-L30), [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp:27`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp#L27) |

#### Standard Abbreviation Dictionary
As mandated by [`AGENTS.md:99-140`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md#L99-L140), variables and functions composed of multiple words must use the following standard abbreviations:
`buffer -> buff`, `width -> w`, `height -> h`, `acknowledge -> ack`, `description -> desc`, `destination -> dest`, `source -> src`, `diagnostic -> diag`, `calibration -> calib`, `capture -> cap`, `rotation -> rot`, `pieces -> pcs`, `point -> pt`, `points -> pts`, `geometry -> geom`, `camera -> cam`, `cameras -> cams`, `left -> l`, `right -> r`, `confidence -> conf`, `matrix -> mat`, `generation -> gen`, `detection -> det`, `detections -> dets`, `iteration -> iter`, `result -> res`, `return -> ret`, `distance -> dist`, `class -> cls`, `count -> cnt`, `argument -> arg`, `runtime -> rt`, `error -> err`, `parameter -> param`, `image -> img`, `information -> info`, `latency -> lat`, `multiply -> mul`, `transformation -> trans`, `configuration -> configs`, `display -> disp`, `statistic -> stats`, `previous -> prev`.

---

### 2) Formatting and Linting

- **C++ Conventions**:
  - Two-space indentation, braces on their own line.
  - Compiled with strict compiler flags: `-Wall -Wextra -Wpedantic` ([`lekiwi_perception/CMakeLists.txt:10`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L10), [`teleop_zhongli_servo_hw/CMakeLists.txt:5`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L5)).
  - Clean compilation required (zero warnings policy).
- **Python Conventions**:
  - Four-space indentation.
  - Compliant with PEP 8 and PEP 257.
- **Documentation Structure**:
  - Default rule: Every class, function, and method must have comments explaining the "why" / intent ([`AGENTS.md:47-67`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md#L47-L67)).
  - Strict compliance mode: Python uses PEP 257 docstrings with typed `Args:`, C++ uses full Doxygen block tags (`@brief`, `@param`, `@return`).

---

### 3) Import and Module Conventions

- **C++ Include Order**:
  1. Standard library headers (`<atomic>`, `<chrono>`, `<memory>`, `<vector>`).
  2. Third-party C/C++ library headers (`<gst/gst.h>`, `<opencv2/opencv.hpp>`, `<yaml-cpp/yaml.h>`).
  3. ROS 2 core and message headers (`<rclcpp/rclcpp.hpp>`, `<sensor_msgs/msg/image.hpp>`, `<trajectory_msgs/msg/joint_trajectory.hpp>`).
  4. Project local headers (`"teleop_zhongli_servo_hw/teleop_uarm_node.hpp"`, `"camera_streamer_component.hpp"`).
- **Python Import Order**:
  1. Standard library (`threading`, `math`, `pathlib`, `dataclasses`).
  2. Third-party libraries (`yaml`).
  3. ROS 2 core packages (`rclpy`, `rclpy.node`, `rclpy.qos`).
  4. ROS 2 messages and interfaces (`lekiwi_interfaces.msg`, `control_msgs.action`).
  5. Local package modules (`from lekiwi_control.fsm import ...`).

---

### 4) Error and Logging Conventions

- **Logging**:
  - C++ uses `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`, and `RCLCPP_DEBUG` macros with `get_logger()`.
  - Python uses `self.get_logger().info()`, `self.get_logger().warning()`, `self.get_logger().error()`.
- **Diagnostics**:
  - Real-time diagnostic telemetry reported via `diagnostic_updater::Updater` in perception, teleop, and hardware nodes ([`lekiwi_perception/src/camera_streamer_component.cpp:57`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp#L57), [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp:31-34`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp#L31-L34), [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp:117`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp#L117)).
- **Hardware Bus Resilience**:
  - Checksum validation, timeouts, and error counters (`consecutive_error_count`, `serial_error_count`) published in `DriveStatus.msg` and reported to diagnostics.

---

### 5) Testing Conventions

- **Test Placement**:
  - C++ tests placed in `<package>/test/test_<feature>.cpp`.
  - Python tests placed in `<package>/test/test_<feature>.py`.
- **Frameworks**:
  - C++: GoogleTest via `ament_add_gtest`.
  - Python: pytest via `ament_add_pytest_test`.
- **Isolation Policy**:
  - Tests must never require physical camera hardware or physical serial USB dongles to be plugged in.
  - Mock streams (`videotestsrc`), synthetic packets, and `mock_components/GenericSystem` are used for CI and automated regression.

---

### 6) Evidence

- [`AGENTS.md:44-144`](file://$HOME/docker_ws/lekiwi_ros2/AGENTS.md#L44-L144)
- [`lekiwi_perception/CMakeLists.txt:9-11`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L9-L11)
- [`teleop_zhongli_servo_hw/CMakeLists.txt:4-6`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt#L4-L6)
- [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp)
- [`lekiwi_perception/src/camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp)
- [`lekiwi_control/lekiwi_control/fsm.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/fsm.py)
- [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp)

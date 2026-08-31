# Codebase Structure

## Core Sections (Required)

### 1) Top-Level Map

List only meaningful top-level directories and files.

| Path | Purpose | Evidence |
|------|---------|----------|
| [`lekiwi_interfaces/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_interfaces) | Custom ROS 2 messages (`CameraMode`, `HailoInferenceStatus`, `DriveStatus`, `ServoTelemetry`) and services (`SetCamMode`, `ResetMotorBus`, `SetDriveEnabled`) | [`lekiwi_interfaces/package.xml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_interfaces/package.xml) |
| [`lekiwi_perception/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception) | C++ GStreamer video streamer components, HailoRT NPU chess inference component, post-processing plugins, FEN board mapping, and debug visualizer | [`lekiwi_perception/CMakeLists.txt`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt) |
| [`lekiwi_control/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control) | Python task orchestrator node managing lifecycle boot sequences, mode transitions, and LeRobot follower arm trajectory bridge | [`lekiwi_control/CMakeLists.txt`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt) |
| [`lekiwi_ftservo_hardware/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware) | `ros2_control` C++ hardware interface plugin (`SystemInterface`) for Feetech STS serial servos bus using an asynchronous I/O worker thread | [`lekiwi_ftservo_hardware/lekiwi_ftservo_hardware_plugin.xml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/lekiwi_ftservo_hardware_plugin.xml) |
| [`lekiwi_icm20948_hardware/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware) | `ros2_control` C++ sensor interface plugin (`SensorInterface`) for the ICM-20948 9-DoF IMU communicating over I2C | [`lekiwi_icm20948_hardware/lekiwi_icm20948_hardware_plugin.xml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/lekiwi_icm20948_hardware_plugin.xml) |
| [`lekiwi_description/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_description) | URDF / Xacro kinematics and joint definitions (`lekiwi_robot.urdf.xacro`, `ros2_control.xacro`), mesh STL files, and CAD assets | [`lekiwi_description/urdf/lekiwi_robot.urdf.xacro`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_description/urdf/lekiwi_robot.urdf.xacro) |
| [`lekiwi_bringup/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup) | Top-level runtime launch files (`robot.launch.py`, `cameras.launch.py`, `control.launch.py`), parameter YAML configurations, and system udev rules | [`lekiwi_bringup/launch/robot.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py) |
| [`scripts/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/scripts) | Standalone developer and diagnostic utilities (`analyzing_traces.py`, `save_image.py`) | [`scripts/analyzing_traces.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/scripts/analyzing_traces.py) |
| [`deprecated/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/deprecated) | Archived legacy modules ignored during colcon build via `COLCON_IGNORE` (`hailo_perception_node`, `lekiwi_hardwares`, `lekiwi_perception_bak`) | [`deprecated/COLCON_IGNORE`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/deprecated/COLCON_IGNORE) |
| [`docs/codebase/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/docs/codebase) | Standardized codebase documentation created by the `acquire-codebase-knowledge` workflow | [`docs/codebase/.codebase-scan.txt`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/docs/codebase/.codebase-scan.txt) |

### 2) Entry Points

- **Main runtime entry point**:
  - `ros2 launch lekiwi_bringup robot.launch.py` ([`lekiwi_bringup/launch/robot.launch.py:12-181`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py#L12-L181)): Orchestrates the entire robot topology, bringing up robot description, composed camera components, task orchestrator, IMU filter pipeline, and optional teleoperation.
- **Subsystem entry points**:
  - `ros2 launch lekiwi_bringup cameras.launch.py` ([`lekiwi_bringup/launch/cameras.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/cameras.launch.py)): Starts the `lekiwi_perception_container` containing 4 `CameraStreamerComponent` instances and the `HailoChessInferenceComponent`.
  - `ros2 launch lekiwi_bringup description.launch.py` ([`lekiwi_bringup/launch/description.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/description.launch.py)): Loads robot description, TF2 publisher (`robot_state_publisher`), and `ros2_control` controller manager.
  - `ros2 launch lekiwi_bringup control.launch.py` ([`lekiwi_bringup/launch/control.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/control.launch.py)): Launches `task_orchestrator` and optional `lerobot_arm_bridge`.
  - `ros2 launch lekiwi_bringup imu.launch.py` ([`lekiwi_bringup/launch/imu.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/imu.launch.py)): Launches IMU sensor broadcaster, Madgwick/Complementary filter, and IMU frame transformer.
  - `ros2 launch lekiwi_bringup teleop.launch.py` ([`lekiwi_bringup/launch/teleop.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/teleop.launch.py)): Launches joy and teleop twist nodes for gamepad control.
- **Executable Node Binaries**:
  - `hailo_chess_inference_node` ([`lekiwi_perception/CMakeLists.txt:104-106`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L104-L106))
  - `camera_streamer_node` ([`lekiwi_perception/CMakeLists.txt:114-116`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L114-L116))
  - `chess_visualizer_node` ([`lekiwi_perception/CMakeLists.txt:109-111`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt#L109-L111))
  - `task_orchestrator` ([`lekiwi_control/CMakeLists.txt:11`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt#L11))
  - `lerobot_arm_bridge` ([`lekiwi_control/CMakeLists.txt:13`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt#L13))

### 3) Module Boundaries

| Boundary | What belongs here | What must not be here |
|----------|-------------------|------------------------|
| `lekiwi_interfaces` | `.msg` and `.srv` interface schemas only | Execution logic, C++/Python nodes, hardware protocol libraries |
| `lekiwi_perception` | GStreamer pipelines, HailoRT inference, OpenCV processing, vision diagnostics | Robot joint commands, serial/I2C hardware drivers, FSM mode switching decisions |
| `lekiwi_control` | Task orchestration FSM, lifecycle state management, LeRobot arm trajectory translation | Direct `/dev/` file access, GStreamer pipelines, low-level actuator communication |
| `lekiwi_ftservo_hardware` | Feetech STS serial packet encoding/decoding, ros2_control SystemInterface integration, async serial I/O | Deep learning models, image processing, high-level path planning |
| `lekiwi_icm20948_hardware` | ICM-20948 I2C register configuration, gyro calibration, ros2_control SensorInterface | Motor commands, FSM orchestration, image processing |
| `lekiwi_description` | URDF/Xacro kinematic definitions, joint limits, meshes, transmission macros | ROS executable nodes, sensor data processing, serial communications |
| `lekiwi_bringup` | Launch compositions, runtime YAML parameter files, udev rules | Algorithmic code, custom message definitions, C++ plugin implementations |

### 4) Naming and Organization Rules

- **File naming pattern**:
  - C++ source and header files: `snake_case` with `.cpp` and `.hpp` (e.g. `camera_streamer_component.cpp`, `sts_protocol.hpp`).
  - Python scripts and modules: `snake_case` with `.py` (e.g. `task_orchestrator.py`, `lerobot_arm_bridge.py`).
  - ROS 2 messages and services: `PascalCase` with `.msg` and `.srv` (e.g. `CameraMode.msg`, `SetCamMode.srv`).
  - Configuration files: `snake_case.yaml` (e.g. `lekiwi_joints.yaml`, `gscam_cameras.yaml`).
- **Directory organization pattern**:
  - Organized by ROS 2 package layer (`include/<package>/`, `src/`, `config/`, `launch/`, `test/`, `urdf/`, `assets/`).
- **Import / include conventions**:
  - C++ public headers placed in `include/` and exported via `ament_export_include_directories(include)`.
  - Python packages declared in `setup.py` / `ament_python_install_package` with internal imports like `from lekiwi_control.fsm import ...`.

### 5) Evidence

- [`lekiwi_bringup/launch/robot.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_perception/CMakeLists.txt`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt)
- [`lekiwi_control/CMakeLists.txt`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt)
- [`lekiwi_ftservo_hardware/lekiwi_ftservo_hardware_plugin.xml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/lekiwi_ftservo_hardware_plugin.xml)
- [`lekiwi_icm20948_hardware/lekiwi_icm20948_hardware_plugin.xml`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/lekiwi_icm20948_hardware_plugin.xml)
- [`lekiwi_description/urdf/lekiwi_robot.urdf.xacro`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_description/urdf/lekiwi_robot.urdf.xacro)
- [`deprecated/COLCON_IGNORE`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/deprecated/COLCON_IGNORE)


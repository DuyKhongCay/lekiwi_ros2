# Architecture

## Core Sections (Required)

### 1) Architectural Style

- **Primary style**: Component-Based ROS 2 Architecture with Lifecycle Node Management, Event-Driven Task Orchestration, Tag-Based Fiducial Localization, Nav2 Holonomic Path Execution, and Hardware Asynchronous I/O Execution.
- **Why this classification**:
  - All vision data paths run as composed in-process components (`rclcpp_components` inside `component_container_mt`) using zero-copy intra-process communications.
  - Critical perception nodes implement standard ROS 2 lifecycle states (`unconfigured` -> `inactive` -> `active`) managed autonomously by the `TaskOrchestratorNode`.
  - The motor control hardware driver isolates real-time `ros2_control` cycles (< 10 µs `read()`/`write()`) from 1 Mbps serial bus communication latency via a dedicated asynchronous background I/O worker thread and thread-safe shared double buffers.
  - Localization relies on AprilTag 36h11 markers and OpenCV SolvePnP (`lekiwi_tag_localization`) broadcasting dynamic coordinate transforms between the robot camera and the chessboard arena.
  - Navigation utilizes the Nav2 SmacPlanner2D and DWB controller with static costmaps and priority-based `twist_mux` arbitration for smooth omni-directional trajectory tracking.
- **Primary constraints**:
  - **Embedded SBC Constraints**: Must execute efficiently on Raspberry Pi 5 ARM64 architecture under memory and CPU constraints.
  - **Zero-Copy Vision Bandwidth**: High-resolution CSI (IMX219) and USB camera frames are gated at the GStreamer pipeline level (`valve` elements) so inactive cameras consume 0 CPU/NPU resources.
  - **Real-Time Control Loop**: The `ros2_control` update loop (50 Hz) must never block on serial I/O or UART timeouts.
  - **Sensorless Arena Safety**: Costmaps operate on static arena geometry + keepout inflation layers without requiring an active 2D LiDAR scanner.

---

### 2) System Flow

```text
                                      +------------------------------------+
                                      |       Task Orchestrator FSM        |
                                      |      (lekiwi_control / rclpy)      |
                                      +-----------------+------------------+
                                                        | /camera_mode
                     +----------------------------------+----------------------------------+
                     |                                                                     |
                     v                                                                     v
+------------------------------------+                                   +------------------------------------+
|    lekiwi_perception Container     |                                   |    ros2_control Controller Mgr     |
|    (GStreamer + HailoRT + OCV)     |                                   |  (lekiwi_bringup / ros2_control)   |
+------------------------------------+                                   +-----------------+------------------+
| • CameraStreamerComponent (x4)     |                                                     |
|   (Dynamic GStreamer Valve Gate)   |                            +------------------------+------------------------+
| • HailoChessInferenceComponent     |                            |                                                 |
|   (Hailo-8/8L NPU YOLO & FEN)      |                            v                                                 v
| • ChessVisualizerComponent         |            +-------------------------------+                 +-------------------------------+
+-----------------+------------------+            |     LeKiwiFeetechHardware     |                 |    ICM20948SensorHardware     |
                  |                               |   (Async I/O Worker Thread)   |                 |  (I2C 9-DoF IMU Broadcaster)  |
                  | /camera/image_raw             +---------------+---------------+                 +---------------+---------------+
                  v                                               | /dev/lekiwi_serial                              | /dev/i2c-1
+------------------------------------+                            v                                                 v
|      lekiwi_tag_localization       |            +-------------------------------+                 +-------------------------------+
|   (AprilTag + SolvePnP Solver)     |            | 9x Feetech STS3215 Servos     |                 | ICM-20948 9-DoF IMU Sensor    |
+-----------------+------------------+            | (6-DoF Arm + 3-Wheel Omni)    |                 +-------------------------------+
                  | TF: map -> chessboard         +-------------------------------+
                  v
+------------------------------------+
|         lekiwi_navigation          |
|    (Nav2 SmacPlanner + DWB)        |
+-----------------+------------------+
                  | /cmd_vel_nav
                  v
+------------------------------------+
|             twist_mux              | ===> /omni_base_controller/cmd_vel
| (P10:Nav2, P100:Teleop, P255:EStop)|
+------------------------------------+
```

1. **Vision Streaming & Gating**: `CameraStreamerComponent` opens GStreamer capture pipelines; dynamic `valve` elements only push image frames when enabled by `/camera_mode`.
2. **NPU Neural Inference & Board Detection**: In `CHESS_MODE`, camera frames are processed on the Hailo-8 NPU for YOLO piece detection and mapped to FEN board notations.
3. **Fiducial Tag Localization**: `lekiwi_tag_localization` detects AprilTags on the chessboard boundary, computes 6-DoF pose via OpenCV SolvePnP, and broadcasts TF frames.
4. **Navigation & Costmap Generation**: `lekiwi_navigation` generates collision-free global and local paths avoiding the central chessboard obstacle zone.
5. **Command Multiplexing & Safety**: `twist_mux` arbitrates between Nav2 `/cmd_vel_nav`, gamepad `/cmd_vel_teleop`, and `/safety/estop_active`.
6. **Actuation Execution**: `ros2_control` dispatches velocity goals to the 3 omni-wheel servos via the asynchronous Feetech hardware driver.

---

### 3) Layer/Module Responsibilities

| Layer or module | Owns | Must not own | Evidence |
|-----------------|------|--------------|----------|
| `lekiwi_interfaces` | Message and service interface definitions | Algorithmic logic, node executables | [`lekiwi_interfaces/package.xml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_interfaces/package.xml) |
| `lekiwi_perception` | Camera streaming, GStreamer lifecycle management, HailoRT NPU deep learning execution, FEN state publishing | Motor control, hardware serial communication | [`lekiwi_perception/CMakeLists.txt`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt) |
| `lekiwi_tag_localization` | AprilTag fiducial detection, SolvePnP chessboard pose estimation, TF coordinate publishing | Direct actuator commands, path planning | [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp) |
| `lekiwi_navigation` | Nav2 planner and controller servers, static costmap configuration, command velocity arbitration | NPU vision models, serial motor protocol parsing | [`lekiwi_navigation/launch/navigation.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/launch/navigation.launch.py) |
| `lekiwi_control` | High-level robot mode FSM, subsystem lifecycle orchestration, LeRobot arm trajectory translation | Low-level serial protocol framing, video encoding | [`lekiwi_control/lekiwi_control/task_orchestrator.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py) |
| `lekiwi_ftservo_hardware` | Feetech STS serial protocol driver, ros2_control SystemInterface, background I/O worker thread | Image processing, global path planning | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp) |
| `lekiwi_icm20948_hardware` | ICM-20948 I2C sensor driver, gyro bias calibration, ros2_control SensorInterface | Motor torque control, higher-level FSM logic | [`lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp) |
| `teleop_zhongli_servo_hw` | Zhongli serial bus driver, midpoint calibration CLI, leader joint kinematic remapping | Vision processing, navigation costmaps | [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp) |
| `lekiwi_description` | URDF/Xacro geometry models, joint limits, transmissions, ros2_control hardware tags | Executable logic, hardware communication | [`lekiwi_description/urdf/lekiwi_robot.urdf.xacro`](file:///root/docker_ws/lekiwi_ros2/lekiwi_description/urdf/lekiwi_robot.urdf.xacro) |
| `lekiwi_bringup` | Master launch files, runtime parameter YAML configs, udev rules | Algorithmic implementations, custom interface schemas | [`lekiwi_bringup/launch/robot.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py) |

---

### 4) Reused Patterns

| Pattern | Where found | Why it exists |
|---------|-------------|---------------|
| **Component Composition (`rclcpp_components`)** | [`lekiwi_perception`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception), [`lekiwi_tag_localization`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization) | Allows high-frequency sensor frames to pass between camera streamers, AprilTag locators, and Hailo inferencers with zero copy IPC. |
| **Lifecycle Managed Node (`rclcpp_lifecycle`)** | [`lekiwi_perception/src/camera_streamer_component.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp) | Provides deterministic startup, shutdown, and error recovery transitions orchestrated by the central FSM. |
| **Asynchronous I/O Worker Thread** | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp:45`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp#L45) | Isolates the real-time `ros2_control` execution loop from blocking 1 Mbps serial UART latencies. |
| **Multiplexed Command Arbitration (`twist_mux`)** | [`lekiwi_navigation/config/twist_mux.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/config/twist_mux.yaml) | Safely prioritizes autonomous navigation, manual gamepad teleoperation, and software emergency stop locks. |
| **SolvePnP Spatial Estimation** | [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp) | Computes accurate 6-DoF chessboard transform relative to camera optical frames. |

---

### 5) Known Architectural Risks

- **Risk 1: Dual Serial Bus Port Inversion**: `/dev/lekiwi_serial` and `/dev/uarm_leader` can swap upon reboot if udev rules are not properly matched to serial numbers.
- **Risk 2: Multi-Stream CPU Bottleneck**: Streaming all 4 cameras simultaneously without dynamic GStreamer valve gating can saturate the Raspberry Pi 5 CPU.
- **Risk 3: Costmap Boundary Bleed**: Inaccurate AprilTag localization could cause the static chessboard obstacle zone to drift, resulting in mobile base collisions.

---

### 6) Evidence

- [`lekiwi_bringup/launch/robot.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_navigation/launch/navigation.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/launch/navigation.launch.py)
- [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp)
- [`lekiwi_perception/src/camera_streamer_component.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp)
- [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)

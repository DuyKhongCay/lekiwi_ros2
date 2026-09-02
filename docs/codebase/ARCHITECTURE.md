# Architecture

## Core Sections (Required)

### 1) Architectural Style

- **Primary style**: Component-Based ROS 2 Architecture with Lifecycle Node Management, Event-Driven Task Orchestration, and Hardware Asynchronous I/O Execution.
- **Why this classification**:
  - All vision data paths run as composed in-process components (`rclcpp_components` inside `component_container_mt`) using zero-copy intra-process communications.
  - Critical perception nodes implement standard ROS 2 lifecycle states (`unconfigured` -> `inactive` -> `active`) managed autonomously by the `TaskOrchestratorNode`.
  - The motor control hardware driver isolates real-time `ros2_control` cycles (< 10 µs `read()`/`write()`) from 1 Mbps serial bus communication latency via a dedicated asynchronous background I/O worker thread and thread-safe shared double buffers.
  - The teleoperation subsystem (`teleop_zhongli_servo_hw`) implements a high-frequency polling loop (50 Hz) decoupling serial leader reading from trajectory generation and diagnostic monitoring (1 Hz).
- **Primary constraints**:
  - **Embedded SBC Constraints**: Must execute efficiently on Raspberry Pi 5 ARM64 architecture under memory and CPU constraints.
  - **Zero-Copy Vision Bandwidth**: High-resolution CSI (IMX219) and USB camera frames are gated at the GStreamer pipeline level (`valve` elements) so inactive cameras consume 0 CPU/NPU resources.
  - **Real-Time Control Loop**: The `ros2_control` update loop (50 Hz) must never block on serial I/O or UART timeouts.
  - **Leader-Follower Remapping**: Leader arm kinematics must deterministically map to follower joint ranges and controller modes (`joint_trajectory`, `forward_position`, `joint_states_only`).

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
+------------------------------------+            |     LeKiwiFeetechHardware     |                 |    ICM20948SensorHardware     |
                                                  |   (Async I/O Worker Thread)   |                 |  (I2C 9-DoF IMU Broadcaster)  |
                                                  +---------------+---------------+                 +---------------+---------------+
                                                                  | /dev/lekiwi_serial                              | /dev/i2c-1
                                                                  v                                                 v
                                                  +-------------------------------+                 +-------------------------------+
                                                  | 9x Feetech STS3215 Servos     |                 | ICM-20948 9-DoF IMU Sensor    |
                                                  | (6-DoF Arm + 3-Wheel Omni)    |                 +-------------------------------+
                                                  +-------------------------------+

                                       [ Teleoperation Subsystems ]
   +------------------------------+                 +-------------------------------+
   |        uArm Leader Arm       |                 |       Gamepad Teleop          |
   | (Zhongli / Feetech Servos)   |                 | (Linux Joystick / joy_teleop) |
   +--------------+---------------+                 +---------------+---------------+
                  | /dev/uarm_leader                                | /dev/gamepad
                  v                                                 v
   +------------------------------+                 +-------------------------------+
   |      teleop_uarm_node        |                 |      joy_linux_node           |
   | (Kinematic Remapper 50 Hz)   |                 |        joy_teleop             |
   +--------------+---------------+                 +---------------+---------------+
                  | /arm_trajectory_controller/joint_trajectory     | /omni_base_controller/cmd_vel
                  +------------------------+------------------------+
                                           |
                                           v
                             [ ros2_control Controllers ]
```

1. **Video Ingestion & Hardware Gating**:
   `CameraStreamerComponent` captures video streams via GStreamer (`libcamerasrc` for CSI stereo cameras, `v4l2src` for USB cameras). If the current camera mode does not match `active_modes`, the GStreamer `valve name=gate` is set to `drop=true`, preventing frames from traversing the downstream queue, color conversion, or ROS publishing pipeline ([`lekiwi_perception/src/camera_streamer_component.cpp:115-135`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp#L115-L135)).

2. **Hardware-Accelerated Perception**:
   In `CHESS_THINKING` mode, stereo camera frames are passed to `HailoChessInferenceComponent`. HailoRT runs YOLO piece detection on the Hailo-8/8L NPU, computes perspective transformation matrices, maps detected bounding boxes into board coordinates, and publishes the Forsyth-Edwards Notation (FEN) string to `/hailo_chess_inference/fen` ([`lekiwi_perception/src/hailo/chess_vision_mapper.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/hailo/chess_vision_mapper.cpp)).

3. **Task & State Orchestration**:
   `TaskOrchestratorNode` maintains the system state machine with 4 states: `STANDBY` (0), `NAVIGATING` (1), `CHESS_THINKING` (2), and `MANIPULATION_LEROBOT` (3). It bootstraps lifecycle transitions of perception nodes, validates requested state transitions against `ALLOWED_TRANSITIONS`, and publishes the latched `/camera_mode` topic ([`lekiwi_control/lekiwi_control/task_orchestrator.py:178-234`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py#L178-L234)).

4. **Trajectory & Arm Translation**:
   In `MANIPULATION_LEROBOT` mode, `LeRobotArmBridge` receives raw arm joint positions from LeRobot policies, translates them into radian trajectory points based on calibrated midpoints in `lekiwi_arm_calib.yaml`, and sends action goals to `/arm_controller/follow_joint_trajectory` ([`lekiwi_control/lekiwi_control/lerobot_arm_bridge.py:121-141`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/lerobot_arm_bridge.py#L121-L141)).

5. **Teleoperation Leader-Follower Mapping**:
   `teleop_uarm_node` polls the unpowered leader arm servos via `zhongli_protocol` over `/dev/uarm_leader` at 50 Hz. The `KinematicRemapper` maps raw leader angles through calibration tables defined in `uarm_teleop_calib.yaml`, accounts for offset, direction, scale, and limits, and dispatches trajectory commands directly to the follower `arm_trajectory_controller` or `arm_forward_controller` ([`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp)).

6. **Motion Execution & Async Serial Driver**:
   `arm_controller` (JointTrajectoryController) and `omni_base_controller` (OmniWheelDriveController) pass position and velocity targets to `LeKiwiFeetechHardwareInterface`. The hardware interface writes targets into a shared mutex-protected command buffer and returns immediately (< 10 µs). The background `io_worker_thread_` reads the buffer, issues synchronous write/read packet sequences over the 1 Mbps serial line (`/dev/lekiwi_serial`), and updates joint telemetry ([`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)).

7. **IMU Sensor Pipeline & Diagnostics**:
   `ICM20948SensorHardware` reads 9-DoF IMU data over I2C (`/dev/i2c-1`), broadcasted by `imu_sensor_broadcaster` and filtered by `imu_filter_madgwick` ([`lekiwi_bringup/launch/imu.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/imu.launch.py)). Meanwhile, `diagnostic_aggregator` monitors hardware states and CPU/RAM/Disk metrics ([`lekiwi_bringup/launch/diagnostics.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/diagnostics.launch.py)).

---

### 3) Layer/Module Responsibilities

| Layer or module | Owns | Must not own | Evidence |
|-----------------|------|--------------|----------|
| `lekiwi_interfaces` | Message and service definitions for modes, statuses, telemetry | Implementation logic, node executables | [`lekiwi_interfaces/package.xml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_interfaces/package.xml) |
| `teleop_zhongli_servo_hw` | Zhongli leader arm protocol, calibration CLI, kinematic remapping, follower dispatching | Vision inference, robot URDF, mobile base kinematics | [`teleop_zhongli_servo_hw/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/CMakeLists.txt) |
| `lekiwi_perception` | Camera acquisition, GStreamer pipelines, HailoRT neural inference, chessboard mapping, visual debugging | Motor commands, trajectory planning, system FSM logic | [`lekiwi_perception/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/CMakeLists.txt) |
| `lekiwi_control` | Task orchestration FSM, lifecycle coordination, LeRobot trajectory bridging | Low-level serial communication, camera frame decoding | [`lekiwi_control/CMakeLists.txt`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/CMakeLists.txt) |
| `lekiwi_ftservo_hardware` | Feetech STS protocol implementation, async serial I/O, ros2_control SystemInterface | AI inference, image processing, high-level task logic | [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp) |
| `lekiwi_icm20948_hardware` | ICM-20948 I2C register configuration, gyro calibration, ros2_control SensorInterface | Motor torque control, trajectory execution | [`lekiwi_icm20948_hardware/include/lekiwi_icm20948_hardware/icm20948_sensor_hardware.hpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/include/lekiwi_icm20948_hardware/icm20948_sensor_hardware.hpp) |
| `lekiwi_navigation` | Nav2 behavior tree configurations, costmaps, planner and controller plugins | Hardware serial drivers, NPU model weights | [`lekiwi_navigation/README.md`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_navigation/README.md) |
| `lekiwi_description` | Robot kinematics, joint limits, CAD mesh assets, ros2_control Xacro wiring | Node binaries, hardware serial I/O | [`lekiwi_description/urdf/lekiwi_robot.urdf.xacro`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_description/urdf/lekiwi_robot.urdf.xacro) |
| `lekiwi_bringup` | Subsystem compositions, launch arguments, parameter files, udev rules | Algorithmic logic, custom messages | [`lekiwi_bringup/launch/robot.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py) |

---

### 4) Reused Patterns

| Pattern | Where found | Why it exists |
|---------|-------------|---------------|
| **Composed Components** (`rclcpp_components`) | [`lekiwi_perception/src/camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp) | Allows multiple camera drivers and inference nodes to run inside a single process container (`component_container_mt`), sharing image pointers with zero IPC serialization overhead. |
| **Lifecycle State Machine** (`rclcpp_lifecycle`) | [`lekiwi_perception/include/hailo_chess_inference_component.hpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/include/hailo_chess_inference_component.hpp), [`lekiwi_perception/include/camera_streamer_component.hpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/include/camera_streamer_component.hpp) | Guarantees deterministic node startup, hardware initialization, memory allocation, and graceful teardown. |
| **Double-Buffered Async I/O Worker** | [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp:69-89`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp#L69-L89) | Prevents the 50 Hz real-time controller loop from blocking on serial UART delays (~15-20 ms roundtrip for 9 servos). `read()` and `write()` access mutexed memory structures in < 10 µs. |
| **Dynamic GStreamer Valve Gating** | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:17`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L17), [`lekiwi_perception/src/camera_streamer_component.cpp:115-135`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp#L115-L135) | Dynamically drops video buffers at the source when cameras are inactive, saving CPU/memory bandwidth on the embedded host. |
| **Hardware Plugin Abstraction** (`pluginlib`) | [`lekiwi_ftservo_hardware/lekiwi_ftservo_hardware_plugin.xml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/lekiwi_ftservo_hardware_plugin.xml), [`lekiwi_icm20948_hardware/lekiwi_icm20948_hardware_plugin.xml`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/lekiwi_icm20948_hardware_plugin.xml) | Allows switching transparently between mock components (`mock_components/GenericSystem`) and real hardware drivers via launch arguments (`hardware_type:=mock/real`). |
| **Interactive Midpoint Calibration CLI** | [`teleop_zhongli_servo_hw/src/calibrate_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/calibrate_uarm_node.cpp) | Guides user through physical zero-locking and range of motion measurement, generating pure YAML without manual calculation errors. |

---

### 5) Known Architectural Risks

- **Multiple Serial Bus Management**: Robot utilizes two independent serial buses: `/dev/lekiwi_serial` (follower arm + omni base) and `/dev/uarm_leader` (teleop leader arm). Explicit udev symlinks by USB vendor/product and serial numbers are essential to prevent port swap upon boot.
- **Hailo NPU Virtual Device Sharing**: `HailoChessInferenceComponent` utilizes virtual device group IDs (`vdevice_group_id: lekiwi_chess`). Running concurrent deep learning networks on HailoRT must be coordinated to prevent device resource starvation.

---

### 6) Evidence

- [`lekiwi_bringup/launch/robot.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_bringup/launch/teleop_uarm.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/teleop_uarm.launch.py)
- [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp)
- [`lekiwi_perception/src/camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp)
- [`lekiwi_perception/src/hailo_chess_inference_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/hailo_chess_inference_component.cpp)
- [`lekiwi_control/lekiwi_control/task_orchestrator.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py)
- [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)
- [`lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_icm20948_hardware/src/icm20948_sensor_hardware.cpp)

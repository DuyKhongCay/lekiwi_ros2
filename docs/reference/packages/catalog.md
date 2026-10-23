# Packages Catalog

The LeKiwi ROS 2 workspace is organized into 15 active packages across 6 functional domains:

---

## 1. Core Orchestration & Game Decision

| Package | Language | Build Tool | Description |
| :--- | :---: | :---: | :--- |
| `lekiwi_orchestrator` | Python (`rclpy`) | `ament_python` | High-level autonomous chess mission orchestrator FSM, readiness-gated navigation startup, and dynamic camera mode switching. |
| `lekiwi_chess_master` | C++ | `ament_cmake` | Chess game state tracking, FEN string generation, and Stockfish 16 action server implementing `ComputeBestMove.action`. |

---

## 2. Motion Planning & Physical-AI Policy Serving

| Package | Language | Build Tool | Description |
| :--- | :---: | :---: | :--- |
| `lekiwi_motion` | C++ | `ament_cmake` | Kinematics feasibility checker, workspace reachability validation, system readiness gating, and torque management state. |
| `lerobot_policy_server` | Python / ZeroMQ | `setuptools` | High-throughput ZeroMQ inference server bridging LeRobot physical-AI policies (ACT, Diffusion, SmolVLA) with ROS 2. |
| `lekiwi_inference` | Python / C++ | `ament_cmake` | Client runtime for remote asynchronous policy inference (Phase 1) and onboard Hailo-8 NPU execution (Phase 3). |

---

## 3. Perception & Sensor Hub

| Package | Language | Build Tool | Description |
| :--- | :---: | :---: | :--- |
| `lekiwi_perception` | C++ Components | `ament_cmake` | Lifecycle-managed dual CSI camera hub, dynamic GStreamer valve control, Hailo-8/8L NPU YOLO inference, and chessboard detection. |

---

## 4. Hardware Abstraction & Teleoperation

| Package | Language | Build Tool | Description |
| :--- | :---: | :---: | :--- |
| `lekiwi_ftservo_hardware` | C++ `ros2_control` | `ament_cmake` | `SystemInterface` hardware plugin for 9 Feetech STS3215 servos over a 1 Mbps serial bus with dedicated asynchronous I/O thread. |
| `lekiwi_icm20948_hardware` | C++ `ros2_control` | `ament_cmake` | `SensorInterface` hardware plugin for ICM-20948 9-DoF IMU communicating over hardware I2C (`/dev/i2c-1`). |
| `teleop_zhongli_servo_hw` | C++ Driver | `ament_cmake` | Dedicated protocol driver, calibration CLI, and joint publisher for Zhongli / uArm leader teleoperation arms. |

---

## 5. System Composition, Kinematics & Interfaces

| Package | Language | Build Tool | Description |
| :--- | :---: | :---: | :--- |
| `lekiwi_interfaces` | Custom IDL | `ament_cmake` | Custom ROS 2 Actions (`ComputeBestMove`, `ExecuteChessMove`), Services (`CheckMoveFeasibility`, `ExecuteCollectionTask`), and Messages. |
| `lekiwi_description` | URDF / Xacro | `ament_cmake` | Kinematic robot description, CAD STL meshes, joint limits, transmissions, and `ros2_control` hardware tags. |
| `lekiwi_bringup` | Launch & Config | `ament_cmake` | Master launch compositions (`robot.launch.py`), controller parameters, udev rules, and diagnostic monitors. |

---

## 6. Calibration & Physical-AI Dataset Pipeline

| Package | Language | Build Tool | Description |
| :--- | :---: | :---: | :--- |
| `lekiwi_calibration` | Python / OpenCV | `ament_python` | Unified calibration suite: Chessboard AprilTag bundle adjustment, Hand-Eye calibration, and Omni Base kinematics tuning. |
| `chess_episode_recorder` | C++ / ROS 2 | `ament_cmake` | Real-time episode recorder for LeKiwi Chess Physical-AI with Gamepad D-Pad workflow and lossless MCAP storage. |
| `chess_rosbag_to_lerobot` | Python / LeRobot | `ament_python` | Converter pipeline transforming LeKiwi MCAP rosbags into standard HuggingFace LeRobot v3.0 dataset formats. |


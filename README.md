# LeKiwi ROS 2: Autonomous Mobile Manipulator & Physical-AI Platform

<div align="center">

[![English](https://img.shields.io/badge/Language-English-blue.svg)](README.md)
[![Tiếng Việt](https://img.shields.io/badge/Ngôn_ngữ-Tiếng_Việt-red.svg)](README.vi.md)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy%20%7C%20Humble-22314E.svg?logo=ros)](https://docs.ros.org/)
[![Hardware](https://img.shields.io/badge/Hardware-Raspberry%20Pi%205-C51A4A.svg?logo=raspberrypi)](https://www.raspberrypi.com/)
[![HailoRT](https://img.shields.io/badge/NPU-Hailo--8%2F8L%2026TOPS-00A651.svg)](https://hailo.ai/)
[![Physical-AI](https://img.shields.io/badge/Physical--AI-LeRobot%20v3.0-FF6F00.svg)](https://github.com/huggingface/lerobot)
[![Middleware](https://img.shields.io/badge/Middleware-Zenoh%20RMW-black.svg)](https://zenoh.io/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

**[English](README.md)** | **[Tiếng Việt](README.vi.md)**

<p align="center">
  <em>An open-source desktop autonomous mobile manipulator integrating holonomic omni-drive, 6-DoF precision manipulation, real-time edge AI perception, autonomous chess play, and state-of-the-art imitation learning.</em>
</p>

</div>

---

> [!IMPORTANT]
> ### 📚 Comprehensive Technical Documentation & Wiki Portal
> This README provides a high-level showcase, project vision, and quick-start entrypoints. For the complete engineering handbook — including **System Architecture deep-dives, 15 Active Packages catalog, custom ROS 2 Action/Service contracts, hardware pinout specs, calibration workflows, and distributed container setup** — visit our project Wiki:
> 
> 👉 **[Explore the LeKiwi Project GitHub Wiki](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)**

---

## 🌟 Project Vision & Highlights

**LeKiwi** is an integrated autonomous mobile manipulation robotics platform designed to bridge physical robotics with modern **Physical-AI (Imitation Learning & VLA policies)** and **High-Performance Edge AI**. Built around a 3-wheel holonomic omnidirectional chassis and a 6-DoF serial arm, LeKiwi demonstrates complex real-world interaction through autonomous chess gameplay against human players.

```
       [ Dual CSI Cameras ]                 [ 9x Feetech STS3215 Servos ]
                │                                         │
                ▼                                         ▼
   ┌───────────────────────────┐             ┌───────────────────────────┐
   │    Hailo-8 M.2 NPU HAT    │             │   ros2_control Hardware   │
   │  Stereo Vision & 26 TOPS  │             │ 1 Mbps Async Serial Bus   │
   └─────────────┬─────────────┘             └─────────────▲─────────────┘
                 │                                         │
                 ▼                                         │
   ┌───────────────────────────────────────────────────────┴─────────────┐
   │            LeKiwi Autonomous Orchestrator & Mission FSM             │
   │      Stockfish 16 Engine  ◄──►  ZeroMQ LeRobot Policy Server        │
   └─────────────────────────────────────────────────────────────────────┘
```

### Core Capabilities
* **♟️ Autonomous Game Intelligence:** End-to-end autonomous chess playing powered by the Stockfish 16 chess engine, dynamic FSM mission orchestration, and visual board state verification.
* **⚡ Accelerated Edge Perception:** Real-time dual CSI stereo vision and YOLO neural inference running at 30+ FPS via the Raspberry Pi 5 Hailo-8 M.2 NPU HAT+.
* **🤖 Precision Manipulation & Control:** 6-DoF robotic arm driven by 9 smart Feetech STS3215 bus servos via an asynchronous `ros2_control` hardware abstraction layer at 1 Mbps.
* **🦾 Physical-AI Imitation Learning:** Native streaming bridge connecting LeRobot policy models (ACT, Diffusion Policy, SmolVLA) with ROS 2 via ZeroMQ binary transport.
* **🌐 Zero-Multicast Distributed Networking:** Clean edge deployment on Raspberry Pi 5 with Docker and Zenoh middleware (`rmw_zenoh_cpp`), enabling instant remote RViz2 monitoring without Wi-Fi multicast packet loss.

---

## 📸 Feature Showcase: LeKiwi in Action

Below are the primary operating scenarios of LeKiwi. *(Photos, animated GIFs, and demonstration videos are being actively recorded and linked below)*.

### 1. 🤖 Physical Robot & 3-Wheel Omni Chassis
The platform features an agile 3-wheel holonomic base capable of omnidirectional translation and in-place rotation, combined with an onboard Raspberry Pi 5 brain and integrated power management.

| Robot Overview | Hardware Architecture |
| :---: | :---: |
| ![LeKiwi Physical Robot Overview](docs/images/showcase/lekiwi_robot_overview.png)<br>*(Placeholder: LeKiwi full robot standing on the test arena)* | ![Chassis & Arm Assembly](docs/images/showcase/lekiwi_chassis_arm.png)<br>*(Placeholder: 3-wheel omni base and 6-DoF arm assembly)* |

---

### 2. ♟️ Autonomous Chess Master & 6-DoF Pick-and-Place
LeKiwi detects chessboard squares, identifies opponent piece moves, calculates optimal counter-moves using Stockfish 16, and executes physical pick-and-place trajectories with workspace reachability validation.

| Visual Board Evaluation | Precision Pick-and-Place |
| :---: | :---: |
| ![Chessboard State Tracking](docs/images/showcase/chess_vision_evaluation.png)<br>*(Placeholder: YOLO board corner & FEN detection in action)* | ![Physical Move Execution](docs/images/showcase/chess_piece_manipulation.png)<br>*(Placeholder: Arm gripper picking and relocating a chess piece)* |

---

### 3. 👁️ Edge Vision & Hailo-8 NPU Acceleration
Equipped with dual IMX CSI cameras and the 26 TOPS Hailo-8 NPU, LeKiwi processes stereo vision, board corner estimation, and object detection with minimal CPU utilization.

| Stereo Camera Pipeline | Hailo NPU Inference Stream |
| :---: | :---: |
| ![Dual CSI Camera Streams](docs/images/showcase/stereo_camera_stream.png)<br>*(Placeholder: Real-time stereo camera stream & rectification)* | ![Hailo YOLO Detection](docs/images/showcase/hailo_yolo_detection.png)<br>*(Placeholder: 30 FPS neural detection bounding boxes)* |

---

### 4. 🦾 Teleoperation & Physical-AI Dataset Pipeline
Collect demonstration episodes via the Zhongli / uArm leader teleoperation arm or USB gamepad, record lossless MCAP rosbags, and convert them seamlessly into Hugging Face LeRobot datasets.

| Leader Arm Teleoperation | LeRobot Policy Serving |
| :---: | :---: |
| ![Bilateral Teleoperation Arm](docs/images/showcase/teleop_leader_arm.png)<br>*(Placeholder: Operator teaching arm trajectories via leader arm)* | ![Inference Policy Rollout](docs/images/showcase/lerobot_inference_rollout.png)<br>*(Placeholder: ZeroMQ autonomous policy execution)* |

---

## 🚀 Getting Started & Documentation Portal

> [!TIP]
> ### 🛠️ Ready to Build or Run LeKiwi?
> Complete step-by-step setup guides, toolchain installation, simulation setups, and distributed deployment walk-throughs have been organized on our **Project Wiki**:
> 
> * 🐳 **[Raspberry Pi 5 Docker Container Deployment](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Deploy the pre-packaged containerized stack with Hailo-8 and Zenoh drivers.
> * 💻 **[Native Host Build & Mock Simulation](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Set up `colcon` workspaces on Ubuntu 24.04/22.04 without physical hardware.
> * ♟️ **[Autonomous Chess Playing Mission Guide](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Launch the Stockfish FSM orchestrator and vision pipeline.
> * 🦾 **[LeRobot Physical-AI & Teleoperation Setup](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)** — Stream imitation learning policies and record MCAP episodes.
> * 🌐 **[Distributed PC Monitoring via Zenoh](docs/how-to-guides/deployment/distributed-pi5-docker-and-pc-zenoh-setup.md)** — Connect your remote workstation to monitor topics and view 3D RViz2 displays.
> 
> 👉 **[Explore Full Installation Guides on GitHub Wiki](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)**

---

## 🗺️ Project Roadmap & Milestones

The development of LeKiwi follows a staged progression from low-level hardware abstraction to autonomous manipulation and physical-AI intelligence:

- [x] **Phase 1: Hardware Abstraction & Omnidirectional Base**
  - [x] 3-wheel holonomic omnidirectional chassis kinematics and odometry.
  - [x] 9-DoF ICM-20948 IMU sensor integration over hardware I2C.
  - [x] Feetech STS3215 asynchronous serial bus driver operating at 1 Mbps with `ros2_control`.
  - [x] Dedicated persistent udev rules and hardware mapping.

- [x] **Phase 2: Edge Perception & Hardware Acceleration**
  - [x] Dual CSI stereo camera pipeline with dynamic GStreamer control.
  - [x] Raspberry Pi 5 Hailo-8 M.2 NPU HAT+ integration (26 TOPS).
  - [x] Real-time YOLO chessboard square & piece detection at 30+ FPS.
  - [x] Hand-Eye calibration and AprilTag bundle adjustment suite.

- [x] **Phase 3: Autonomous Chess Playing & Mission Orchestration**
  - [x] Stockfish 16 chess engine ROS 2 action server (`ComputeBestMove`).
  - [x] High-level Mission Orchestrator FSM with dynamic camera switching.
  - [x] 6-DoF Cartesian trajectory planning with workspace reachability validation.
  - [x] Complete autonomous human-robot chess game playing loop.

- [/] **Phase 4: Physical-AI & Imitation Learning (LeRobot)**
  - [x] ZeroMQ high-throughput streaming bridge between LeRobot policies and ROS 2.
  - [x] Gamepad and leader-arm MCAP episode recorder (`chess_episode_recorder`).
  - [x] Lossless dataset conversion into Hugging Face LeRobot v3.0 format.
  - [🟡] End-to-end evaluation of ACT, Diffusion Policy, and SmolVLA policies on real hardware.

- [ ] **Phase 5: Room-Scale Autonomous Navigation & Multi-Task VLA**
  - [ ] Nav2 integration with SLAM Toolbox and lidar/visual odometry for global navigation.
  - [ ] Voice-prompted mission dispatch and multi-task Vision-Language-Action (VLA) interaction.
  - [ ] Cloud-to-edge policy distribution and continuous demonstration learning.

---

## 🤝 Reference Projects & Acknowledgements

The LeKiwi ecosystem stands on the shoulders of remarkable open-source robotics, physical-AI, and edge-computing projects. We gratefully acknowledge the following reference repositories:

| Category | Reference Project | Role & Architectural Influence in LeKiwi |
| :--- | :--- | :--- |
| **1. Hardware & Base Design** | [`DuyKhongCay/lekiwi-robot`](https://github.com/DuyKhongCay/lekiwi-robot) | Physical CAD models, 3D printing STL files, mechanical bill of materials (BOM), and assembly specs. |
| | [`SIGRobotics-UIUC/LeKiwi`](https://github.com/SIGRobotics-UIUC/LeKiwi) | Foundational mechanical design and original concept of the LeKiwi omni-manipulator. |
| | [`legalaspro/feetech_ros2_driver`](https://github.com/legalaspro/feetech_ros2_driver) | Asynchronous serial bus communication driver for Feetech STS3215 smart servos. |
| | [`NTkot/icm20948_i2c`](https://github.com/NTkot/icm20948_i2c) | Low-level Linux I2C communication library for the ICM-20948 9-DoF IMU. |
| | [`MINT-SJTU/LeRobot-Anything-U-Arm`](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm) | Teleoperation leader arm communication protocol and joint remapping conventions. |
| **2. Physical-AI & Imitation Learning** | [`huggingface/lerobot`](https://github.com/huggingface/lerobot) | Core Physical-AI framework for training imitation learning policies (ACT, Diffusion, SmolVLA). |
| | [`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai) | Architectural pattern for high-performance ZeroMQ streaming bridges between ROS 2 and LeRobot. |
| | [`fiveages-sim/lerobot_ros2`](https://github.com/fiveages-sim/lerobot_ros2) | ROS 2 simulation integration and teleoperation workflow references. |
| **3. Edge Vision & Acceleration** | [`hailo-ai/hailo-apps`](https://github.com/hailo-ai/hailo-apps) | Hailo-8 M.2 hardware acceleration pipelines and GStreamer inference plugin architectures. |
| | [`ros-drivers/gscam`](https://github.com/ros-drivers/gscam) | GStreamer camera pipeline driver bridging v4l2/CSI video feeds into ROS 2 Image topics. |
| **4. Chess Logic & Game Intelligence** | [`Disservin/chess-library`](https://github.com/Disservin/chess-library) | High-performance header-only C++ chess library for board representation and move generation. |
| | [`wojciechc1/AI_Chess`](https://github.com/wojciechc1/AI_Chess) | Visual chessboard detection and computer vision state validation concepts. |
| **5. Engineering Skills & Tooling** | [`dbwls99706/ros2-engineering-skills`](https://github.com/dbwls99706/ros2-engineering-skills) | Production engineering guidelines, executor architectures, and ROS 2 lifecycle design patterns. |
| | [`github/awesome-copilot`](https://github.com/github/awesome-copilot) | Curated agent prompts, instructions, skills, and AI pair-programming workflows. |

> [!NOTE]
> **Community Gratitude:** In addition to the repositories listed above, LeKiwi builds upon countless academic publications, ROS 2 community contributions, Hugging Face open models, and Linux kernel drivers that cannot all be listed here. We extend our sincere thanks to the global robotics and open-source physical-AI communities!

---

> [!NOTE]
> ### 🤖 AI-Assisted Documentation Notice
> This repository documentation was authored and structured with the assistance of advanced AI engineering agents. While every section undergoes cross-verification against the codebase, automated documentation may occasionally contain discrepancies. If you spot any error, broken link, or outdated instruction, please [open a GitHub Issue](https://github.com/DuyKhongCay/lekiwi_ros2/issues) to help maintainers and contributors keep the project accurate!

---

## 📜 License & Maintainers

* **License:** Distributed under the [Apache-2.0 License](LICENSE).
* **Project Maintainer:** **DuyKhongCay** ([@DuyKhongCay](https://github.com/DuyKhongCay))
* **Wiki & Documentation:** [https://github.com/DuyKhongCay/lekiwi_ros2/wiki](https://github.com/DuyKhongCay/lekiwi_ros2/wiki)

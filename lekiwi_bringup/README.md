# lekiwi_bringup

Top-level robot launch orchestrations, multi-subsystem lifecycles, and deployment configurations for the LeKiwi autonomous mobile manipulator.

---

## Overview

The `lekiwi_bringup` package serves as the centralized entry point for bootstrapping and configuring the physical or simulated LeKiwi robot. It orchestrates the lifecycle transitions of distributed hardware drivers, GStreamer dual/triple camera streaming, EKF odometry fusion, ros2_control controllers, Nav2 mobile base navigation, Stockfish chess decision-making, and high-level mission supervision.

```
                         [ robot.launch.py ]
                                  │
    ┌──────────────┬──────────────┼──────────────┬──────────────┐
    ▼              ▼              ▼              ▼              ▼
[controllers]  [cameras]   [localization]  [navigation]   [orchestrator]
 (ros2_control) (GStreamer)   (Dual EKF)     (Nav2 Omni)    (System FSM)
    │              │              │              │              │
    ▼              ▼              ▼              ▼              ▼
[Hardware]    [Hailo NPU]     [IMU / Odom]  [Costmaps]     [Stockfish /
(STS3215/IMU) (Perception)    (/tf tree)    (Trajectory)   Manipulation]
```

---

## Subsystem Launch Architecture

The bringup stack is composed of modular, independently launchable launch descriptions located in `launch/`:

| Launch File | Responsibility | Core Nodes Spawned |
| :--- | :--- | :--- |
| `robot.launch.py` | Top-level master bringup coordinating all subsystems. | Spawns description, controllers, perception, localization, navigation, and orchestrator. |
| `controllers.launch.py` | ros2_control controller manager, hardware interfaces, and torque safety. | `ros2_control_node`, `spawner` (arm, base, IMU), `torque_manager_node`. |
| `orchestrator.launch.py` | Mission supervision, workspace reachability, and game FSM. | `system_readiness_node`, `workspace_checker`, `chess_mission_orchestrator`. |
| `cameras.launch.py` | Multi-camera acquisition and GStreamer pipeline lifecycle management. | `camera_streamer_component` (wrist, side, stereo left/right). |
| `localization.launch.py` | Coordinate frame fusion and dual-layer Extended Kalman Filtering. | `ekf_filter_node` (local `odom` & global `map`), `imu_transform_node`. |
| `navigation.launch.py` | Nav2 autonomous mobile base navigation for holonomic 3-wheel base. | `controller_server`, `planner_server`, `bt_navigator`, `costmap_2d`. |
| `chess_master.launch.py` | Stockfish 16 engine action server, FEN tracker, and 2D visualizer. | `stockfish_driver`, `game_state_tracker`, `chess_2d_visualizer`. |
| `teleop_gamepad.launch.py` | Joystick manual teleoperation for mobile base navigation and testing. | `joy_node`, `teleop_twist_joy`. |
| `diagnostics.launch.py` | System-wide health aggregation and hardware telemetry monitoring. | `diagnostic_aggregator`. |

---

## Launch Arguments Reference (`robot.launch.py`)

Inspect available parameters at runtime:
```bash
ros2 launch lekiwi_bringup robot.launch.py --show-args
```

| Argument Name | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `hardware_type` | `string` | `"real"` | Target hardware interface: `"real"` (Feetech STS & ICM20948) or `"mock"` (simulated). |
| `manipulation` | `string` | `"kinematics"` | Manipulation paradigm: `"kinematics"` (analytical IK, accel=0), `"policy"` (LeRobot inference, accel=50), or `"false"` (disabled). |
| `use_sim_time` | `bool` | `false` | Synchronizes all nodes to `/clock` when running in simulation. |
| `orchestrator` | `bool` | `true` | Starts the system readiness monitor and chess mission supervisor. |
| `cameras` | `bool` | `true` | Starts GStreamer camera streamers and Hailo perception pipelines. |
| `localization` | `bool` | `true` | Starts dual-instance robot_localization EKF odometry fusion. |
| `navigation` | `bool` | `true` | Starts Nav2 autonomous navigation stack for the 3-wheel omni base. |
| `chess_master` | `bool` | `true` | Starts Stockfish 16 action server and match tracking nodes. |
| `record` | `bool` | `false` | Enables episode recording mode during manipulation actions. |
| `calib_mode` | `bool` | `false` | Enables raw uncompressed camera streams for hand-eye calibration. |

---

## Configuration Reference

Centralized parameters are declared in `config/` and passed directly to nodes:

* **`config/control/controllers.yaml`**: `ros2_control` controller parameters, joint interfaces, velocity limits, and `torque_manager` safety thresholds.
* **`config/control/orchestrator.yaml`**: Coordinate frame definitions (`map`, `odom`, `base_footprint`, `chessboard_frame`), reachability margins, named landmark poses (`approach`, `stow`, `clear_left`, `clear_right`), and system readiness tolerances. See [lekiwi_motion documentation](../lekiwi_motion/README.md) for motion planning contracts.
* **`config/localization/ekf_local.yaml` & `ekf_global.yaml`**: Odometry sensor fusion matrices for wheel odometry and IMU.
* **`config/navigation/nav2_params.yaml`**: Costmaps, Smac Planner, and DWB controller tuned for holonomic 120° omni chassis.

---

## Quick Start & Verification

### 1. Mock Hardware Verification (Desktop / CI)

Verify system graph and topic resolution without physical hardware:

```bash
# Launch entire stack in mock hardware mode
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=mock \
  manipulation:=kinematics
```

### 2. Physical Robot Deployment (Raspberry Pi 5)

```bash
# Full physical deployment on robot arena
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=real \
  manipulation:=kinematics \
  record:=false
```

### 3. Subsystem Health & Diagnostics

Inspect aggregated diagnostics and topic connectivity:

```bash
# Verify active controller states
ros2 control list_controllers

# Monitor real-time system health
ros2 topic echo /diagnostics | grep -A 5 "Hardware Status"
```

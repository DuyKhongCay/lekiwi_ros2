---
post_title: 'teleop_zhongli_servo_hw'
author1: 'duykhongcay'
post_slug: 'teleop-zhongli-servo-hw'
microsoft_alias: 'duykhongcay'
featured_image: ''
categories:
  - robotics
tags:
  - ros2_control
  - uarm
  - zhongli
  - teleoperation
  - urdf
ai_note: 'Documentation updated following technical writer standards'
summary: 'High-performance ROS 2 Leader Arm Teleoperation driver with dynamic URDF limit ingestion for uArm and Zhongli serial bus servos'
post_date: '2026-10-12'
---

## Overview

The `teleop_zhongli_servo_hw` package provides a high-performance, real-time ROS 2 teleoperation driver for the uArm Leader Arm powered by Zhongli ASCII serial bus servos. It maps human operator arm motions to a follower robot arm (such as the LeKiwi mobile manipulator or SO-ARM100) with bounded latency, wait-free execution, and dynamic kinematic limit calculation directly from the robot URDF model.

The driver communicates with the leader arm servos over a half-duplex UART bus at a strictly enforced **115,200 baud**, decoupling serial bus I/O from the ROS 2 executor loop via an asynchronous background thread and a wait-free `realtime_tools::RealtimeBuffer` (< 50 ns access time).

---

## Architecture and Concurrency Model

```text
                    +-------------------------------------------------------+
                    |           Human Operator Manual Arm Motion            |
                    |           (uArm / Zhongli Serial Bus Servos)          |
                    +---------------------------+---------------------------+
                                                | 115,200 Baud (RS-232/TTL)
                    +---------------------------v---------------------------+
                    |                   ZhongliBusWorker                    |
                    |       (50 Hz Background Asynchronous Thread)          |
                    |   - Sequential #PRAD! Position Polling (50 Hz)        |
                    |   - Interleaved Round-Robin #PRTV! Telemetry (1 Hz)   |
                    |   - Forward Kinematics (Servo PWM -> Joint Radian)    |
                    |   - 3-Tier Fault Escalation & Buffer Flush            |
                    +---------------------------+---------------------------+
                                                | Wait-free Write (< 50 ns)
                    +---------------------------v---------------------------+
                    |             realtime_tools::RealtimeBuffer            |
                    |             (Triple-Buffered Kinematic State)         |
                    +---------------------------+---------------------------+
                                                | Wait-free Read (< 50 ns)
                    +---------------------------v---------------------------+
                    |                    TeleopUarmNode                     |
                    |             (50 Hz ROS 2 Periodic Timer)              |
                    |   - Publishes: /leader/joint_states (SensorDataQoS)   |
                    |   - Subscribes: /robot_description (TRANSIENT_LOCAL)  |
                    |   - Dynamic Follower Joint Limit Parsing (urdf::Model)|
                    +---------------------------+---------------------------+
                                                | Motion-Safety Interlock
                    +---------------------------v---------------------------+
                    |                  FollowerDispatcher                   |
                    |   - JointTrajectoryController (/joint_trajectory)     |
                    |   - ForwardCommandController (/commands)              |
                    |   - Anti-Jerk First-Order Low-Pass Filter (LPF)       |
                    +-------------------------------------------------------+
```

### Key Technical Characteristics

- **Standard 115,200 Baud Protocol**: Operates strictly at 115,200 bps with an internal 50 µs inter-command spacing and 70 ms read timeout, maintaining rock-solid compatibility with standard USB-UART bridges (PL2303, CH340, CP2102) without baud rate negotiation overhead.
- **Asynchronous Decoupled I/O (`ZhongliBusWorker`)**: Serial transactions are executed on an isolated POSIX thread paced by `std::this_thread::sleep_until` (50 Hz). The ROS 2 publishing loop never blocks on UART transactions.
- **Wait-Free Real-Time Buffer (`RealtimeBuffer`)**: The ROS 2 loop retrieves fresh kinematic states in $< 50\text{ ns}$ with zero lock contention.
- **Dynamic URDF Kinematic Ingestion**: Joint limits (`lower`, `upper`) and motion ranges for the follower arm are ingested dynamically from `/robot_description`, eliminating brittle, hardcoded follower calibration values.
- **Motion-Safety Interlock (Rule 12)**: The driver publishes `/leader/joint_states` for logging and monitoring, but strictly halts follower command dispatching until verified URDF limits arrive.
- **Zero Heap Allocation in Hot Path**: Scratch vectors and message payloads are pre-allocated during initialization (MISRA C++ Rule 18-0-1 compliance).

---

## Dynamic URDF Ingestion and Kinematic Remapping

### 1. Ingestion via `robot_description`

The node automatically subscribes to `/robot_description` published by `robot_state_publisher` (from `description.launch.py`) with `TRANSIENT_LOCAL` durability and `RELIABLE` reliability QoS:

```cpp
const auto qos = rclcpp::QoS(1).transient_local().reliable();
robot_description_sub_ = create_subscription<std_msgs::msg::String>(
    "robot_description", qos,
    std::bind(&TeleopUarmNode::on_robot_description, this, std::placeholders::_1));
```

The node parses the URDF XML using `urdf::Model`, extracts joint limit spans ($[f_{\min}, f_{\max}]$), and computes remapping parameters dynamically.

### 2. Kinematic Remapping Strategies

The driver transforms leader joint positions $\theta_L$ to follower commands $\theta_F$ using four selectable modes:

| Mode | Target Joints | Mathematical Formula | Purpose |
| :--- | :--- | :--- | :--- |
| `DIRECT` | `arm_shoulder_pan` | $\theta_F = \text{clamp}(\theta_L, f_{\min}, f_{\max})$ | Direct 1:1 angular correspondence. |
| `MIN_MAX` | `arm_gripper` | $\theta_F = f_{\min} + \left(\frac{\theta_L - l_{\min}}{l_{\max} - l_{\min}}\right) (f_{\max} - f_{\min})$ | Normalizes $0\%\dots 100\%$ stroke regardless of actuator span differences. |
| `SCALE_OFFSET` | `arm_shoulder_lift`, `arm_elbow_flex`, `arm_wrist_flex`, `arm_wrist_roll` | $\theta_F = \text{clamp}(k \cdot \theta_L + b, f_{\min}, f_{\max})$, where $k = \frac{\Delta f}{\Delta l}$ | Proportional range matching preserving zero-offset alignment. |
| `CENTERED_SCALE` | User configurable | Separate scaling for $\theta_L > 0$ and $\theta_L < 0$ | Preserves exact $0.0\text{ rad}$ home posture anchor while accommodating asymmetric limits. |

---

## ROS 2 Topics and Parameters

### Exported Topics

| Topic | Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/leader/joint_states` | `sensor_msgs/msg/JointState` | `SensorDataQoS` (Best Effort, depth 5) | Leader arm joint positions published at 50 Hz. |
| `/arm_trajectory_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Reliable | Follower command stream in `joint_trajectory` mode. |
| `/arm_forward_controller/commands` | `std_msgs/msg/Float64MultiArray` | Reliable | Follower command stream in `forward_command` mode. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable | Hardware health, serial status, and temperature telemetry. |

### Subscribed Topics

| Topic | Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/robot_description` | `std_msgs/msg/String` | `Transient Local`, `Reliable` | Follower robot URDF model for dynamic joint limit ingestion. |

### Node Parameters (`teleop_uarm_node`)

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | string | `"/dev/uarm_leader"` | Serial TTY port path for leader arm. |
| `baudrate` | int | `115200` | Serial baud rate (strictly enforced to 115200). |
| `publish_rate_hz` | double | `50.0` | Main teleoperation loop and publish frequency in Hz. |
| `leader_topic` | string | `"/leader/joint_states"` | Output topic for leader joint states. |
| `calibration_file` | string | `""` | Path to physical leader calibration YAML (`uarm_teleop_calib.yaml`). |
| `arm_mode` | string | `"joint_trajectory"` | Follower dispatch mode: `"joint_trajectory"`, `"forward_command"`, or `"joint_states_only"`. |
| `point_dt_s` | double | `0.04` | Time delta between trajectory waypoints in seconds. |
| `lpf_alpha` | double | `1.0` | Command smoothing filter factor ($1.0 = \text{no filtering}$, $0.2 = \text{aggressive smoothing}$). |
| `joint_remap_modes` | string[] | `[]` | Remap mode overrides (e.g. `["arm_gripper=min_max", "arm_shoulder_pan=direct"]`). |

---

## Safety Mechanisms and Fault Handling

1. **Motion-Safety Interlock (Rule 12)**:
   - If `/robot_description` is unavailable at startup, the node issues throttled warnings (`RCLCPP_WARN_THROTTLE`), continues publishing `/leader/joint_states`, but **suspends all command dispatching** to the follower arm until valid limits arrive.
2. **Anti-Jerk Low-Pass Filtering & Clamping**:
   - Every joint command is strictly bounded by $[f_{\min}, f_{\max}]$ before transmission.
   - An optional first-order low-pass filter ($\alpha$) prevents sharp operator jerks from causing excessive follower joint accelerations.
3. **3-Tier Fault Escalation**:
   - **Tier 1 (Single read timeout)**: Flushes serial input FIFO buffer to clear partial or corrupted ASCII frames.
   - **Tier 2 (5 consecutive errors / 100 ms)**: Marks the real-time kinematic snapshot invalid (`valid = false`), preventing upstream nodes from consuming stale data.
   - **Tier 3 (100 consecutive errors / 2.0 s)**: Gracefully closes and re-opens the serial port to recover from USB-UART hardware disconnects or driver hangs.

---

## Diagnostics and Health Telemetry

Telemetry is evaluated by `diagnostic_updater` and published to `/diagnostics` under hardware ID `uarm_leader_zhongli`:

| Metric | Normal Range | Warning (WARN) | Critical (ERROR) |
| :--- | :--- | :--- | :--- |
| Serial Connection | Connected | Intermittent timeout errors | Worker disconnected |
| Servo Temperature | $< 60\,^\circ\text{C}$ | $60\,^\circ\text{C}\dots 74\,^\circ\text{C}$ | $\ge 75\,^\circ\text{C}$ (Overheat) |
| Bus Voltage | $6.0\,\text{V}\dots 12.0\,\text{V}$ | $< 6.0\,\text{V}$ | $< 5.0\,\text{V}$ (Undervoltage) |
| Publish Frequency | $50.0 \pm 10\,\text{Hz}$ | Deviation $> 10\,\text{Hz}$ | Worker disconnected (0 Hz) |

---

## Build and Testing

### Prerequisites

Install system dependencies:
```bash
sudo apt-get update && sudo apt-get install -y libserial-dev
```

### Compilation

Build the package using `colcon`:
```bash
colcon build --packages-select teleop_zhongli_servo_hw --symlink-install
```

### Unit Tests

Run the GoogleTest suite:
```bash
colcon test --packages-select teleop_zhongli_servo_hw --event-handlers console_direct+
colcon test-result --verbose
```

### Usage

1. **Calibrate Leader Arm (Zero Lock & Range of Motion)**:
```bash
ros2 run teleop_zhongli_servo_hw calibrate_uarm_node --port /dev/uarm_leader --baud 115200 --out uarm_teleop_calib.yaml
```

2. **Launch Teleoperation Node**:
```bash
ros2 launch teleop_zhongli_servo_hw teleop_uarm.launch.py
```

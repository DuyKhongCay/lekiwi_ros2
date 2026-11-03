# lekiwi_inference

Physical-AI policy inference and real-time streaming bridge connecting Hugging Face **LeRobot** (ACT, Diffusion Policy, SmolVLA) with ROS 2 and `ros2_control`.

---

## Overview

The `lekiwi_inference` package provides an asynchronous, low-latency execution bridge for Physical-AI policies on the LeKiwi mobile manipulator robot. It decouples high-level visual-language-action (VLA) neural policy inference from high-rate real-time joint actuation using a two-tier cognitive/actuation timing architecture:

* **Cognition Loop (25 Hz)**: Synchronizes multi-view camera observations (wrist, side, overhead) with arm joint feedback, evaluates delta-observation filtering, and dispatches asynchronous inference requests to the LeRobot policy server over ZeroMQ.
* **Actuation Loop (50 Hz)**: Drains the Real-Time Chunking (RTC) action queue, applies Exponential Moving Average (EMA) joint smoothing, enforces velocity and soft deadband bounds, and streams forward position commands to `ros2_control`.

```
    [ Dual/Triple Cameras ]       [ Joint States ]
              │                           │
              ▼                           ▼
    ┌───────────────────────────────────────────┐
    │          Observation Packaging            │
    │     (SmolVLA State + Spatial Context)     │
    └─────────────────────┬─────────────────────┘
                          │ 25 Hz Cognition Loop
                          ▼
    ┌───────────────────────────────────────────┐         ZeroMQ REQ/REP
    │         Async ZeroMQ Client               │ ◄────────────────────────►  [ LeRobot Policy Server ]
    │      (MessagePack Transport Layer)        │                              (ACT / SmolVLA Policy)
    └─────────────────────┬─────────────────────┘
                          │ Action Chunk (30 Steps)
                          ▼
    ┌───────────────────────────────────────────┐
    │     RTC Action Queue & Joint Smoother     │
    │   (Weighted Aggregation + EMA Velocity)   │
    └─────────────────────┬─────────────────────┘
                          │ 50 Hz Actuation Loop
                          ▼
    ┌───────────────────────────────────────────┐
    │       ros2_control Hardware Layer         │
    │   (/arm_forward_controller/commands)      │
    └───────────────────────────────────────────┘
```

---

## Core Features

- **Asynchronous ZeroMQ Transport**: High-throughput binary MessagePack serialization over TCP (`127.0.0.1:8090`) or Unix Domain Sockets (`ipc:///tmp/lerobot_policy.ipc`), eliminating ROS 2 serialization overhead for large image tensors.
- **Real-Time Chunking (RTC) Compensation**: Reconciles neural network inference latency ($\sim 14$ steps) by dynamically blending predicted action chunks using weighted temporal averaging.
- **Bumpless Joint Command Smoothing**: Enforces velocity limits, exponential moving average (EMA $\alpha = 0.75$), and soft spatial deadbands to eliminate servo jerk and protect Feetech STS3215 bus servos.
- **Action Server Integration**: Implements the `ExecuteChessMove.action` server, supporting autonomous pick, place, move, and clear primitives alongside manual enabling and E-Stop services.
- **Standard Diagnostics**: Exposes real-time policy health, round-trip inference latency, queue depth, and frame drop rates to `/diagnostics`.

---

## ROS 2 Node Reference: `async_ros2_client_node`

### 1. Subscribed Topics

| Topic Name | Message Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/cameras/usb_wrist/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Best Effort, Depth 1 | Eye-in-Hand wrist camera feed. |
| `/cameras/usb_side/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Best Effort, Depth 1 | Lateral workspace perspective camera feed. |
| `/cameras/stereo_right/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Best Effort, Depth 1 | Overhead/stereo camera workspace feed. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Best Effort, Depth 1 | Feedback positions of the 6-DoF arm servos. |

### 2. Published Topics

| Topic Name | Message Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/arm_forward_controller/commands` | `std_msgs/msg/Float64MultiArray` | Reliable, Depth 1 | Target joint position commands (rad) streamed at 50 Hz to `ros2_control`. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Volatile, Depth 10 | Inference latency, queue health, and server connection telemetry. |

### 3. Services and Actions

| Interface Name | Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `~/execute_chess_move` | `lekiwi_interfaces/action/ExecuteChessMove` | Action Server | High-level goal endpoint coordinating physical pick-and-place trajectories. |
| `/lekiwi/inference/enable` | `std_srvs/srv/SetBool` | Service Server | Enables or pauses policy streaming without terminating the node. |
| `/lekiwi/estop` | `std_srvs/srv/Trigger` | Service Server | Immediate software emergency stop; halts actuation and clears action queues. |

---

## Configuration Parameter Reference

Key parameters configured via `config/control/orchestrator.yaml` or launch arguments:

| Parameter | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `server_address` | `string` | `"127.0.0.1:8090"` | Target ZeroMQ policy server endpoint (`tcp://...` or `ipc://...`). |
| `policy_type` | `string` | `"smolvla"` | Architecture type: `"smolvla"`, `"act"`, or `"diffusion"`. |
| `repo_id` | `string` | `"DuyKhongCay/lekiwi_chess_smolvla"` | Hugging Face policy identifier. |
| `control_rate` | `float` | `25.0` | Frequency of observation capture and inference requests (Hz). |
| `actuation_rate` | `float` | `50.0` | Frequency of joint command actuation and filtering loop (Hz). |
| `actions_per_chunk` | `int` | `30` | Horizon length of action chunks predicted per inference cycle. |
| `chunk_size_threshold` | `float` | `0.35` | Queue drain threshold triggering pre-emptive chunk inference. |
| `observation_filter_epsilon` | `float` | `0.05` | Minimum joint delta required to trigger fresh inference when stationary. |
| `enable_actuation_smoothing` | `bool` | `true` | Activates EMA low-pass filtering on commanded joint positions. |
| `ema_alpha` | `float` | `0.75` | EMA smoothing weight ($0.0 < \alpha \le 1.0$). Lower values increase smoothing. |
| `delta_deadband_m` | `float` | `0.012` | Cartesian threshold suppressing micro-jitters near destination. |
| `default_inference_delay` | `int` | `14` | Anticipated neural latency in discrete control steps for RTC compensation. |

---

## Quick Start & Usage

### 1. Launch Inference Node with Robot Stack

```bash
# Launch robot controllers and policy client in mock or physical mode
ros2 launch lekiwi_bringup robot.launch.py \
  hardware_type:=real \
  manipulation:=policy
```

### 2. Manual CLI Service Interaction

```bash
# Pause policy execution
ros2 service call /lekiwi/inference/enable std_srvs/srv/SetBool "{data: false}"

# Resume policy execution
ros2 service call /lekiwi/inference/enable std_srvs/srv/SetBool "{data: true}"

# Trigger immediate Emergency Stop
ros2 service call /lekiwi/estop std_srvs/srv/Trigger
```

### 3. Monitoring Diagnostics

```bash
ros2 topic echo /diagnostics | grep -A 10 "Policy Inference Status"
```

---

## Verification & Unit Testing

The package includes 7 automated unit test suites built with `pytest` verifying communication, mathematical smoothing, and action coordination:

```bash
# Run all inference unit tests via colcon
colcon test --packages-select lekiwi_inference --event-handlers console_direct+

# Inspect detailed test outcomes
colcon test-result --verbose
```

### Test Suite Descriptions

| Test Suite Target | Source File | Description |
| :--- | :--- | :--- |
| `test_async_client_filter` | `test/test_async_client_filter.py` | Validates observation delta gating, stale frame rejection, and client worker threads. |
| `test_helpers` | `test/test_helpers.py` | Asserts coordinate transformations, reachability heuristics, and EMA math. |
| `test_joint_command_smoother` | `test/test_joint_command_smoother.py` | Validates exponential moving average filters, soft deadband suppression, and velocity clamping. |
| `test_node_integration` | `test/test_node_integration.py` | Exercises node lifecycle, parameter parsing, ROS 2 topic routing, and service triggers. |
| `test_rtc_action_queue` | `test/test_rtc_action_queue.py` | Tests Real-Time Chunking action queuing, horizon blending, and weighted temporal aggregation. |
| `test_smolvla_observation` | `test/test_smolvla_observation.py` | Validates multi-image observation tensor formatting, token metadata, and state concatenation. |
| `test_zmq_transport` | `test/test_zmq_transport.py` | Asserts MessagePack serialization, ZeroMQ REQ/REP socket resilience, and timeout recovery. |


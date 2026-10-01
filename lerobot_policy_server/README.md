# LeRobot Policy Server (ZeroMQ) for LeKiwi Robot

GPU/Host-side policy inference server for **LeKiwi ROS 2**, serving [HuggingFace LeRobot](https://github.com/huggingface/lerobot) policy predictions over **ZeroMQ (ZMQ)**.

This package is optimized for real-time robotic teleoperation and autonomous manipulation (e.g. ACT, SmolVLA, Diffusion Policy), allowing policies to run on a dedicated GPU workstation or cloud instance (Vast.ai, RunPod) and communicate with the robot with low latency.

---

## Key Features

- **Pure ZeroMQ Transport**: Single high-performance `zmq.REP` socket handling handshakes, dynamic policy configuration, streaming inference, and graceful teardown. No gRPC/Protobuf compilation or dependencies.
- **Binary Wire Protocol**: Low latency serialization using `msgpack` metadata header + raw contiguous array bytes (no Python `pickle`).
- **Dynamic Observation Handling**: Supports stereo/wrist cameras (JPEG compressed or raw RGB) and joint state telemetry without hardcoded joint dimensions.
- **VLA Aspect-Ratio Preservation**: Avoids premature image resizing to ensure compatibility with vision-language-action policies (SmolVLA, Pi0).
- **Dual Build Compatibility**: Fully compatible with PyPA standards (`pyproject.toml`, PEP 621, PEP 561) and ROS 2 build tools (`colcon build`).

---

## Installation

### Option 1: Editable Install (Development)

```bash
cd /root/docker_ws/lekiwi_ros2/lerobot_policy_server
pip install -e .
```

### Option 2: Build via Colcon in ROS 2 Workspace

```bash
cd /root/docker_ws/lekiwi_ros2
colcon build --packages-select lerobot_policy_server --symlink-install
source install/setup.bash
```

---

## Running the Server

### Using the Console Script:

```bash
lerobot-policy-server --host 0.0.0.0 --port 8090 --fps 50 --device cuda
```

### Using Python Module:

```bash
python3 -m lerobot_policy_server --host 0.0.0.0 --port 8090 --fps 50 --device cuda
```

### Available CLI Options:

| Flag | Default | Environment Variable | Description |
| :--- | :--- | :--- | :--- |
| `--host` | `0.0.0.0` | `POLICY_SERVER_HOST` | Host address to bind ZMQ socket |
| `--port` | `8090` | `POLICY_SERVER_PORT` | TCP port |
| `--fps` | `30` | `POLICY_SERVER_FPS` | Target inference frame rate |
| `--inference-latency` | `0.033` | `POLICY_SERVER_INFERENCE_LATENCY` | Target inference latency budget (s) |
| `--device` | `cuda` | `POLICY_SERVER_DEVICE` | Compute device (`cuda` or `cpu`) |
| `--log-level` | `INFO` | `LOGLEVEL` | Logging level (`DEBUG`, `INFO`, `WARNING`) |

---

## Wire Protocol Specification

Communication uses ZeroMQ `REQ` (Client / Robot) and `REP` (Server).

### Message Types:

1. **`handshake`**:
   - Client sends `{"type": "handshake"}`
   - Server resets session state and returns `{"status": "ok"}`
2. **`policy_config`**:
   - Client sends `{"type": "policy_config", "data": <serialized_remote_policy_config>}`
   - Server loads model weights and returns `{"status": "ok"}`
3. **`infer`**:
   - Client sends `{"type": "infer", "data": <binary_timed_observation>}`
   - Server predicts action chunk and returns `{"status": "ok", "actions": <binary_actions_chunk>}`
4. **`disconnect`**:
   - Client sends `{"type": "disconnect"}`
   - Server releases policy, cleans CUDA cache (`torch.cuda.empty_cache()`), returns `{"status": "ok"}`

### Binary Observation Format:
```
[4 bytes: uint32-LE header length] + [msgpack metadata header] + [raw binary array/tensor parts]
```

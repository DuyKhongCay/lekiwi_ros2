# CLI Tools & Utility Scripts Reference

This document catalogs standalone utility scripts available in `lekiwi_ros2/scripts/` and official ROS 2 package CLI entry points.

---

## 🛠️ Diagnostics & Analysis Scripts

### 1. `scripts/analyzing_traces.py`
Analyzes ROS 2 executor callback execution durations, scheduling latencies, and symbol resolutions from a `tracetools` trace:

```bash
python3 scripts/analyzing_traces.py /root/.ros/tracing/lekiwi_perception_trace
```

### 2. `scripts/save_image.py`
Subscribes to an active ROS 2 camera image topic, captures a single frame, and writes it directly to disk as a PNG/JPEG file:

```bash
python3 scripts/save_image.py
```

---

## 📦 Package CLI Entry Points

### 1. LeRobot Dataset Converter (`chess_rosbag_to_lerobot`)
Converts raw MCAP episode rosbags into standard Hugging Face LeRobot v3.0 format:

```bash
ros2 run chess_rosbag_to_lerobot chess_convert \
    --input-dir ros_bag/session_demo \
    --config $(ros2 pkg prefix chess_rosbag_to_lerobot)/share/chess_rosbag_to_lerobot/config/lekiwi_chess.yaml \
    --repo-id "local/my_dataset" \
    --output-dir datasets/my_dataset \
    --vcodec libsvtav1
```

### 2. Policy Inference Client Node (`lekiwi_inference`)
Launches the asynchronous ZeroMQ policy client node connecting to a local or remote LeRobot policy server:

```bash
ros2 run lekiwi_inference async_ros2_client_node --ros-args \
    -p server_address:="127.0.0.1:8090"
```

### 3. LeRobot Policy Server CLI (`lerobot_policy_server`)
Spawns the ZeroMQ inference server on a workstation:

```bash
python3 -m lerobot_policy_server --host 0.0.0.0 --port 8090 --fps 30 --device cuda
```

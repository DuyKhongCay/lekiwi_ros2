# CLI Tools & Utility Scripts Reference

This document catalogs standalone utility scripts available in `lekiwi_ros2/scripts/` and official ROS 2 package CLI entry points.

---

## 🛠️ Diagnostics & Analysis Scripts

### 1. `scripts/analyze_bag.py`
Analyzes recorded MCAP demonstration rosbags for topic publication rates, message counts, timestamp alignment, and dropped frames:

```bash
python3 scripts/analyze_bag.py --bag ros_bag/session_chess_demo/episode_000001
```

### 2. `scripts/calib_analysis/analyze_square_calib.py`
Evaluates chessboard vision calibration datasets and computes camera reprojection accuracy and tag orientation matrices:

```bash
python3 scripts/calib_analysis/analyze_square_calib.py
```

### 3. `scripts/save_image.py`
Extracts raw image frames from camera topics and saves JPEG/PNG files for dataset inspection.

---

## 📦 Package CLI Entry Points

### 1. Teleoperation Leader Calibration (`teleop_zhongli_servo_hw`)
Directly benchmarks and zeros the optical encoders of the leader arm without starting the full ROS 2 graph:

```bash
ros2 run teleop_zhongli_servo_hw calibrate_uarm_node
```

### 2. LeRobot Dataset Converter (`chess_rosbag_to_lerobot`)
Converts raw MCAP episode rosbags into standard Hugging Face LeRobot v3.0 format:

```bash
ros2 run chess_rosbag_to_lerobot chess_convert \
    --input-dir ros_bag/session_demo \
    --config $(ros2 pkg prefix chess_rosbag_to_lerobot)/share/chess_rosbag_to_lerobot/config/lekiwi_chess.yaml \
    --repo-id "local/my_dataset" \
    --output-dir datasets/my_dataset \
    --vcodec libsvtav1
```

### 3. LeRobot Policy Server CLI (`lerobot_policy_server`)
Spawns the ZeroMQ inference server on a workstation:

```bash
python3 -m lerobot_policy_server --host 0.0.0.0 --port 8090 --fps 30 --device cuda
```

# How-To: Convert MCAP Rosbag to LeRobot v3.0 Dataset

This guide explains how to convert raw LeKiwi MCAP rosbags into Hugging Face LeRobot format for training ACT, Diffusion, and SmolVLA policies.

---

## 1. Converter Pipeline Execution

Run `chess_rosbag_to_lerobot` using the `chess_convert` entry point with required configuration files:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 run chess_rosbag_to_lerobot chess_convert \
    --input-dir ~/lekiwi_ws/src/lekiwi_ros2/ros_bag/session_chess_demo \
    --config $(ros2 pkg prefix chess_rosbag_to_lerobot)/share/chess_rosbag_to_lerobot/config/lekiwi_chess.yaml \
    --repo-id "local/lekiwi_chess_lerobot" \
    --output-dir ~/datasets/lekiwi_chess_lerobot \
    --vcodec libsvtav1
```

### Conversion steps executed automatically:
1. Decompresses ROS `CompressedImage` frames into synchronized video streams (`observation.images.head_camera`, `observation.images.wrist_camera`).
2. Interpolates joint states and action commands to a constant 30 Hz timeline.
3. Computes statistical normalization parameters (min, max, mean, std) across all joint degrees of freedom.
4. Generates Hugging Face dataset metadata (`info.json` and `stats.json`).

---

## 2. Dataset Inspection & Verification

Verify the dataset using the LeRobot Python API:

```python
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(root="~/datasets/lekiwi_chess_lerobot")
print(f"Total episodes: {dataset.num_episodes}")
print(f"Total frames: {dataset.num_frames}")
print(f"Features: {dataset.features}")
```

---

## 3. Upload to Hugging Face Hub (Optional)

Push the dataset to Hugging Face Hub for training on GPU clusters:

```bash
huggingface-cli login
python3 -m lerobot.scripts.push_dataset_to_hub \
    --raw_dir ~/datasets/lekiwi_chess_lerobot \
    --repo-id "your-org/lekiwi-chess-v1"
```

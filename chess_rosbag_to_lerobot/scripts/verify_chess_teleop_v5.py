# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Dry-run verification of SmolVLA 12D observation state and episode sorting."""

from __future__ import annotations

import logging
from pathlib import Path
import sys

from chess_rosbag_to_lerobot.bag_reader import (
    find_episode_dirs,
    get_custom_data,
    open_reader,
)
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    parse_chess_task,
    sort_episodes,
)
from chess_rosbag_to_lerobot.config import load_config
from chess_rosbag_to_lerobot.decoders import decode

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Error: ROS 2 runtime not available. Please source /opt/ros/jazzy/setup.bash.")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("verify_teleop_v5")


def main():
    workspace = Path(__file__).resolve().parents[3]
    bag_root = workspace / "lekiwi_ros2" / "ros_bag" / "chess_teleop_v5"
    cfg_path = workspace / "lekiwi_ros2" / "chess_rosbag_to_lerobot" / "config" / "lekiwi_chess.yaml"

    if not bag_root.is_dir():
        print(f"Dataset directory not found: {bag_root}")
        return

    cfg = load_config(cfg_path)
    episodes = find_episode_dirs(bag_root)
    print(f"Found {len(episodes)} episodes in {bag_root}")

    sorted_episodes = sort_episodes(
        episodes,
        task_extractor=lambda ep: get_custom_data(ep).get("task", cfg.task),
    )
    print(f"Successfully sorted {len(sorted_episodes)} episodes by task primitive and piece type.")

    joint_state_msg_cls = get_message("sensor_msgs/msg/JointState")
    state_spec = next(f for f in cfg.features if f.key == "observation.state")

    for idx, ep_dir in enumerate(sorted_episodes[:5]):
        custom = get_custom_data(ep_dir)
        task_str = custom.get("task", cfg.task)
        parsed = parse_chess_task(task_str)
        print(
            f"[{idx + 1}] {ep_dir.name}: Task={parsed.task_type.value}, "
            f"Piece={parsed.piece_type}, Color={parsed.piece_color}"
        )


if __name__ == "__main__":
    main()


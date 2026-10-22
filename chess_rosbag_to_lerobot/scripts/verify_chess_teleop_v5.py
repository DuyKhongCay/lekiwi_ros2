# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Dry-run verification of phase conditioning and milestone detection across all 67 episodes."""

from __future__ import annotations

import logging
from pathlib import Path
import sys
import numpy as np

from chess_rosbag_to_lerobot.bag_reader import (
    find_episode_dirs,
    get_custom_data,
    get_topic_types,
    open_reader,
)
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    condition_episode_frames,
    detect_kinematic_milestones,
    resolve_episode_spatial_plan,
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
        print(f"Error: Dataset directory not found: {bag_root}")
        sys.exit(1)

    cfg = load_config(cfg_path)
    episodes = find_episode_dirs(bag_root)
    print(f"Found {len(episodes)} episodes in {bag_root}")

    joint_state_msg_cls = get_message("sensor_msgs/msg/JointState")
    state_spec = next(f for f in cfg.features if f.key == "observation.state")

    success_count = 0
    fail_count = 0
    task_counts = {}

    leading_idle_before = []
    leading_idle_after = []
    trailing_stow_before = []
    trailing_stow_after = []

    for idx, ep_dir in enumerate(episodes):
        custom = get_custom_data(ep_dir)
        task_str = custom.get("task", cfg.task)

        spatial_plan = resolve_episode_spatial_plan(task_str, cfg.chessboard)
        task_type = spatial_plan.task_type
        task_counts[task_type.value] = task_counts.get(task_type.value, 0) + 1

        # Read joint states from bag
        reader = open_reader(ep_dir)
        states = []
        raw_frames = []

        while reader.has_next():
            topic, data, bag_ts_ns = reader.read_next()
            if topic == "/joint_states":
                msg = deserialize_message(data, joint_state_msg_cls)
                q = decode(msg, state_spec)
                states.append(q)
                raw_frames.append({
                    "observation.state": q,
                    "action": q.copy(),
                    "task": spatial_plan.canonical_task,
                })

        if not states:
            logger.error("[%d/%d] %s: No joint states found!", idx + 1, len(episodes), ep_dir.name)
            fail_count += 1
            continue

        try:
            milestones = detect_kinematic_milestones(
                states, task_type, cfg.phase_conditioning, cfg.fps
            )
            conditioned = condition_episode_frames(raw_frames, spatial_plan, cfg)

            n_raw = len(raw_frames)
            n_cond = len(conditioned)
            raw_tail = n_raw - milestones.settle_idx

            leading_idle_before.append(milestones.onset_idx)
            leading_idle_after.append(min(milestones.onset_idx, cfg.phase_conditioning.trim_leading_idle_frames))
            trailing_stow_before.append(raw_tail)

            # Check conditioned tail
            cond_states = [f["observation.state"] for f in conditioned]
            cond_milestones = detect_kinematic_milestones(
                cond_states, task_type, cfg.phase_conditioning, cfg.fps
            )
            cond_tail = len(conditioned) - cond_milestones.settle_idx
            trailing_stow_after.append(cond_tail)

            success_count += 1
            logger.info(
                "[%d/%d] %s: %s | raw=%d -> cond=%d | onset=%d grasp=%s rel=%s settle=%d | raw_tail=%d cond_tail=%d",
                idx + 1,
                len(episodes),
                ep_dir.name,
                task_type.value.upper(),
                n_raw,
                n_cond,
                milestones.onset_idx,
                str(milestones.grasp_idx),
                str(milestones.release_idx),
                milestones.settle_idx,
                raw_tail,
                cond_tail,
            )

        except Exception as exc:
            logger.error("[%d/%d] %s: FAILED - %s", idx + 1, len(episodes), ep_dir.name, exc)
            fail_count += 1

    print("\n" + "=" * 60)
    print(f"VERIFICATION SUMMARY: {success_count}/{len(episodes)} episodes succeeded ({fail_count} failed)")
    print(f"Task Breakdown: {task_counts}")
    print(f"Leading Idle: before [min={min(leading_idle_before)}, max={max(leading_idle_before)}, avg={np.mean(leading_idle_before):.1f}] frames")
    print(f"Leading Idle: after  [min={min(leading_idle_after)}, max={max(leading_idle_after)}, avg={np.mean(leading_idle_after):.1f}] frames")
    print(f"Trailing Stow: before [min={min(trailing_stow_before)}, max={max(trailing_stow_before)}, avg={np.mean(trailing_stow_before):.1f}] frames")
    print(f"Trailing Stow: after  [min={min(trailing_stow_after)}, max={max(trailing_stow_after)}, avg={np.mean(trailing_stow_after):.1f}] frames")
    print("=" * 60)

    if fail_count > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()


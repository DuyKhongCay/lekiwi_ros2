# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Core converter: reference-topic-driven rosbag2 -> LeRobot Dataset v3.0."""

from __future__ import annotations

import logging
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

from chess_rosbag_to_lerobot.bag_reader import (
    find_episode_dirs,
    get_custom_data,
    get_topic_types,
    msg_time_ns,
    open_reader,
)
from chess_rosbag_to_lerobot.buffers import LastBuffer
from chess_rosbag_to_lerobot.chess_spatial_bridge import resolve_pick_place_poses
from chess_rosbag_to_lerobot.config import Config, FeatureSpec
from chess_rosbag_to_lerobot.decoders import decode, get_lerobot_dtype
from chess_rosbag_to_lerobot.tf_replayer import TfReplayer

logger = logging.getLogger(__name__)

POS_KEYS = {"observation.state", "action"}


def _is_visual(spec: FeatureSpec) -> bool:
    return spec.msg_type in ("sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image")


def _add_pos_suffix(names: List[str]) -> List[str]:
    return [n if n.endswith(".pos") else f"{n}.pos" for n in names]


def _format_sync_line(topic: str, s: Dict[str, Any]) -> str:
    mean_ms = (s.get("mean_dt_s") or 0.0) * 1e3
    max_ms = (s.get("max_dt_s") or 0.0) * 1e3
    match_rate = (s.get("match_rate") or 0.0) * 100.0
    miss_empty = int(s.get("miss_empty") or 0)
    miss_future = int(s.get("miss_future") or 0)
    miss_stale = int(s.get("miss_stale") or 0)

    p95 = s.get("p95_dt_s")
    p95_str = f" dt_p95={p95 * 1e3:5.1f}ms" if (p95 is not None and not np.isnan(p95)) else ""

    short_topic = topic if len(topic) <= 42 else "..." + topic[-39:]
    return (
        f"{short_topic:42s} | match={match_rate:5.1f}% | dt_mean={mean_ms:5.1f}ms{p95_str} "
        f"| dt_max={max_ms:5.1f}ms | miss(e/f/s)={miss_empty}/{miss_future}/{miss_stale}"
    )


def build_lerobot_features(cfg: Config, use_videos: bool = True) -> Dict[str, Any]:
    """Construct schema accepted by LeRobotDataset.create()."""
    features: Dict[str, Any] = {}

    for spec in cfg.features:
        if _is_visual(spec):
            if spec.shape is None:
                raise ValueError(f"Visual feature '{spec.key}' must provide shape in YAML")
            features[spec.key] = {
                "dtype": "video" if use_videos else "image",
                "shape": tuple(int(x) for x in spec.shape),
                "names": ["height", "width", "channels"],
            }
        elif spec.msg_type.startswith("spatial/"):
            # Chess spatial coordinates [x, y, z]
            features[spec.key] = {
                "dtype": "float32",
                "shape": (3,),
                "names": ["x", "y", "z"],
            }
        else:
            names = list(spec.names) if spec.names is not None else None
            if names is not None and spec.key in POS_KEYS:
                names = _add_pos_suffix(names)

            shape = (len(names),) if names is not None else (tuple(spec.shape) if spec.shape else (1,))
            features[spec.key] = {
                "dtype": get_lerobot_dtype(spec.msg_type),
                "shape": shape,
                "names": names,
            }

    return features


def convert_one_bag(
    bag_dir: Path,
    cfg: Config,
    dataset: Any,
    *,
    collect_p95: bool = False,
) -> Tuple[int, int]:
    """Convert a single rosbag episode directory into dataset frames."""
    if not _ROS_AVAILABLE:
        raise RuntimeError("ROS 2 python runtime (rclpy, rosidl_runtime_py) not available")

    # 1. Resolve episode task from custom metadata or YAML fallback
    custom = get_custom_data(bag_dir)
    episode_task = custom.get("task")
    task_str = str(episode_task) if episode_task else cfg.task

    # 2. Open bag and introspect topics
    reader = open_reader(bag_dir)
    bag_topic_types = get_topic_types(reader)

    # 3. Match topics and prepare message deserialization classes
    topic_to_spec: Dict[str, FeatureSpec] = {s.topic: s for s in cfg.features if s.topic}
    topic_to_msg_class: Dict[str, type] = {}

    for topic, spec in topic_to_spec.items():
        bag_type = bag_topic_types.get(topic)
        if bag_type is None:
            raise ValueError(f"Configured topic not in bag: {topic} (key={spec.key})")
        if bag_type != spec.msg_type:
            raise ValueError(
                f"Topic {topic} type mismatch: config has {spec.msg_type}, bag has {bag_type}"
            )
        topic_to_msg_class[topic] = get_message(bag_type)

    # Add /tf and /tf_static if present in bag
    tf_msg_cls = get_message("tf2_msgs/msg/TFMessage") if "tf2_msgs/msg/TFMessage" in bag_topic_types.values() else None
    tf_replayer = TfReplayer()

    # 4. Check reference topic
    if cfg.reference_topic not in topic_to_spec:
        raise ValueError(f"Reference topic '{cfg.reference_topic}' not found in config features")
    ref_spec = topic_to_spec[cfg.reference_topic]

    # 5. Initialize buffers for non-reference topics
    buffers: Dict[str, LastBuffer] = {}
    for spec in cfg.features:
        if not spec.topic or spec.topic == cfg.reference_topic:
            continue
        max_age = spec.max_age_s if spec.max_age_s is not None else cfg.default_max_age_s
        buffers[spec.topic] = LastBuffer(
            max_age_ns=int(max_age * 1e9),
            collect_p95=collect_p95,
        )

    # 6. Pre-calculate chess spatial pick and place vectors
    pick_metric, place_metric = resolve_pick_place_poses(task_str, cfg.chessboard)

    frame_count = 0
    dropped_count = 0

    # 7. Process rosbag messages sequentially
    while reader.has_next():
        topic, data, bag_ts_ns = reader.read_next()

        # Handle TF messages
        if topic in ("/tf", "/tf_static") and tf_msg_cls is not None:
            msg = deserialize_message(data, tf_msg_cls)
            tf_replayer.handle_tf_message(msg, is_static=(topic == "/tf_static"))
            continue

        spec = topic_to_spec.get(topic)
        if spec is None:
            continue

        msg_class = topic_to_msg_class[topic]
        msg = deserialize_message(data, msg_class)
        ts_ns = msg_time_ns(msg, spec.stamp_src, bag_ts_ns)

        if topic == cfg.reference_topic:
            # Reference tick clock: assemble frame
            frame: Dict[str, Any] = {}
            frame[ref_spec.key] = decode(msg, ref_spec)
            frame["task"] = task_str

            # Add Chess Spatial features
            pick_pose = pick_metric.copy()
            place_pose = place_metric.copy()
            if cfg.chessboard.base_frame != cfg.chessboard.board_frame:
                transformed_pick = tf_replayer.transform_point(
                    pick_pose, cfg.chessboard.board_frame, cfg.chessboard.base_frame, ts_ns
                )
                if transformed_pick is not None:
                    pick_pose = transformed_pick

                transformed_place = tf_replayer.transform_point(
                    place_pose, cfg.chessboard.board_frame, cfg.chessboard.base_frame, ts_ns
                )
                if transformed_place is not None:
                    place_pose = transformed_place

            for f_spec in cfg.features:
                if f_spec.key == "observation.chess.pick_pose":
                    frame[f_spec.key] = pick_pose
                elif f_spec.key == "observation.chess.place_pose":
                    frame[f_spec.key] = place_pose

            # Sample remaining buffered features
            drop = False
            for other_spec in cfg.features:
                if not other_spec.topic or other_spec.topic == cfg.reference_topic:
                    continue

                val = buffers[other_spec.topic].asof(ts_ns)
                if val is None:
                    drop = True
                    break
                frame[other_spec.key] = val

            if drop:
                dropped_count += 1
                continue

            dataset.add_frame(frame)
            frame_count += 1

        else:
            # Non-reference topic: decode and push into buffer
            decoded_val = decode(msg, spec)
            buffers[topic].push(ts_ns, decoded_val)

    # Episode summary
    drop_pct = (100.0 * dropped_count / max(1, frame_count + dropped_count))
    logger.info("Episode %s: %d frames emitted, %d dropped (%.1f%%)", bag_dir.name, frame_count, dropped_count, drop_pct)
    for topic, buf in sorted(buffers.items()):
        logger.info("  %s", _format_sync_line(topic, buf.summary()))

    if frame_count > 0:
        dataset.save_episode()
    else:
        logger.warning("Episode %s produced 0 valid frames, skipping save", bag_dir.name)

    return frame_count, dropped_count


def convert_all_bags(
    cfg: Config,
    input_dir: Path,
    output_dir: Optional[Path],
    repo_id: str,
    use_videos: bool = True,
    vcodec: str = "libsvtav1",
    push_to_hub: bool = False,
    collect_p95: bool = False,
    overwrite: bool = False,
) -> None:
    """Orchestrate end-to-end conversion across all episode bags in input_dir."""
    episodes = find_episode_dirs(input_dir)
    if not episodes:
        raise RuntimeError(f"No valid rosbag episode directories found in {input_dir}")

    logger.info("Found %d episode(s) to convert in %s", len(episodes), input_dir)

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as e:
        raise ImportError(
            "LeRobot is required for dataset conversion. Please install lerobot or run in an environment with lerobot installed."
        ) from e

    target_path = output_dir if output_dir is not None else (Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id)
    if overwrite and target_path.exists():
        logger.info("Removing existing dataset at %s (--overwrite)", target_path)
        shutil.rmtree(target_path)

    features = build_lerobot_features(cfg, use_videos=use_videos)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_dir,
        fps=cfg.fps,
        robot_type=cfg.robot_type,
        features=features,
        use_videos=use_videos,
        video_backend=vcodec,
    )

    t0 = time.perf_counter()
    total_frames = 0
    total_dropped = 0

    for ep_idx, ep_dir in enumerate(episodes):
        logger.info("[%d/%d] Converting episode: %s", ep_idx + 1, len(episodes), ep_dir.name)
        frames, dropped = convert_one_bag(ep_dir, cfg, dataset, collect_p95=collect_p95)
        total_frames += frames
        total_dropped += dropped

    if push_to_hub:
        logger.info("Pushing dataset to Hugging Face Hub: %s", repo_id)
        dataset.push_to_hub()

    elapsed = time.perf_counter() - t0
    logger.info(
        "Conversion finished in %.1fs: %d episodes, %d total frames, %d dropped (%.1f%%)",
        elapsed,
        len(episodes),
        total_frames,
        total_dropped,
        (100.0 * total_dropped / max(1, total_frames + total_dropped)),
    )

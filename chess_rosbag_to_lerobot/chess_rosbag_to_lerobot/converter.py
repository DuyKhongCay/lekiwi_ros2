# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Core converter: reference-topic-driven rosbag2 -> LeRobot Dataset v3.0."""

from __future__ import annotations

import logging
from pathlib import Path
import shutil
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple
import numpy as np
import yaml

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
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    EpisodeSpatialPlan,
    assemble_environment_state,
    resolve_episode_spatial_plan,
)
from chess_rosbag_to_lerobot.config import Config, FeatureSpec
from chess_rosbag_to_lerobot.decoders import decode, get_lerobot_dtype
from chess_rosbag_to_lerobot.tf_replayer import TfReplayer

logger = logging.getLogger(__name__)

POS_KEYS = {"observation.state", "action"}

ENV_STATE_10D_NAMES = [
    "source_x", "source_y", "source_z",
    "target_x", "target_y", "target_z",
    "is_move", "is_clear", "is_pick", "is_place",
]


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
            # Chess spatial environment state (10D: [source(3), target(3), one_hot(4)])
            features[spec.key] = {
                "dtype": "float32",
                "shape": (10,),
                "names": ENV_STATE_10D_NAMES,
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


# -----------------------------------------------------------------------------
# Decomposed Episode Processing Helpers (Cognitive Complexity <= 6 each)
# -----------------------------------------------------------------------------

def _extract_episode_task(bag_dir: Path, default_task: str) -> str:
    """Extract episode task instruction from metadata.yaml or episode_info.yaml."""
    custom = get_custom_data(bag_dir)
    episode_task = custom.get("task")
    if not episode_task:
        info_path = bag_dir / "episode_info.yaml"
        if info_path.is_file():
            try:
                info_data = yaml.safe_load(info_path.read_text(encoding="utf-8")) or {}
                episode_task = info_data.get("task")
            except Exception as e:
                logger.debug("Failed to read %s: %s", info_path, e)

    return str(episode_task) if episode_task else default_task


def _resolve_and_filter_plan(
    bag_dir: Path,
    task_str: str,
    cfg: Config,
    allowed_types: Set[str],
) -> Optional[EpisodeSpatialPlan]:
    """Parse task into spatial plan and verify primitive against allowed task types."""
    try:
        spatial_plan = resolve_episode_spatial_plan(task_str, cfg.chessboard)
    except ValueError as exc:
        logger.warning("Skipping episode %s: %s", bag_dir.name, exc)
        return None

    if spatial_plan.task_type.value not in allowed_types:
        logger.info(
            "Skipping episode %s: task_type '%s' not in allowed %s",
            bag_dir.name,
            spatial_plan.task_type.value,
            sorted(list(allowed_types)),
        )
        return None

    return spatial_plan


def _prepare_topic_specs(
    bag_topic_types: Dict[str, str],
    cfg: Config,
) -> Tuple[Dict[str, FeatureSpec], Dict[str, type], FeatureSpec]:
    """Validate topic availability and resolve ROS message deserialization classes."""
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

    if cfg.reference_topic not in topic_to_spec:
        raise ValueError(f"Reference topic '{cfg.reference_topic}' not found in config features")

    return topic_to_spec, topic_to_msg_class, topic_to_spec[cfg.reference_topic]


def _init_buffers(cfg: Config, collect_p95: bool) -> Dict[str, LastBuffer]:
    """Initialize As-Of sample buffers for non-reference topics."""
    buffers: Dict[str, LastBuffer] = {}
    for spec in cfg.features:
        if not spec.topic or spec.topic == cfg.reference_topic:
            continue
        max_age = spec.max_age_s if spec.max_age_s is not None else cfg.default_max_age_s
        buffers[spec.topic] = LastBuffer(
            max_age_ns=int(max_age * 1e9),
            collect_p95=collect_p95,
        )
    return buffers


def _build_reference_frame(
    ref_msg: Any,
    ts_ns: int,
    ref_spec: FeatureSpec,
    spatial_plan: EpisodeSpatialPlan,
    buffers: Dict[str, LastBuffer],
    tf_replayer: TfReplayer,
    cfg: Config,
) -> Optional[Dict[str, Any]]:
    """Assemble synchronized frame dictionary at reference tick; returns None if stale."""
    frame: Dict[str, Any] = {
        ref_spec.key: decode(ref_msg, ref_spec),
        "task": spatial_plan.canonical_task,
    }

    # Transform board-frame landmarks if needed
    def transform_endpoint(pt: np.ndarray) -> Optional[np.ndarray]:
        if cfg.chessboard.base_frame == cfg.chessboard.board_frame:
            return pt
        return tf_replayer.transform_point(
            pt, cfg.chessboard.board_frame, cfg.chessboard.base_frame, ts_ns
        )

    env_state = assemble_environment_state(spatial_plan, transform_endpoint)
    for f_spec in cfg.features:
        if f_spec.key == "observation.environment_state":
            frame[f_spec.key] = env_state

    # Sample remaining buffered features
    for other_spec in cfg.features:
        if not other_spec.topic or other_spec.topic == cfg.reference_topic:
            continue

        val = buffers[other_spec.topic].asof(ts_ns)
        if val is None:
            return None  # Stale or missing sample: drop frame
        frame[other_spec.key] = val

    return frame


def _log_and_save_episode(
    bag_dir: Path,
    frame_count: int,
    dropped_count: int,
    buffers: Dict[str, LastBuffer],
    dataset: Any,
) -> Tuple[int, int]:
    """Emit telemetry summary and persist valid episode."""
    drop_pct = 100.0 * dropped_count / max(1, frame_count + dropped_count)
    logger.info(
        "Episode %s: %d frames emitted, %d dropped (%.1f%%)",
        bag_dir.name,
        frame_count,
        dropped_count,
        drop_pct,
    )
    for topic, buf in sorted(buffers.items()):
        logger.info("  %s", _format_sync_line(topic, buf.summary()))

    if frame_count > 0:
        dataset.save_episode()
    else:
        logger.warning("Episode %s produced 0 valid frames, skipping save", bag_dir.name)

    return frame_count, dropped_count


# -----------------------------------------------------------------------------
# Main Conversion Orchestrator
# -----------------------------------------------------------------------------

def convert_one_bag(
    bag_dir: Path,
    cfg: Config,
    dataset: Any,
    *,
    collect_p95: bool = False,
    allowed_task_types: Optional[Set[str]] = None,
) -> Tuple[int, int]:
    """Convert a single rosbag episode directory into dataset frames."""
    task_str = _extract_episode_task(bag_dir, cfg.task)
    if not task_str:
        logger.warning("Skipping episode %s: no task instruction found", bag_dir.name)
        return 0, 0

    active_allowed = allowed_task_types if allowed_task_types is not None else set(cfg.include_task_types)
    spatial_plan = _resolve_and_filter_plan(bag_dir, task_str, cfg, active_allowed)
    if spatial_plan is None:
        return 0, 0

    if not _ROS_AVAILABLE:
        raise RuntimeError("ROS 2 python runtime (rclpy, rosidl_runtime_py) not available")

    reader = open_reader(bag_dir)
    bag_topic_types = get_topic_types(reader)
    topic_to_spec, topic_to_msg_class, ref_spec = _prepare_topic_specs(bag_topic_types, cfg)
    buffers = _init_buffers(cfg, collect_p95)

    tf_msg_cls = get_message("tf2_msgs/msg/TFMessage") if "tf2_msgs/msg/TFMessage" in bag_topic_types.values() else None
    tf_replayer = TfReplayer()

    frame_count = 0
    dropped_count = 0

    while reader.has_next():
        topic, data, bag_ts_ns = reader.read_next()

        if topic in ("/tf", "/tf_static") and tf_msg_cls is not None:
            msg = deserialize_message(data, tf_msg_cls)
            tf_replayer.handle_tf_message(msg, is_static=(topic == "/tf_static"))
            continue

        spec = topic_to_spec.get(topic)
        if spec is None:
            continue

        msg = deserialize_message(data, topic_to_msg_class[topic])
        ts_ns = msg_time_ns(msg, spec.stamp_src, bag_ts_ns)

        if topic == cfg.reference_topic:
            frame = _build_reference_frame(
                msg, ts_ns, ref_spec, spatial_plan, buffers, tf_replayer, cfg
            )
            if frame is None:
                dropped_count += 1
            else:
                dataset.add_frame(frame)
                frame_count += 1
        else:
            buffers[topic].push(ts_ns, decode(msg, spec))

    return _log_and_save_episode(bag_dir, frame_count, dropped_count, buffers, dataset)


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
    allowed_task_types: Optional[Sequence[str]] = None,
) -> None:
    """Orchestrate end-to-end conversion across all episode bags in input_dir."""
    episodes = find_episode_dirs(input_dir)
    if not episodes:
        raise RuntimeError(f"No valid rosbag episode directories found in {input_dir}")

    logger.info("Found %d episode(s) to convert in %s", len(episodes), input_dir)

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.configs.video import RGBEncoderConfig
    except ImportError as e:
        raise ImportError(
            "LeRobot is required for dataset conversion. Please install lerobot or run in an environment with lerobot installed."
        ) from e

    target_path = output_dir if output_dir is not None else (Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id)
    if overwrite and target_path.exists():
        logger.info("Removing existing dataset at %s (--overwrite)", target_path)
        shutil.rmtree(target_path)

    features = build_lerobot_features(cfg, use_videos=use_videos)

    if use_videos and vcodec:
        # Hardware NVENC encoders reject LeRobot's default tight GOP size (g=2)
        encoder_kwargs = {"vcodec": vcodec}
        if "nvenc" in vcodec.lower():
            encoder_kwargs["g"] = None
        rgb_encoder = RGBEncoderConfig(**encoder_kwargs)
    else:
        rgb_encoder = None

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_dir,
        fps=cfg.fps,
        robot_type=cfg.robot_type,
        features=features,
        use_videos=use_videos,
        rgb_encoder=rgb_encoder,
    )

    active_allowed = set(allowed_task_types) if allowed_task_types is not None else set(cfg.include_task_types)

    t0 = time.perf_counter()
    total_frames = 0
    total_dropped = 0

    for ep_idx, ep_dir in enumerate(episodes):
        logger.info("[%d/%d] Converting episode: %s", ep_idx + 1, len(episodes), ep_dir.name)
        frames, dropped = convert_one_bag(
            ep_dir,
            cfg,
            dataset,
            collect_p95=collect_p95,
            allowed_task_types=active_allowed,
        )
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

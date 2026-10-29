# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Core converter: reference-topic-driven rosbag2 -> LeRobot Dataset v3.0."""

from __future__ import annotations

import json
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

from chess_rosbag_to_lerobot.annotation_builder import AnnotationBuilder
from chess_rosbag_to_lerobot.bag_reader import (
    find_episode_dirs,
    get_custom_data,
    get_topic_types,
    msg_time_ns,
    open_reader,
)
from chess_rosbag_to_lerobot.buffers import LastBuffer
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    parse_chess_task,
    sort_episodes,
)
from chess_rosbag_to_lerobot.config import Config, FeatureSpec
from chess_rosbag_to_lerobot.decoders import decode, get_lerobot_dtype
from chess_rosbag_to_lerobot.tf_replayer import TfReplayer
from chess_rosbag_to_lerobot.trajectory_analyzer import (
    EpisodeTrajectoryPlan,
    TrajectoryAnalyzer,
)

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
        elif spec.key == "observation.state":
            # Dynamic Relative Delta 12D: 6 arm joints + 3D End-Effector pos + 3D delta to subtask goal
            names = list(spec.names) if spec.names is not None else []
            names = _add_pos_suffix(names)
            names = names + [
                "ee_x", "ee_y", "ee_z",
                "delta_x", "delta_y", "delta_z",
            ]
            features[spec.key] = {
                "dtype": "float32",
                "shape": (12,),
                "names": names,
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
# Decomposed Episode Processing Helpers
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
    trajectory_plan: EpisodeTrajectoryPlan,
    buffers: Dict[str, LastBuffer],
    tf_replayer: TfReplayer,
    cfg: Config,
) -> Optional[Dict[str, Any]]:
    """Assemble synchronized frame dictionary at reference tick; returns None if stale or TF lookup fails."""
    active_subtask = trajectory_plan.get_active_subtask(ts_ns)

    try:
        ee_xyz = tf_replayer.lookup_tcp_pose(
            cfg.teleop_filter.tool_frame,
            cfg.teleop_filter.base_frame,
            ts_ns,
        )
    except Exception as exc:
        logger.debug("Failed TF lookup for TCP at ts=%d ns: %s", ts_ns, exc)
        return None

    # Calculate 3D Dynamic Relative Delta: goal_xyz - ee_xyz
    delta_xyz = (active_subtask.goal_xyz - ee_xyz).astype(np.float32)

    ref_val = decode(ref_msg, ref_spec)
    if ref_spec.key == "observation.state":
        ref_val = np.concatenate([ref_val, ee_xyz, delta_xyz]).astype(np.float32)

    frame: Dict[str, Any] = {
        ref_spec.key: ref_val,
        "task": trajectory_plan.canonical_task,
    }

    # Sample remaining buffered features
    for other_spec in cfg.features:
        if not other_spec.topic or other_spec.topic == cfg.reference_topic:
            continue

        val = buffers[other_spec.topic].asof(ts_ns)
        if val is None:
            return None  # Stale or missing sample: drop frame

        if other_spec.key == "observation.state":
            # 6 joints + 3D ee_xyz + 3D delta_xyz -> 12D observation.state
            state_12d = np.concatenate([val, ee_xyz, delta_xyz]).astype(np.float32)
            frame[other_spec.key] = state_12d
        else:
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
        if dataset is not None:
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
) -> Tuple[int, int, Optional[EpisodeTrajectoryPlan]]:
    """Convert a single rosbag episode directory into dataset frames using 2-pass filtering."""
    task_str = _extract_episode_task(bag_dir, cfg.task)
    if not task_str:
        logger.warning("Skipping episode %s: no task instruction found", bag_dir.name)
        return 0, 0, None

    active_allowed = allowed_task_types if allowed_task_types is not None else set(cfg.include_task_types)
    try:
        parsed = parse_chess_task(task_str)
    except ValueError as exc:
        logger.warning("Skipping episode %s: %s", bag_dir.name, exc)
        return 0, 0, None

    if parsed.task_type.value not in active_allowed:
        logger.info(
            "Skipping episode %s: task_type '%s' not in allowed %s",
            bag_dir.name,
            parsed.task_type.value,
            sorted(list(active_allowed)),
        )
        return 0, 0, None

    if not _ROS_AVAILABLE:
        raise RuntimeError("ROS 2 python runtime (rclpy, rosidl_runtime_py) not available")

    # Pass 1: Scan TF and JointStates to run 4-layer False Event Filtering
    reader_pass1 = open_reader(bag_dir)
    bag_topic_types = get_topic_types(reader_pass1)
    topic_to_spec, topic_to_msg_class, ref_spec = _prepare_topic_specs(bag_topic_types, cfg)

    tf_msg_cls = get_message("tf2_msgs/msg/TFMessage") if "tf2_msgs/msg/TFMessage" in bag_topic_types.values() else None
    tf_replayer = TfReplayer()

    # Seed nominal static transform (e.g. map -> chessboard_frame) if defined in config
    if cfg.chessboard.chessboard_pose_in_map:
        tf_replayer.seed_static_transform(
            parent_frame=cfg.chessboard.map_frame,
            child_frame=cfg.chessboard.board_frame,
            pose_6d=cfg.chessboard.chessboard_pose_in_map,
        )

    # Determine topic delivering joint states for trajectory analysis
    joint_states_topic = None
    for s in cfg.features:
        if s.key == "observation.state" and s.topic:
            joint_states_topic = s.topic
            break
    if joint_states_topic is None:
        joint_states_topic = "/joint_states"

    joint_samples: List[Tuple[int, np.ndarray]] = []

    while reader_pass1.has_next():
        topic, data, bag_ts_ns = reader_pass1.read_next()
        if topic in ("/tf", "/tf_static") and tf_msg_cls is not None:
            msg = deserialize_message(data, tf_msg_cls)
            tf_replayer.handle_tf_message(msg, is_static=(topic == "/tf_static"))
        elif topic == joint_states_topic and topic in topic_to_msg_class:
            spec = topic_to_spec[topic]
            msg = deserialize_message(data, topic_to_msg_class[topic])
            ts_ns = msg_time_ns(msg, spec.stamp_src, bag_ts_ns)
            decoded = decode(msg, spec)
            joint_samples.append((ts_ns, decoded))
    del reader_pass1

    # Analyze trajectory and resolve subtasks with 4-layer filter
    analyzer = TrajectoryAnalyzer(
        tf_replayer=tf_replayer,
        filter_cfg=cfg.teleop_filter,
        landmarks=cfg.chessboard.landmarks,
    )
    plan = analyzer.analyze(
        joint_samples=joint_samples,
        task_str=task_str,
        task_type=parsed.task_type.value,
        clear_bin_side=parsed.clear_bin_side.value if parsed.clear_bin_side else None,
    )

    if plan is None:
        logger.warning(
            "Skipping episode %s: trajectory analysis failed 4-layer false event validation",
            bag_dir.name,
        )
        return 0, 0, None

    # Pass 2: Synchronous playback driving LeRobot frame assembly
    reader = open_reader(bag_dir)
    buffers = _init_buffers(cfg, collect_p95)
    frame_count = 0
    dropped_count = 0

    while reader.has_next():
        topic, data, bag_ts_ns = reader.read_next()

        if topic in ("/tf", "/tf_static"):
            continue

        spec = topic_to_spec.get(topic)
        if spec is None:
            continue

        msg = deserialize_message(data, topic_to_msg_class[topic])
        ts_ns = msg_time_ns(msg, spec.stamp_src, bag_ts_ns)

        if topic == cfg.reference_topic:
            frame = _build_reference_frame(
                msg, ts_ns, ref_spec, plan, buffers, tf_replayer, cfg
            )
            if frame is None:
                dropped_count += 1
            else:
                if dataset is not None:
                    dataset.add_frame(frame)
                frame_count += 1
        else:
            buffers[topic].push(ts_ns, decode(msg, spec))
    del reader

    frames, dropped = _log_and_save_episode(bag_dir, frame_count, dropped_count, buffers, dataset)
    return frames, dropped, plan


MANIFEST_FILE_NAME = "converted_bags.json"


def _get_bag_identifier(ep_dir: Path, input_dir: Path) -> str:
    """Compute unique namespaced identifier for an episode bag (e.g. 'chess_teleop_v6/episode_000001')."""
    try:
        rel = ep_dir.relative_to(input_dir)
        return f"{input_dir.name}/{rel.as_posix()}"
    except ValueError:
        return f"{ep_dir.parent.name}/{ep_dir.name}"


def _load_converted_manifest(target_path: Path) -> Set[str]:
    """Load set of successfully converted bag directory names from dataset metadata."""
    manifest_path = target_path / "meta" / MANIFEST_FILE_NAME
    if manifest_path.is_file():
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
            return set(data.get("converted_bags", []))
        except Exception as exc:
            logger.warning("Could not read manifest at %s: %s", manifest_path, exc)
            return set()
    return set()


def _save_converted_manifest(target_path: Path, converted_bags: Set[str]) -> None:
    """Atomically persist successfully converted bag directory names to manifest."""
    manifest_path = target_path / "meta" / MANIFEST_FILE_NAME
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "converted_bags": sorted(list(converted_bags)),
        "count": len(converted_bags),
    }
    manifest_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


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

    target_path = output_dir if output_dir is not None else (Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id)
    dataset_exists = target_path.exists() and (target_path / "meta" / "info.json").exists()

    if overwrite and target_path.exists():
        logger.info("Removing existing dataset at %s (--overwrite)", target_path)
        shutil.rmtree(target_path)
        already_converted: Set[str] = set()
    else:
        already_converted = _load_converted_manifest(target_path)

    def _is_already_converted(ep: Path) -> bool:
        bag_id = _get_bag_identifier(ep, input_dir)
        if bag_id in already_converted:
            return True
        # Backward compatibility: match legacy flat names only if whole manifest has no session slashes
        if ep.name in already_converted and not any("/" in item for item in already_converted):
            return True
        return False

    # Filter out already converted episodes when resuming
    pending_episodes = [ep for ep in episodes if not _is_already_converted(ep)]

    if not pending_episodes:
        logger.info(
            "All %d episode(s) in %s have already been successfully converted to %s. Nothing to do.",
            len(episodes),
            input_dir,
            target_path,
        )
        return

    # Sort episodes deterministically: Task (MOVE, CLEAR, PICK, PLACE) -> Piece -> Color -> ID
    pending_episodes = sort_episodes(
        pending_episodes,
        task_extractor=lambda ep: _extract_episode_task(ep, cfg.task),
    )

    logger.info(
        "Total episodes found: %d | Already converted: %d | Pending conversion: %d (sorted by task & piece)",
        len(episodes),
        len(already_converted),
        len(pending_episodes),
    )

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.configs.video import RGBEncoderConfig
    except ImportError as e:
        raise ImportError(
            "LeRobot is required for dataset conversion. Please install lerobot or run in an environment with lerobot installed."
        ) from e

    features = build_lerobot_features(cfg, use_videos=use_videos)

    if use_videos and vcodec:
        # Hardware NVENC encoders reject LeRobot's default tight GOP size (g=2)
        encoder_kwargs = {"vcodec": vcodec}
        if "nvenc" in vcodec.lower():
            encoder_kwargs["g"] = None
        rgb_encoder = RGBEncoderConfig(**encoder_kwargs)
    else:
        rgb_encoder = None

    if dataset_exists and not overwrite:
        logger.info("Resuming existing LeRobot dataset at %s to append pending episodes", target_path)
        dataset = LeRobotDataset.resume(
            repo_id=repo_id,
            root=target_path,
            rgb_encoder=rgb_encoder,
        )
    else:
        logger.info("Creating new LeRobot dataset at %s", target_path)
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
    newly_converted: Set[str] = set(already_converted)
    episode_to_records: Dict[int, List[Dict[str, Any]]] = {}

    for ep_idx, ep_dir in enumerate(pending_episodes):
        logger.info("[%d/%d] Converting episode: %s", ep_idx + 1, len(pending_episodes), ep_dir.name)
        frames, dropped, plan = convert_one_bag(
            ep_dir,
            cfg,
            dataset,
            collect_p95=collect_p95,
            allowed_task_types=active_allowed,
        )
        total_frames += frames
        total_dropped += dropped
        if frames > 0 and plan is not None:
            bag_id = _get_bag_identifier(ep_dir, input_dir)
            newly_converted.add(bag_id)
            _save_converted_manifest(target_path, newly_converted)

            if dataset is not None:
                current_ep_idx = dataset.num_episodes - 1
                episode_to_records[current_ep_idx] = AnnotationBuilder.build_all_language_records(
                    plan.subtask_spans,
                    plan.canonical_task,
                    task_type=plan.task_type,
                )

    if total_frames > 0:
        logger.info("Finalizing LeRobot dataset...")
        dataset.finalize()
        if episode_to_records:
            logger.info("Attaching steerable language annotations to parquet shards...")
            AnnotationBuilder.attach_language_annotations(Path(dataset.root), episode_to_records)
    else:
        logger.info("No new frames added to dataset.")

    if push_to_hub:
        logger.info("Pushing dataset to Hugging Face Hub: %s", repo_id)
        dataset.push_to_hub()

    elapsed = time.perf_counter() - t0
    logger.info(
        "Conversion finished in %.1fs: %d pending processed (%d total in dataset), %d new frames, %d dropped (%.1f%%)",
        elapsed,
        len(pending_episodes),
        len(newly_converted),
        total_frames,
        total_dropped,
        (100.0 * total_dropped / max(1, total_frames + total_dropped)),
    )


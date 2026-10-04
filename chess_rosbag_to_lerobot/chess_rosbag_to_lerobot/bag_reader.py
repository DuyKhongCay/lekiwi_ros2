# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Rosbag reading utilities, metadata extraction, and episode discovery."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional
import yaml

try:
    import rosbag2_py
    _ROSBAG2_AVAILABLE = True
except ImportError:
    _ROSBAG2_AVAILABLE = False

logger = logging.getLogger(__name__)


def read_bag_metadata(bag_dir: Path) -> Dict[str, Any]:
    """Read metadata.yaml from a rosbag directory."""
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.is_file():
        return {}

    try:
        data = yaml.safe_load(meta_path.read_text(encoding="utf-8")) or {}
        info = data.get("rosbag2_bagfile_information", {})
        return info if isinstance(info, dict) else {}
    except Exception as e:
        logger.warning("Failed to parse %s: %s", meta_path, e)
        return {}


def get_custom_data(bag_dir: Path) -> Dict[str, Any]:
    """Extract custom_data dictionary saved in bag metadata.yaml."""
    info = read_bag_metadata(bag_dir)
    custom = info.get("custom_data", {})
    return custom if isinstance(custom, dict) else {}


def get_storage_id(bag_dir: Path, default: str = "mcap") -> str:
    """Determine storage format from metadata (defaults to mcap)."""
    info = read_bag_metadata(bag_dir)
    sid = info.get("storage_identifier", default)
    return str(sid) if sid else default


def open_reader(bag_dir: Path) -> Any:
    """Open a rosbag2 SequentialReader for the given episode directory."""
    if not _ROSBAG2_AVAILABLE:
        raise RuntimeError("rosbag2_py is not available in the current environment")

    storage_id = get_storage_id(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def get_topic_types(reader: Any) -> Dict[str, str]:
    """Map topic names to message type strings in the opened bag."""
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def find_episode_dirs(root: Path | str) -> List[Path]:
    """Recursively or sequentially locate directories containing metadata.yaml."""
    root_path = Path(root)
    if not root_path.exists():
        return []

    # Case 1: The root directory itself is a bag
    if (root_path / "metadata.yaml").is_file():
        return [root_path]

    # Case 2: Direct subdirectories contain metadata.yaml
    dirs = sorted([
        d for d in root_path.iterdir()
        if d.is_dir() and (d / "metadata.yaml").is_file()
    ])

    # Case 3: Nested subdirectories (e.g. root/experiment/episode_*)
    if not dirs:
        dirs = sorted([
            p.parent for p in root_path.glob("**/metadata.yaml")
        ])

    return dirs


def header_stamp_to_ns(msg: Any) -> Optional[int]:
    """Extract nanoseconds from msg.header.stamp if present."""
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    return sec * 1_000_000_000 + nanosec


def msg_time_ns(msg: Any, stamp_src: str, bag_ts_ns: int) -> int:
    """Determine authoritative timestamp for a message in nanoseconds."""
    if stamp_src == "header":
        ts = header_stamp_to_ns(msg)
        if ts is not None and ts > 0:
            return ts
    return int(bag_ts_ns)

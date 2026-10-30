# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List, Optional
import yaml

VALID_TASK_TYPES = ("move", "clear", "pick", "place")


@dataclass(frozen=True)
class TeleopFilterConfig:
    """Configuration for teleoperation event detection and false event filtering."""
    tool_frame: str = "gripperframe"
    base_frame: str = "base_footprint"
    z_threshold_m: float = 0.08
    dwell_time_s: float = 0.5
    transit_min_s: float = 1.0
    gripper_close_threshold: float = 0.15


@dataclass(frozen=True)
class LandmarksConfig:
    """Calibrated 3D Cartesian landmark coordinates [x, y, z] in base_footprint (meters)."""
    stow: List[float] = field(default_factory=lambda: [0.0704, -0.1882, 0.1757])
    clear_left: List[float] = field(default_factory=lambda: [-0.0029, 0.1435, 0.1702])
    clear_right: List[float] = field(default_factory=lambda: [-0.0027, -0.1451, 0.1633])


@dataclass(frozen=True)
class ToolTransformConfig:
    """Calibrated static transform from parent link to child TCP frame."""
    parent_frame: str = "gripper"
    child_frame: str = "gripperframe"
    pose_6d: List[float] = field(
        default_factory=lambda: [
            -0.0079,
            -0.000218214,
            -0.103424,
            -1.57079632679,
            0.0,
            -1.57079632679,
        ]
    )


@dataclass
class FeatureSpec:
    """Specification for a single dataset feature."""
    key: str
    msg_type: str
    topic: Optional[str] = None
    stamp_src: str = "bag"  # 'bag' or 'header'
    shape: Optional[List[int]] = None
    names: Optional[List[str]] = None
    max_age_s: Optional[float] = None


@dataclass
class Config:
    """Global conversion configuration."""
    robot_type: str
    fps: int
    reference_topic: str
    default_max_age_s: float
    task: str
    landmarks: LandmarksConfig = field(default_factory=LandmarksConfig)
    tool_transform: ToolTransformConfig = field(default_factory=ToolTransformConfig)
    teleop_filter: TeleopFilterConfig = field(default_factory=TeleopFilterConfig)
    features: List[FeatureSpec] = field(default_factory=list)
    include_task_types: List[str] = field(
        default_factory=lambda: ["move", "clear", "pick", "place"]
    )


def load_config(path: Path | str) -> Config:
    """Load and validate configuration from YAML file."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping, got {type(data)}")

    # Landmarks section (top-level)
    lm_data = data.get("landmarks", {})
    default_lm = LandmarksConfig()
    landmarks = LandmarksConfig(
        stow=[float(x) for x in lm_data.get("stow", default_lm.stow)],
        clear_left=[float(x) for x in lm_data.get("clear_left", default_lm.clear_left)],
        clear_right=[float(x) for x in lm_data.get("clear_right", default_lm.clear_right)],
    )

    # Tool Transform section
    tt_data = data.get("tool_transform", {})
    default_tt = ToolTransformConfig()
    raw_pose = tt_data.get("pose_6d", default_tt.pose_6d)
    tool_transform = ToolTransformConfig(
        parent_frame=str(tt_data.get("parent_frame", default_tt.parent_frame)),
        child_frame=str(tt_data.get("child_frame", default_tt.child_frame)),
        pose_6d=[float(x) for x in raw_pose],
    )

    # Teleop Filter section
    tf_data = data.get("teleop_filter", {})
    default_tf = TeleopFilterConfig()
    teleop_filter = TeleopFilterConfig(
        tool_frame=str(tf_data.get("tool_frame", default_tf.tool_frame)),
        base_frame=str(tf_data.get("base_frame", default_tf.base_frame)),
        z_threshold_m=float(tf_data.get("z_threshold_m", default_tf.z_threshold_m)),
        dwell_time_s=float(tf_data.get("dwell_time_s", default_tf.dwell_time_s)),
        transit_min_s=float(tf_data.get("transit_min_s", default_tf.transit_min_s)),
        gripper_close_threshold=float(
            tf_data.get("gripper_close_threshold", default_tf.gripper_close_threshold)
        ),
    )

    # Features
    features: List[FeatureSpec] = []
    for f in data.get("features", []):
        spec = FeatureSpec(
            key=f["key"],
            msg_type=f["msg_type"],
            topic=f.get("topic"),
            stamp_src=f.get("stamp_src", "bag"),
            shape=list(f["shape"]) if "shape" in f else None,
            names=list(f["names"]) if "names" in f else None,
            max_age_s=float(f["max_age_s"]) if "max_age_s" in f else None,
        )
        features.append(spec)

    raw_task_types = data.get("include_task_types", ["move", "clear", "pick", "place"])
    include_task_types = [str(t).lower() for t in raw_task_types]
    for task_type in include_task_types:
        if task_type not in VALID_TASK_TYPES:
            raise ValueError(
                f"Invalid task_type '{task_type}' in include_task_types. "
                f"Allowed types: {VALID_TASK_TYPES}"
            )

    return Config(
        robot_type=str(data.get("robot_type", "lekiwi")),
        fps=int(data.get("fps", 25)),
        reference_topic=str(data.get("reference_topic", "")),
        default_max_age_s=float(data.get("default_max_age_s", 0.08)),
        task=str(data.get("task", "Chess pick and place")),
        landmarks=landmarks,
        tool_transform=tool_transform,
        teleop_filter=teleop_filter,
        features=features,
        include_task_types=include_task_types,
    )

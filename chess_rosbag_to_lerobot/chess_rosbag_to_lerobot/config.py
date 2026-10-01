# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List, Optional
import yaml


@dataclass
class ChessboardConfig:
    """Chessboard geometry parameters matching chessboard_pose_estimator SSOT."""
    board_frame: str = "chessboard_frame"
    base_frame: str = "base_footprint"
    square_size_m: float = 0.040
    grasp_clearance_z_m: float = 0.025
    origin_at_center: bool = True


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
    chessboard: ChessboardConfig = field(default_factory=ChessboardConfig)
    features: List[FeatureSpec] = field(default_factory=list)


def load_config(path: Path | str) -> Config:
    """Load and validate configuration from YAML file."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping, got {type(data)}")

    # Chessboard section
    cb_data = data.get("chessboard", {})
    cb_cfg = ChessboardConfig(
        board_frame=cb_data.get("board_frame", "chessboard_frame"),
        base_frame=cb_data.get("base_frame", "base_footprint"),
        square_size_m=float(cb_data.get("square_size_m", 0.040)),
        grasp_clearance_z_m=float(cb_data.get("grasp_clearance_z_m", 0.025)),
        origin_at_center=bool(cb_data.get("origin_at_center", True)),
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

    return Config(
        robot_type=str(data.get("robot_type", "lekiwi")),
        fps=int(data.get("fps", 25)),
        reference_topic=str(data.get("reference_topic", "")),
        default_max_age_s=float(data.get("default_max_age_s", 0.08)),
        task=str(data.get("task", "Chess pick and place")),
        chessboard=cb_cfg,
        features=features,
    )

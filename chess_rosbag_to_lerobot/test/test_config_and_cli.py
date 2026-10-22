# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import logging
from pathlib import Path
import tempfile
import numpy as np
import pytest
import yaml

from chess_rosbag_to_lerobot.cli import build_parser
from chess_rosbag_to_lerobot.config import Config, load_config
from chess_rosbag_to_lerobot.converter import convert_one_bag


def test_load_config_lekiwi_chess():
    cfg_path = Path(__file__).parent.parent / "config" / "lekiwi_chess.yaml"
    cfg = load_config(cfg_path)

    assert cfg.robot_type == "lekiwi"
    assert cfg.fps == 25
    assert cfg.reference_topic == "/cameras/usb_wrist/image_raw/compressed"
    assert cfg.default_max_age_s == 0.05
    assert cfg.include_task_types == ["move", "clear", "pick", "place"]

    # Chessboard check
    assert cfg.chessboard.board_frame == "chessboard_frame"
    assert cfg.chessboard.base_frame == "base_footprint"
    assert cfg.chessboard.map_frame == "map"
    assert cfg.chessboard.chessboard_pose_in_map == [0.0, 0.0, 0.006, 0.0, 0.0, 0.0]
    assert cfg.chessboard.square_size_m == 0.0475
    assert cfg.chessboard.grasp_clearance_z_m == 0.035

    # Landmarks check (XYZ in base_footprint)
    assert len(cfg.chessboard.landmarks.stow) == 3
    assert len(cfg.chessboard.landmarks.clear_left) == 3
    assert len(cfg.chessboard.landmarks.clear_right) == 3

    # Features check
    feature_keys = [f.key for f in cfg.features]
    assert "observation.images.wrist" in feature_keys
    assert "observation.images.side" in feature_keys
    assert "observation.images.overhead" in feature_keys
    assert "observation.state" in feature_keys
    assert "action" in feature_keys
    assert "observation.environment_state" in feature_keys

    # Check 10D environment_state feature shape
    env_spec = next(f for f in cfg.features if f.key == "observation.environment_state")
    assert env_spec.shape == [10]


def test_cli_parser_and_task_types():
    parser = build_parser()
    args = parser.parse_args([
        "--input-dir", "/tmp/episodes",
        "--config", "/tmp/config.yaml",
        "--repo-id", "local/test_dataset",
        "--overwrite",
        "--sync-p95",
        "--task-types", "move", "clear",
    ])

    assert str(args.input_dir) == "/tmp/episodes"
    assert str(args.config) == "/tmp/config.yaml"
    assert args.repo_id == "local/test_dataset"
    assert args.overwrite is True
    assert args.sync_p95 is True
    assert args.use_videos is True
    assert args.task_types == ["move", "clear"]


def test_convert_one_bag_skips_ambiguous_clear_via_value_error(caplog):
    """Verify convert_one_bag catches standard ValueError and skips without opening MCAP."""
    with tempfile.TemporaryDirectory() as tmpdir:
        bag_path = Path(tmpdir)
        meta = {
            "rosbag2_bagfile_information": {
                "custom_data": {
                    "task": "Clear d4 to onboard bin",
                },
            },
        }
        (bag_path / "metadata.yaml").write_text(yaml.safe_dump(meta))

        cfg = Config(
            robot_type="lekiwi",
            fps=25,
            reference_topic="/cam",
            default_max_age_s=0.1,
            task="default",
        )

        with caplog.at_level(logging.WARNING):
            frames, dropped = convert_one_bag(bag_path, cfg, None)

        assert frames == 0
        assert dropped == 0
        assert any("Skipping episode" in rec.message and "Ambiguous CLEAR" in rec.message for rec in caplog.records)

# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from pathlib import Path
import pytest
from chess_rosbag_to_lerobot.config import load_config
from chess_rosbag_to_lerobot.cli import build_parser


def test_load_config_lekiwi_chess():
    cfg_path = Path(__file__).parent.parent / "config" / "lekiwi_chess.yaml"
    cfg = load_config(cfg_path)

    assert cfg.robot_type == "lekiwi"
    assert cfg.fps == 25
    assert cfg.reference_topic == "/cameras/usb_wrist/image_raw/compressed"
    assert cfg.default_max_age_s == 0.08

    # Chessboard check
    assert cfg.chessboard.board_frame == "chessboard_frame"
    assert cfg.chessboard.base_frame == "base_footprint"
    assert cfg.chessboard.square_size_m == 0.040

    # Features check
    feature_keys = [f.key for f in cfg.features]
    assert "observation.images.wrist" in feature_keys
    assert "observation.images.side" in feature_keys
    assert "observation.images.board" in feature_keys
    assert "observation.state" in feature_keys
    assert "action" in feature_keys
    assert "observation.chess.pick_pose" in feature_keys
    assert "observation.chess.place_pose" in feature_keys


def test_cli_parser():
    parser = build_parser()
    args = parser.parse_args([
        "--input-dir", "/tmp/episodes",
        "--config", "/tmp/config.yaml",
        "--repo-id", "local/test_dataset",
        "--overwrite",
        "--sync-p95",
    ])

    assert str(args.input_dir) == "/tmp/episodes"
    assert str(args.config) == "/tmp/config.yaml"
    assert args.repo_id == "local/test_dataset"
    assert args.overwrite is True
    assert args.sync_p95 is True
    assert args.use_videos is True

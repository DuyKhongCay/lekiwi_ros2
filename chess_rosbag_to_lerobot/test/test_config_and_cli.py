# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import logging
from pathlib import Path
import tempfile
import pytest
import yaml

from chess_rosbag_to_lerobot.cli import build_parser
from chess_rosbag_to_lerobot.config import Config, load_config
from chess_rosbag_to_lerobot.converter import build_lerobot_features, convert_one_bag


def test_load_config_lekiwi_chess():
    cfg_path = Path(__file__).parent.parent / "config" / "lekiwi_chess.yaml"
    cfg = load_config(cfg_path)

    assert cfg.robot_type == "lekiwi"
    assert cfg.fps == 25
    assert cfg.reference_topic == "/cameras/usb_wrist/image_raw/compressed"
    assert cfg.default_max_age_s == 0.05
    assert cfg.include_task_types == ["move", "clear", "pick", "place"]

    # Landmarks check (XYZ in base_footprint)
    assert len(cfg.landmarks.stow) == 3
    assert len(cfg.landmarks.clear_left) == 3
    assert len(cfg.landmarks.clear_right) == 3

    # Tool transform check
    assert cfg.tool_transform.parent_frame == "gripper"
    assert cfg.tool_transform.child_frame == "gripperframe"
    assert len(cfg.tool_transform.pose_6d) == 6

    # Teleop filter check
    assert cfg.teleop_filter.tool_frame == "gripperframe"
    assert cfg.teleop_filter.base_frame == "base_footprint"
    assert cfg.teleop_filter.z_threshold_m == 0.15
    assert cfg.teleop_filter.dwell_time_s == 0.5
    assert cfg.teleop_filter.transit_min_s == 1.0
    assert cfg.teleop_filter.gripper_close_threshold == 0.15

    # Features check
    feature_keys = [f.key for f in cfg.features]
    assert "observation.images.wrist" in feature_keys
    assert "observation.images.side" in feature_keys
    assert "observation.images.overhead" in feature_keys
    assert "observation.state" in feature_keys
    assert "action" in feature_keys
    assert "observation.environment_state" not in feature_keys
    assert not hasattr(cfg, "phase_conditioning")

    # Verify 12D observation.state schema in build_lerobot_features
    lerobot_schema = build_lerobot_features(cfg)
    assert "observation.state" in lerobot_schema
    assert lerobot_schema["observation.state"]["shape"] == (12,)
    names = lerobot_schema["observation.state"]["names"]
    assert len(names) == 12
    assert names[0] == "arm_shoulder_pan.pos"
    assert names[5] == "arm_gripper.pos"
    assert names[6:9] == ["ee_x", "ee_y", "ee_z"]
    assert names[9:12] == ["delta_x", "delta_y", "delta_z"]
    assert "observation.environment_state" not in lerobot_schema


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
            frames, dropped, plan = convert_one_bag(bag_path, cfg, None)

        assert frames == 0
        assert dropped == 0
        assert plan is None
        assert any("Skipping episode" in rec.message and "Ambiguous CLEAR" in rec.message for rec in caplog.records)


def test_manifest_save_and_load():
    from chess_rosbag_to_lerobot.converter import _load_converted_manifest, _save_converted_manifest

    with tempfile.TemporaryDirectory() as tmpdir:
        target_path = Path(tmpdir)
        # Empty when non-existent
        assert _load_converted_manifest(target_path) == set()

        # Save and reload
        sample_bags = {"episode_000001", "episode_000002", "episode_000005"}
        _save_converted_manifest(target_path, sample_bags)
        loaded = _load_converted_manifest(target_path)
        assert loaded == sample_bags

        # Append and re-save
        sample_bags.add("episode_000006")
        _save_converted_manifest(target_path, sample_bags)
        assert _load_converted_manifest(target_path) == sample_bags


def test_get_bag_identifier():
    from chess_rosbag_to_lerobot.converter import _get_bag_identifier

    input_dir = Path("/data/ros_bag/chess_teleop_v6")
    ep_dir = Path("/data/ros_bag/chess_teleop_v6/episode_000001")
    assert _get_bag_identifier(ep_dir, input_dir) == "chess_teleop_v6/episode_000001"

    # Different session directory with same episode subfolder name
    input_dir_v7 = Path("/data/ros_bag/chess_teleop_v7")
    ep_dir_v7 = Path("/data/ros_bag/chess_teleop_v7/episode_000001")
    assert _get_bag_identifier(ep_dir_v7, input_dir_v7) == "chess_teleop_v7/episode_000001"
    assert _get_bag_identifier(ep_dir, input_dir) != _get_bag_identifier(ep_dir_v7, input_dir_v7)



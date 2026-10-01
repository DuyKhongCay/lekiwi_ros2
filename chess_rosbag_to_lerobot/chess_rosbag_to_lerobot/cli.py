# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Command-line interface for chess_rosbag_to_lerobot."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from chess_rosbag_to_lerobot.config import load_config
from chess_rosbag_to_lerobot.converter import convert_all_bags

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("chess_rosbag_to_lerobot")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="chess_convert",
        description="Convert LeKiwi chess MCAP rosbags into LeRobot Dataset v3.0 format",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        required=True,
        help="Directory containing episode rosbag directories",
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to YAML conversion configuration file",
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="LeRobot dataset repository ID (e.g., 'local/lekiwi_chess_v1' or 'user/dataset')",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Custom output directory (default writes to ~/.cache/huggingface/lerobot/<repo-id>)",
    )
    parser.add_argument(
        "--vcodec",
        type=str,
        default="libsvtav1",
        help="Video codec backend ('libsvtav1', 'libx264', 'h264_nvenc', etc.)",
    )
    parser.add_argument(
        "--use-videos",
        dest="use_videos",
        action="store_true",
        default=True,
        help="Store visual features as MP4 video (default)",
    )
    parser.add_argument(
        "--no-use-videos",
        dest="use_videos",
        action="store_false",
        help="Store visual features as individual image files",
    )
    parser.add_argument(
        "--push-hub",
        action="store_true",
        default=False,
        help="Push finalized dataset directly to Hugging Face Hub",
    )
    parser.add_argument(
        "--sync-p95",
        action="store_true",
        default=False,
        help="Collect 95th percentile sync latency diagnostics",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        default=False,
        help="Delete existing dataset directory before converting",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        cfg = load_config(args.config)
        logger.info("Loaded configuration from %s (robot=%s, fps=%d)", args.config, cfg.robot_type, cfg.fps)

        convert_all_bags(
            cfg=cfg,
            input_dir=args.input_dir,
            output_dir=args.output_dir,
            repo_id=args.repo_id,
            use_videos=args.use_videos,
            vcodec=args.vcodec,
            push_to_hub=args.push_hub,
            collect_p95=args.sync_p95,
            overwrite=args.overwrite,
        )
        return 0

    except Exception as e:
        logger.error("Conversion failed: %s", e, exc_info=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())

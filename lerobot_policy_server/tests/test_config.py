# Copyright 2026 LeKiwi Robotics Team

from __future__ import annotations

from pathlib import Path
import tempfile

import pytest

from lerobot_policy_server.cli import parse_args
from lerobot_policy_server.config import (
    InferenceEngineConfig,
    PolicyServerConfig,
    ZmqServerConfig,
    get_default_config_path,
)


def test_default_config_path_exists():
    """Verify default YAML configuration file can be resolved and exists."""
    path = get_default_config_path()
    assert isinstance(path, Path)
    assert path.is_file(), f"Default config file not found at {path}"


def test_default_yaml_loading():
    """Verify that Draccus loads the default YAML configuration accurately."""
    config = parse_args([])
    assert isinstance(config, PolicyServerConfig)

    # Server config defaults
    assert config.server.host == "0.0.0.0"
    assert config.server.port == 8090
    assert config.server.fps == 25
    assert abs(config.server.environment_dt - (1.0 / 25.0)) < 1e-6

    # Engine config defaults
    assert config.engine.fps == 25
    assert config.engine.policy_type == "smolvla"
    assert "010000/pretrained_model" in config.engine.pretrained_name_or_path
    assert config.engine.actions_per_chunk == 30
    assert config.engine.device in ("cuda", "cpu")
    assert config.engine.preload is True
    assert config.engine.warmup is True
    assert config.engine.rtc.enabled is True
    assert config.engine.rtc.execution_horizon == 10
    assert config.engine.rtc.max_guidance_weight == 10.0
    assert config.engine.rtc.prefix_attention_schedule == "linear"

    # Top-level defaults
    assert config.log_level == "INFO"


def test_cli_overrides_yaml():
    """Verify that command-line arguments override YAML configuration values."""
    argv = [
        "--server.host",
        "127.0.0.1",
        "--server.port",
        "8095",
        "--engine.device",
        "cpu",
        "--engine.actions_per_chunk",
        "15",
        "--engine.preload",
        "true",
        "--log_level",
        "DEBUG",
    ]
    config = parse_args(argv)

    assert config.server.host == "127.0.0.1"
    assert config.server.port == 8095
    assert config.engine.device == "cpu"
    assert config.engine.actions_per_chunk == 15
    assert config.engine.preload is True
    assert config.log_level == "DEBUG"


def test_custom_config_path():
    """Verify loading an arbitrary YAML config via --config_path."""
    custom_yaml_content = """
server:
  host: "10.0.0.1"
  port: 9999
  fps: 25

engine:
  fps: 25
  policy_type: "smolvla"
  pretrained_name_or_path: "/custom/model/path"
  actions_per_chunk: 20
  device: "cpu"
  preload: true

log_level: "WARNING"
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(custom_yaml_content)
        temp_path = f.name

    try:
        config = parse_args(["--config_path", temp_path])
        assert config.server.host == "10.0.0.1"
        assert config.server.port == 9999
        assert config.server.fps == 25
        assert abs(config.server.environment_dt - (1.0 / 25.0)) < 1e-6

        assert config.engine.fps == 25
        assert config.engine.pretrained_name_or_path == "/custom/model/path"
        assert config.engine.actions_per_chunk == 20
        assert config.engine.preload is True
        assert config.log_level == "WARNING"
    finally:
        Path(temp_path).unlink()


def test_environment_dt_properties():
    """Verify environment_dt calculation across different FPS rates."""
    s_cfg = ZmqServerConfig(fps=25)
    assert abs(s_cfg.environment_dt - 0.04) < 1e-6

    e_cfg = InferenceEngineConfig(fps=50)
    assert abs(e_cfg.environment_dt - 0.02) < 1e-6

    # Test edge case with fps <= 0 (handled gracefully via max(1, fps))
    zero_cfg = ZmqServerConfig(fps=0)
    assert zero_cfg.environment_dt == 1.0


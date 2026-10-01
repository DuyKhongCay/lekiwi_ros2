# Copyright 2026 LeKiwi Robotics Team

import os
from lerobot_policy_server.config import InferenceEngineConfig, ZmqServerConfig


def test_zmq_server_config_defaults():
    cfg = ZmqServerConfig()
    assert cfg.host == "0.0.0.0"
    assert cfg.port == 8090
    assert cfg.fps == 30
    assert abs(cfg.environment_dt - (1.0 / 30.0)) < 1e-6
    assert cfg.device == "cuda"


def test_zmq_server_config_from_env(monkeypatch):
    monkeypatch.setenv("POLICY_SERVER_HOST", "127.0.0.1")
    monkeypatch.setenv("POLICY_SERVER_PORT", "9999")
    monkeypatch.setenv("POLICY_SERVER_FPS", "50")
    monkeypatch.setenv("POLICY_SERVER_DEVICE", "cpu")

    cfg = ZmqServerConfig.from_env()
    assert cfg.host == "127.0.0.1"
    assert cfg.port == 9999
    assert cfg.fps == 50
    assert abs(cfg.environment_dt - 0.02) < 1e-6
    assert cfg.device == "cpu"


def test_inference_engine_config():
    cfg = InferenceEngineConfig(fps=50)
    assert abs(cfg.environment_dt - 0.02) < 1e-6

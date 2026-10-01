# Copyright 2026 LeKiwi Robotics Team

import numpy as np
import pytest

from lerobot_policy_server.config import InferenceEngineConfig
from lerobot_policy_server.engine import FPSTracker, InferenceEngine, decode_jpeg_to_rgb


def test_fps_tracker():
    tracker = FPSTracker(window_size=10)
    # Single timestamp
    m1 = tracker.calculate_fps_metrics(100.0)
    assert m1["avg_fps"] == 0.0

    # Next timestamp after 0.02s (50 Hz)
    m2 = tracker.calculate_fps_metrics(100.02)
    assert abs(m2["instant_fps"] - 50.0) < 1.0


def test_inference_engine_lifecycle(dummy_policy_config, dummy_timed_observation):
    cfg = InferenceEngineConfig(fps=50, device="cpu")
    engine = InferenceEngine(cfg)
    assert engine.running is True

    engine.load_policy(dummy_policy_config)
    assert engine.policy_config is not None

    # Predict action chunk
    actions = engine.predict_action_chunk(dummy_timed_observation)
    assert len(actions) == dummy_policy_config.actions_per_chunk
    assert actions[0].get_timestep() == dummy_timed_observation.get_timestep()

    # Clear session
    engine.clear_session()
    assert engine.policy_config is None

    engine.stop()
    assert engine.running is False


def test_jpeg_decoding():
    import cv2
    # Create a small valid test JPEG in memory
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    img[:, :] = (255, 0, 0)  # Blue in BGR
    success, encoded = cv2.imencode(".jpg", img)
    assert success is True

    decoded_rgb = decode_jpeg_to_rgb(encoded.tobytes())
    assert decoded_rgb.shape == (32, 32, 3)
    # In RGB, BGR (255, 0, 0) should be RGB (0, 0, 255) with lossy tolerance
    assert decoded_rgb[0, 0, 2] >= 250
    assert decoded_rgb[0, 0, 0] <= 5

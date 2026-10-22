# Copyright 2026 LeKiwi Robotics Team

from __future__ import annotations

import numpy as np
import pytest
import torch

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation
from lerobot_policy_server.config import InferenceEngineConfig
from lerobot_policy_server.engine import (
    InferenceEngine,
    _decode_jpeg_to_rgb,
    _raw_observation_to_observation,
)
from tests.conftest import make_dummy_jpeg


def test_decode_jpeg_to_rgb():
    """Verify JPEG decoding and BGR -> RGB channel order conversion."""
    jpeg_bytes = make_dummy_jpeg(64, 64)
    rgb = _decode_jpeg_to_rgb(jpeg_bytes)
    assert rgb.shape == (64, 64, 3)
    assert rgb.dtype == np.uint8


def test_raw_observation_to_observation(dummy_timed_observation):
    """Verify conversion of raw observation dict into policy tensors."""
    raw_obs = dummy_timed_observation.get_observation()
    tensor_obs = _raw_observation_to_observation(raw_obs)

    # Images must be (1, 3, 384, 384) float32 in [0, 1]
    for k in ["observation.images.wrist", "observation.images.side", "observation.images.overhead"]:
        assert k in tensor_obs
        t = tensor_obs[k]
        assert isinstance(t, torch.Tensor)
        assert t.shape == (1, 3, 384, 384)
        assert t.dtype == torch.float32
        assert float(t.min()) >= 0.0
        assert float(t.max()) <= 1.0

    # States
    assert tensor_obs["observation.state"].shape == (1, 6)
    assert tensor_obs["observation.state"].dtype == torch.float32

    assert tensor_obs["observation.environment_state"].shape == (1, 10)
    assert tensor_obs["observation.environment_state"].dtype == torch.float32

    assert tensor_obs["task"] == "play chess"


def test_inference_engine_with_act_lekiwi_chess(dummy_policy_config, dummy_timed_observation):
    """Test full model loading and action chunk inference on act_lekiwi_chess."""
    cfg = InferenceEngineConfig(fps=30)
    engine = InferenceEngine(cfg)
    assert engine.running is True

    # Check uninitialized call raises error
    with pytest.raises(RuntimeError, match="Policy not loaded"):
        engine.predict_action_chunk(dummy_timed_observation)

    # Load model
    engine.load_policy(dummy_policy_config)
    assert engine.policy is not None
    assert engine.preprocessor is not None
    assert engine.postprocessor is not None

    # Predict action chunk
    actions = engine.predict_action_chunk(dummy_timed_observation)
    assert len(actions) == 40

    for i, timed_act in enumerate(actions):
        assert isinstance(timed_act, TimedAction)
        assert timed_act.get_timestep() == dummy_timed_observation.get_timestep() + i
        act = timed_act.get_action()
        if hasattr(act, "cpu"):
            act = act.cpu().numpy()
        assert act.shape == (6,)
        assert act.dtype == np.float32
        assert not np.isnan(act).any(), f"NaN detected at step {i}"
        assert not np.isinf(act).any(), f"Inf detected at step {i}"

    # Test must_go reset
    engine.fps_tracker.calculate_fps_metrics(dummy_timed_observation.get_timestamp())
    assert engine.fps_tracker.total_obs_count == 1
    dummy_timed_observation.must_go = True
    actions_must_go = engine.predict_action_chunk(dummy_timed_observation)
    assert len(actions_must_go) == 40
    assert engine.fps_tracker.total_obs_count == 0
    assert engine.fps_tracker.first_timestamp is None

    # Clean up
    engine.clear_session()
    assert engine.policy is None
    engine.stop()
    assert engine.running is False

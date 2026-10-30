# Copyright 2026 LeKiwi Robotics Team

from __future__ import annotations

import numpy as np
import pytest
import torch

from lerobot.async_inference.helpers import TimedAction
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
    """Verify conversion of raw observation dict into SmolVLA policy tensors."""
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

    # 12D unified state [6 joints + 3D ee_xyz + 3D delta_xyz]
    assert tensor_obs["observation.state"].shape == (1, 12)
    assert tensor_obs["observation.state"].dtype == torch.float32

    # Language task instruction
    assert tensor_obs["task"] == "Move white pawn to target"


def test_raw_observation_fails_fast_on_missing_task(dummy_timed_observation):
    """Verify fail-fast behavior when 'task' instruction is missing from observation."""
    raw_obs = dict(dummy_timed_observation.get_observation())
    del raw_obs["task"]

    with pytest.raises(ValueError, match="Missing or empty 'task' instruction"):
        _raw_observation_to_observation(raw_obs)


def test_raw_observation_fails_fast_on_empty_task(dummy_timed_observation):
    """Verify fail-fast behavior when 'task' instruction is empty or whitespace-only."""
    raw_obs = dict(dummy_timed_observation.get_observation())
    raw_obs["task"] = "   "

    with pytest.raises(ValueError, match="Missing or empty 'task' instruction"):
        _raw_observation_to_observation(raw_obs)


def test_raw_observation_fails_fast_on_invalid_state_dim(dummy_timed_observation):
    """Verify fail-fast rejection when observation.state is not 12D."""
    raw_obs = dict(dummy_timed_observation.get_observation())

    # 6D state (old ACT format) must fail immediately
    raw_obs["observation.state"] = np.zeros(6, dtype=np.float32)
    with pytest.raises(ValueError, match="SmolVLA requires 12D observation.state"):
        _raw_observation_to_observation(raw_obs)

    # 10D state must fail immediately
    raw_obs["observation.state"] = np.zeros(10, dtype=np.float32)
    with pytest.raises(ValueError, match="SmolVLA requires 12D observation.state"):
        _raw_observation_to_observation(raw_obs)


def test_inference_engine_with_smolvla_lekiwi_chess(dummy_policy_config, dummy_timed_observation):
    """Test full model loading and action chunk inference on lekiwi_chess_smolvla."""
    cfg = InferenceEngineConfig(fps=30)
    engine = InferenceEngine(cfg)
    assert engine.running is True

    # Check uninitialized call raises error
    with pytest.raises(RuntimeError, match="Policy not loaded"):
        engine.predict_action_chunk(dummy_timed_observation)

    # Load SmolVLA policy
    engine.load_policy(dummy_policy_config)
    assert engine.policy is not None
    assert engine.preprocessor is not None
    assert engine.postprocessor is not None

    # Predict action chunk (30 steps for SmolVLA)
    actions = engine.predict_action_chunk(dummy_timed_observation)
    assert len(actions) == 30

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
    assert len(actions_must_go) == 30
    assert engine.fps_tracker.total_obs_count == 0
    assert engine.fps_tracker.first_timestamp is None

    # Clean up
    engine.clear_session()
    assert engine.policy is None
    engine.stop()
    assert engine.running is False


def test_inference_engine_synthesize_dummy_observation():
    """Verify synthetic warmup observation generation conforms to LeRobot schema."""
    cfg = InferenceEngineConfig(fps=30, warmup=True)
    engine = InferenceEngine(cfg)

    # Before policy load
    obs_uninit = engine._synthesize_dummy_observation()
    assert obs_uninit.get_observation()["task"] == "warmup"
    assert obs_uninit.get_observation()["observation.state"].shape == (12,)
    assert obs_uninit.must_go is True

    # Warmup on uninitialized policy safely logs warning and does not raise
    engine.warmup()


def test_inference_engine_rtc_predict_with_leftover(dummy_policy_config, dummy_timed_observation):
    """Test full RTC inference flow with leftover actions and inference delay."""
    from lerobot_policy_server.config import RTCServerConfig

    cfg = InferenceEngineConfig(
        fps=25,
        warmup=False,
        rtc=RTCServerConfig(
            enabled=True,
            execution_horizon=10,
            max_guidance_weight=10.0,
            prefix_attention_schedule="linear",
        ),
    )
    engine = InferenceEngine(cfg)
    engine.load_policy(dummy_policy_config)

    assert engine.policy._rtc_enabled() is True
    assert engine.policy.rtc_processor is not None

    # Attach leftover physical joint actions (10 steps)
    leftover_phys = np.full((10, 6), fill_value=0.1, dtype=np.float32)
    dummy_timed_observation.prev_chunk_left_over = leftover_phys
    dummy_timed_observation.inference_delay = 2

    actions = engine.predict_action_chunk(dummy_timed_observation)
    assert len(actions) == 30
    for i, act in enumerate(actions):
        arr = act.get_action()
        if hasattr(arr, "cpu"):
            arr = arr.cpu().numpy()
        assert arr.shape == (6,)
        assert not np.isnan(arr).any()

    engine.clear_session()
    engine.stop()

# Copyright 2026 LeKiwi Robotics Team

import numpy as np
import pytest

from lerobot_policy_server.protocol import (
    deserialize_actions,
    deserialize_observation,
    deserialize_policy_config,
    serialize_actions,
    serialize_observation,
    serialize_policy_config,
)


def test_observation_serialization_roundtrip(dummy_timed_observation):
    encoded = serialize_observation(dummy_timed_observation)
    assert isinstance(encoded, bytes)
    assert len(encoded) > 4

    decoded = deserialize_observation(encoded)
    assert decoded.get_timestep() == dummy_timed_observation.get_timestep()
    assert abs(decoded.get_timestamp() - dummy_timed_observation.get_timestamp()) < 1e-5

    obs_dict = decoded.get_observation()
    assert "observation.state" in obs_dict
    assert "observation.images.camera_overhead" in obs_dict
    assert obs_dict["task"] == "pick_and_place"

    np.testing.assert_allclose(
        obs_dict["observation.state"],
        dummy_timed_observation.get_observation()["observation.state"],
    )
    np.testing.assert_array_equal(
        obs_dict["observation.images.camera_overhead"],
        dummy_timed_observation.get_observation()["observation.images.camera_overhead"],
    )


def test_actions_serialization_roundtrip(dummy_actions_chunk):
    encoded = serialize_actions(dummy_actions_chunk)
    assert isinstance(encoded, bytes)

    decoded_actions = deserialize_actions(encoded)
    assert len(decoded_actions) == len(dummy_actions_chunk)

    for orig, dec in zip(dummy_actions_chunk, decoded_actions):
        assert dec.get_timestep() == orig.get_timestep()
        assert abs(dec.get_timestamp() - orig.get_timestamp()) < 1e-5
        np.testing.assert_allclose(dec.get_action(), orig.get_action())


def test_policy_config_serialization_roundtrip(dummy_policy_config):
    encoded = serialize_policy_config(dummy_policy_config)
    decoded = deserialize_policy_config(encoded)

    assert decoded.repo_id == dummy_policy_config.repo_id
    assert decoded.policy_type == dummy_policy_config.policy_type
    assert decoded.fps == dummy_policy_config.fps
    assert decoded.actions_per_chunk == dummy_policy_config.actions_per_chunk


def test_deserialize_truncated_observation():
    with pytest.raises(ValueError, match="data length < 4 bytes"):
        deserialize_observation(b"\x01\x02")

    with pytest.raises(ValueError, match="payload truncated"):
        # 4 bytes header claiming 1000 bytes header length, but no body
        deserialize_observation(b"\xe8\x03\x00\x00")

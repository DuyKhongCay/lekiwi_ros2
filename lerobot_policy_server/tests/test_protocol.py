# Copyright 2026 LeKiwi Robotics Team

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Ensure lekiwi_inference is importable for two-way protocol testing
lekiwi_inf_path = Path("/home/duykhongcay/lerobot_ws/lekiwi_ros2/lekiwi_inference")
if str(lekiwi_inf_path) not in sys.path:
    sys.path.insert(0, str(lekiwi_inf_path))

from lekiwi_inference.transport import zmq_transport as client_proto
from lerobot_policy_server import protocol as server_proto


def test_observation_twoway_serialization(dummy_timed_observation):
    """Test two-way serialization/deserialization between client and server wire implementations."""
    # 1. Client serialize -> Server deserialize
    wire_bytes_from_client = client_proto.serialize_observation(dummy_timed_observation)
    assert isinstance(wire_bytes_from_client, bytes)
    assert len(wire_bytes_from_client) > 4

    server_decoded = server_proto.deserialize_observation(wire_bytes_from_client)
    assert server_decoded.get_timestep() == dummy_timed_observation.get_timestep()
    assert abs(server_decoded.get_timestamp() - dummy_timed_observation.get_timestamp()) < 1e-5
    assert server_decoded.must_go == dummy_timed_observation.must_go

    obs_dict = server_decoded.get_observation()
    for img_key in ["wrist", "side", "overhead"]:
        full_key = f"observation.images.{img_key}"
        assert full_key in obs_dict
        assert isinstance(obs_dict[full_key], (bytes, bytearray))
        assert obs_dict[full_key] == dummy_timed_observation.get_observation()[full_key]

    np.testing.assert_allclose(
        obs_dict["observation.state"],
        dummy_timed_observation.get_observation()["observation.state"],
    )
    np.testing.assert_allclose(
        obs_dict["observation.environment_state"],
        dummy_timed_observation.get_observation()["observation.environment_state"],
    )
    assert obs_dict["task"] == dummy_timed_observation.get_observation()["task"]

    # 2. Server serialize -> Client deserialize
    wire_bytes_from_server = server_proto.serialize_observation(server_decoded)
    client_decoded = client_proto.deserialize_observation(wire_bytes_from_server)

    assert client_decoded.get_timestep() == dummy_timed_observation.get_timestep()
    client_obs = client_decoded.get_observation()
    assert client_obs["task"] == "play chess"
    np.testing.assert_allclose(
        client_obs["observation.state"],
        dummy_timed_observation.get_observation()["observation.state"],
    )
    np.testing.assert_allclose(
        client_obs["observation.environment_state"],
        dummy_timed_observation.get_observation()["observation.environment_state"],
    )


def test_actions_twoway_serialization(dummy_actions_chunk):
    """Test two-way action chunk wire encoding/decoding."""
    # Server serialize -> Client deserialize
    encoded = server_proto.serialize_actions(dummy_actions_chunk)
    assert isinstance(encoded, bytes)

    decoded = client_proto.deserialize_actions(encoded)
    assert len(decoded) == len(dummy_actions_chunk)

    for orig, dec in zip(dummy_actions_chunk, decoded):
        assert dec.get_timestep() == orig.get_timestep()
        assert abs(dec.get_timestamp() - orig.get_timestamp()) < 1e-5
        np.testing.assert_allclose(dec.get_action(), orig.get_action())

    # Client serialize -> Server deserialize
    re_encoded = client_proto.serialize_actions(decoded)
    server_actions = server_proto.deserialize_actions(re_encoded)
    assert len(server_actions) == len(dummy_actions_chunk)
    for orig, s_act in zip(dummy_actions_chunk, server_actions):
        np.testing.assert_allclose(s_act.get_action(), orig.get_action())


def test_policy_config_twoway_serialization(dummy_policy_config):
    """Test policy configuration msgpack serialization."""
    encoded = client_proto.serialize_policy_config(dummy_policy_config)
    decoded = server_proto.deserialize_policy_config(encoded)

    assert decoded.policy_type == dummy_policy_config.policy_type
    assert decoded.pretrained_name_or_path == dummy_policy_config.pretrained_name_or_path
    assert decoded.actions_per_chunk == dummy_policy_config.actions_per_chunk
    assert decoded.device == dummy_policy_config.device

    # Server re-serialize -> Client deserialize
    re_encoded = server_proto.serialize_policy_config(decoded)
    client_decoded = client_proto.deserialize_policy_config(re_encoded)
    assert client_decoded.pretrained_name_or_path == dummy_policy_config.pretrained_name_or_path


def test_deserialize_truncated_observation():
    """Verify robust error raising on malformed or truncated payload buffers."""
    with pytest.raises(ValueError, match="data length < 4 bytes"):
        server_proto.deserialize_observation(b"\x01\x02")

    with pytest.raises(ValueError, match="payload truncated"):
        # 4 bytes claiming 1000 bytes header length, but buffer terminates immediately
        server_proto.deserialize_observation(b"\xe8\x03\x00\x00")


def test_deserialize_truncated_actions():
    """Verify robust error raising on truncated action buffers."""
    with pytest.raises(ValueError, match="data length < 4 bytes"):
        server_proto.deserialize_actions(b"\x00")

    with pytest.raises(ValueError, match="payload truncated"):
        server_proto.deserialize_actions(b"\x10\x00\x00\x00abc")

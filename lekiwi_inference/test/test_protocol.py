# Copyright 2026 LeKiwi Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for binary wire protocol serialization and deserialization."""

import numpy as np
import pytest

from lekiwi_inference.protocol import (
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
    deserialize_actions,
    deserialize_observation,
    deserialize_policy_config,
    serialize_actions,
    serialize_observation,
    serialize_policy_config,
)


def test_observation_roundtrip():
    """Validates lossless serialization and deserialization of timed observations."""
    state = np.array([0.1, -0.2, 0.5, 1.2, -0.8, 0.0], dtype=np.float32)
    img_wrist = b"\xff\xd8\xff\xe0\x00\x10JFIF"  # Mock JPEG bytes
    img_side = b"\xff\xd8\xff\xe0\x00\x10JFIF_SIDE"

    obs = TimedObservation(
        timestamp=12345.678,
        timestep=42,
        observation={
            "observation.state": state,
            "observation.images.wrist": img_wrist,
            "observation.images.side": img_side,
        },
    )

    encoded = serialize_observation(obs)
    assert isinstance(encoded, bytes)
    assert len(encoded) > 4

    decoded = deserialize_observation(encoded)
    assert decoded.get_timestamp() == pytest.approx(12345.678)
    assert decoded.get_timestep() == 42

    raw_decoded = decoded.get_observation()
    assert np.allclose(raw_decoded["observation.state"], state)
    assert raw_decoded["observation.images.wrist"] == img_wrist
    assert raw_decoded["observation.images.side"] == img_side


def test_actions_roundtrip():
    """Validates lossless serialization and deserialization of predicted action horizons."""
    actions = [
        TimedAction(timestamp=100.0, timestep=0, action=np.array([0.1, 0.2, 0.3], dtype=np.float32)),
        TimedAction(timestamp=100.02, timestep=1, action=np.array([0.4, 0.5, 0.6], dtype=np.float32)),
    ]

    encoded = serialize_actions(actions)
    decoded = deserialize_actions(encoded)

    assert len(decoded) == 2
    assert decoded[0].get_timestep() == 0
    assert decoded[1].get_timestep() == 1
    assert np.allclose(decoded[0].get_action(), [0.1, 0.2, 0.3])
    assert np.allclose(decoded[1].get_action(), [0.4, 0.5, 0.6])


def test_policy_config_roundtrip():
    """Validates serialization of remote model configuration parameters."""
    cfg = RemotePolicyConfig(
        repo_id="legalaspro/act_so101_pnp",
        policy_type="act",
        fps=50.0,
        actions_per_chunk=100,
    )

    encoded = serialize_policy_config(cfg)
    decoded = deserialize_policy_config(encoded)

    assert decoded.repo_id == "legalaspro/act_so101_pnp"
    assert decoded.policy_type == "act"
    assert decoded.fps == 50.0
    assert decoded.actions_per_chunk == 100


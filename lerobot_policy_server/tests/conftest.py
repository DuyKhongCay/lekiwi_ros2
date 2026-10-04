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

"""Pytest fixtures for lerobot_policy_server testing."""

import time
import numpy as np
import pytest

from lerobot_policy_server.config import ZmqServerConfig
from lerobot_policy_server.protocol import (
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
)


@pytest.fixture
def dummy_timed_observation() -> TimedObservation:
    """Fixture providing a mock TimedObservation with state and image."""
    img_rgb = np.zeros((64, 64, 3), dtype=np.uint8)
    joint_state = np.array([0.1, -0.2, 0.5, 0.0, 1.2, -0.8], dtype=np.float32)

    return TimedObservation(
        timestamp=time.time(),
        timestep=42,
        observation={
            "observation.images.camera_overhead": img_rgb,
            "observation.state": joint_state,
            "task": "pick_and_place",
        },
        must_go=False,
    )


@pytest.fixture
def dummy_actions_chunk() -> list[TimedAction]:
    """Fixture providing a chunk of predicted TimedActions."""
    t0 = time.time()
    dt = 0.02
    actions = []
    for i in range(10):
        actions.append(
            TimedAction(
                timestamp=t0 + i * dt,
                timestep=i,
                action=np.full(6, fill_value=float(i) * 0.1, dtype=np.float32),
            )
        )
    return actions


@pytest.fixture
def dummy_policy_config() -> RemotePolicyConfig:
    """Fixture providing a sample RemotePolicyConfig."""
    return RemotePolicyConfig(
        repo_id="lekiwi/act_chessboard_manipulation",
        policy_type="act",
        lerobot_features={
            "observation.state": {"dtype": "float32", "shape": [6], "names": ["j1", "j2", "j3", "j4", "j5", "j6"]},
            "action": {"dtype": "float32", "shape": [6], "names": ["a1", "a2", "a3", "a4", "a5", "a6"]},
        },
        fps=50.0,
        actions_per_chunk=10,
        chunk_size_threshold=0.6,
    )

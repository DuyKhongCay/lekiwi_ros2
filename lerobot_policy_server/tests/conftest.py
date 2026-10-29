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

"""Pytest fixtures for lerobot_policy_server testing matching lekiwi_chess_smolvla."""

from __future__ import annotations

import time

import cv2
import numpy as np
import pytest

from lerobot.async_inference.helpers import (
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
)


def make_dummy_jpeg(h: int = 384, w: int = 384) -> bytes:
    """Helper to generate a valid JPEG byte sequence."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = (120, 180, 240)
    success, enc = cv2.imencode(".jpg", img)
    if not success:
        raise RuntimeError("Failed to encode dummy JPEG")
    return enc.tobytes()


@pytest.fixture
def dummy_timed_observation() -> TimedObservation:
    """Fixture providing a TimedObservation matching lekiwi_chess_smolvla schema.

    The 12D unified observation.state vector follows chess_rosbag_to_lerobot:
    - [0:6]: 6-DoF arm joint positions (joint_states)
    - [6:9]: 3D Cartesian TCP/end-effector coordinates (ee_x, ee_y, ee_z)
    - [9:12]: 3D dynamic relative delta to target (delta_x, delta_y, delta_z)
    """
    jpeg_bytes = make_dummy_jpeg(384, 384)

    # 6-DoF arm joint angles in radians
    joint_states = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.05], dtype=np.float32)
    # 3D end-effector TCP pose in base_footprint frame
    ee_xyz = np.array([0.15, -0.05, 0.035], dtype=np.float32)
    # 3D dynamic relative delta (target - ee)
    delta_xyz = np.array([0.10, 0.15, 0.0], dtype=np.float32)
    state_12d = np.concatenate([joint_states, ee_xyz, delta_xyz]).astype(np.float32)

    return TimedObservation(
        timestamp=time.time(),
        timestep=42,
        observation={
            "observation.images.wrist": jpeg_bytes,
            "observation.images.side": jpeg_bytes,
            "observation.images.overhead": jpeg_bytes,
            "observation.state": state_12d,
            "task": "Move white pawn to target",
        },
        must_go=False,
    )


@pytest.fixture
def dummy_actions_chunk() -> list[TimedAction]:
    """Fixture providing a chunk of 30 predicted TimedActions (6D)."""
    t0 = time.time()
    dt = 1.0 / 30.0
    return [
        TimedAction(
            timestamp=t0 + i * dt,
            timestep=i,
            action=np.full(6, fill_value=float(i) * 0.05, dtype=np.float32),
        )
        for i in range(30)
    ]


@pytest.fixture
def dummy_policy_config() -> RemotePolicyConfig:
    """Fixture providing RemotePolicyConfig pointing to lekiwi_chess_smolvla."""
    return RemotePolicyConfig(
        policy_type="smolvla",
        pretrained_name_or_path="/home/duykhongcay/lerobot_ws/lekiwi_chess_smolvla",
        lerobot_features={},
        actions_per_chunk=30,
        device="cpu",
    )

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

"""Unit tests for ZeroMQ transport client and binary wire serialization."""

import threading
import time

import msgpack
import numpy as np
import pytest
import zmq

from lekiwi_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation
from lekiwi_inference.transport.zmq_transport import (
    ZmqPolicyTransport,
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
    img_wrist = b"\xff\xd8\xff\xe0\x00\x10JFIF"
    img_side = b"\xff\xd8\xff\xe0\x00\x10JFIF_SIDE"

    obs = TimedObservation(
        timestamp=12345.678,
        timestep=42,
        observation={
            "observation.state": state,
            "observation.images.wrist": img_wrist,
            "observation.images.side": img_side,
        },
        must_go=True,
    )

    encoded = serialize_observation(obs)
    assert isinstance(encoded, bytes)

    decoded = deserialize_observation(encoded)
    assert decoded.get_timestamp() == pytest.approx(12345.678)
    assert decoded.get_timestep() == 42
    assert decoded.must_go is True

    raw_decoded = decoded.get_observation()
    assert np.allclose(raw_decoded["observation.state"], state)
    assert raw_decoded["observation.images.wrist"] == img_wrist
    assert raw_decoded["observation.images.side"] == img_side


def test_observation_roundtrip_with_rtc():
    """Validates lossless serialization and deserialization of RTC delay and leftover actions."""
    state = np.array([0.1, -0.2, 0.5, 1.2, -0.8, 0.0], dtype=np.float32)
    leftover = np.random.randn(18, 6).astype(np.float32)

    obs = TimedObservation(
        timestamp=999.0,
        timestep=10,
        observation={"observation.state": state},
        must_go=False,
        inference_delay=14,
        prev_chunk_left_over=leftover,
    )

    encoded = serialize_observation(obs)
    assert isinstance(encoded, bytes)

    decoded = deserialize_observation(encoded)
    assert decoded.get_timestamp() == pytest.approx(999.0)
    assert decoded.get_timestep() == 10
    assert decoded.inference_delay == 14
    assert decoded.prev_chunk_left_over is not None
    assert decoded.prev_chunk_left_over.shape == (18, 6)
    assert np.allclose(decoded.prev_chunk_left_over, leftover)


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
        policy_type="act",
        pretrained_name_or_path="legalaspro/act_so101_pnp",
        lerobot_features={"state": {"shape": (6,), "dtype": "float32"}},
        actions_per_chunk=100,
        device="cuda",
    )

    encoded = serialize_policy_config(cfg)
    decoded = deserialize_policy_config(encoded)

    assert decoded.pretrained_name_or_path == "legalaspro/act_so101_pnp"
    assert decoded.policy_type == "act"
    assert decoded.actions_per_chunk == 100
    assert decoded.device == "cuda"


class MockZmqServer:
    """Simulates basic LeRobot policy server responses for transport verification."""

    def __init__(self, port: int = 5566) -> None:
        self.port = port
        self.endpoint = f"tcp://127.0.0.1:{port}"
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REP)
        self.socket.setsockopt(zmq.RCVTIMEO, 100)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(self.endpoint)
        self.running = True

    def run(self) -> None:
        while self.running:
            try:
                raw_req = self.socket.recv()
                req = msgpack.unpackb(raw_req, raw=False)
                msg_type = req.get("type")

                if msg_type in ("handshake", "policy_config", "disconnect"):
                    self.socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))
                elif msg_type == "infer":
                    dummy_actions = [
                        TimedAction(timestamp=time.time(), timestep=0, action=np.array([0.1, 0.2, 0.3], dtype=np.float32))
                    ]
                    actions_bytes = serialize_actions(dummy_actions)
                    resp = {"status": "ok", "actions": actions_bytes}
                    self.socket.send(msgpack.packb(resp, use_bin_type=True))
            except zmq.Again:
                continue
            except Exception:
                break
        try:
            self.socket.close(linger=0)
            self.ctx.term()
        except Exception:
            pass

    def stop(self) -> None:
        self.running = False


def test_zmq_transport_lifecycle():
    """Validates complete client-server transport lifecycle including infer request-reply."""
    port = 5578
    server = MockZmqServer(port=port)
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()

    transport = ZmqPolicyTransport(
        server_address=f"127.0.0.1:{port}", timeout_ms=500, config_timeout_ms=10000
    )
    assert transport.config_timeout_ms == 10000

    try:
        assert transport.connect()
        assert transport.is_connected()
        assert transport.send_handshake()

        cfg = RemotePolicyConfig(pretrained_name_or_path="test_repo", policy_type="act")
        assert transport.send_policy_config(cfg)

        obs = TimedObservation(
            timestamp=10.0,
            timestep=1,
            observation={"observation.state": np.array([0.0, 0.0, 0.0], dtype=np.float32)},
        )
        actions = transport.infer(obs)
        assert actions is not None
        assert len(actions) == 1
        assert np.allclose(actions[0].get_action(), [0.1, 0.2, 0.3])

        transport.disconnect()
        assert not transport.is_connected()
    finally:
        server.stop()
        server_thread.join(timeout=1.0)

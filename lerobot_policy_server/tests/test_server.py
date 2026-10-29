# Copyright 2026 LeKiwi Robotics Team

from __future__ import annotations

import msgpack
import numpy as np
import pytest
import zmq

from lerobot_policy_server.config import ZmqServerConfig
from lerobot_policy_server.protocol import (
    deserialize_actions,
    serialize_observation,
    serialize_policy_config,
)
from lerobot_policy_server.server import ZmqPolicyServer


def test_zmq_server_e2e_smolvla_lekiwi_chess(dummy_policy_config, dummy_timed_observation):
    """End-to-end integration test with ZmqPolicyServer and lekiwi_chess_smolvla."""
    # Bind to port 0 (OS picks an ephemeral port)
    config = ZmqServerConfig(host="127.0.0.1", port=0, fps=30)
    server = ZmqPolicyServer(config)
    server.start_background()

    port = server.bound_port
    assert port > 0
    assert server.running is True

    # Setup client REQ socket
    client_ctx = zmq.Context()
    client_sock = client_ctx.socket(zmq.REQ)
    client_sock.setsockopt(zmq.RCVTIMEO, 25000)  # 25s timeout for model load/inference
    client_sock.setsockopt(zmq.SNDTIMEO, 5000)
    client_sock.connect(f"tcp://127.0.0.1:{port}")

    try:
        # 1. Test Handshake
        client_sock.send(msgpack.packb({"type": "handshake"}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok"

        # 2. Test Policy Config (Loads lekiwi_chess_smolvla)
        cfg_bytes = serialize_policy_config(dummy_policy_config)
        client_sock.send(
            msgpack.packb({"type": "policy_config", "data": cfg_bytes}, use_bin_type=True)
        )
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok", f"Policy config failed: {rep.get('message')}"

        # 3. Test Infer Request (SmolVLA returns 30 action steps)
        obs_bytes = serialize_observation(dummy_timed_observation)
        client_sock.send(msgpack.packb({"type": "infer", "data": obs_bytes}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok", f"Infer request failed: {rep.get('message')}"
        assert "actions" in rep

        actions = deserialize_actions(rep["actions"])
        assert len(actions) == 30
        for i, timed_act in enumerate(actions):
            act = timed_act.get_action()
            assert act.shape == (6,)
            assert act.dtype == np.float32
            assert not np.isnan(act).any()
            assert not np.isinf(act).any()

        # 4. Test Disconnect
        client_sock.send(msgpack.packb({"type": "disconnect"}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok"

        # 5. Test Unknown message type error handling
        client_sock.send(msgpack.packb({"type": "unknown_cmd"}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "error"
        assert "Unknown message type" in rep["message"]

    finally:
        client_sock.close(linger=0)
        client_ctx.term()
        server.stop()
        assert server.running is False


def test_zmq_server_fails_fast_on_missing_task(dummy_policy_config, dummy_timed_observation):
    """Verify that server returns error when client omits 'task' in observation."""
    config = ZmqServerConfig(host="127.0.0.1", port=0, fps=30)
    server = ZmqPolicyServer(config)
    server.start_background()

    port = server.bound_port
    client_ctx = zmq.Context()
    client_sock = client_ctx.socket(zmq.REQ)
    client_sock.setsockopt(zmq.RCVTIMEO, 25000)
    client_sock.setsockopt(zmq.SNDTIMEO, 5000)
    client_sock.connect(f"tcp://127.0.0.1:{port}")

    try:
        # Load policy
        cfg_bytes = serialize_policy_config(dummy_policy_config)
        client_sock.send(
            msgpack.packb({"type": "policy_config", "data": cfg_bytes}, use_bin_type=True)
        )
        assert msgpack.unpackb(client_sock.recv(), raw=False)["status"] == "ok"

        # Strip task from observation
        bad_obs_dict = dict(dummy_timed_observation.get_observation())
        del bad_obs_dict["task"]
        from lerobot.async_inference.helpers import TimedObservation
        bad_obs = TimedObservation(
            timestamp=dummy_timed_observation.get_timestamp(),
            timestep=dummy_timed_observation.get_timestep(),
            observation=bad_obs_dict,
        )

        bad_bytes = serialize_observation(bad_obs)
        client_sock.send(msgpack.packb({"type": "infer", "data": bad_bytes}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "error"
        assert "Missing or empty 'task' instruction" in rep["message"]

    finally:
        client_sock.close(linger=0)
        client_ctx.term()
        server.stop()


def test_zmq_server_fails_fast_on_invalid_state_dim(dummy_policy_config, dummy_timed_observation):
    """Verify that server returns error when client sends non-12D observation.state."""
    config = ZmqServerConfig(host="127.0.0.1", port=0, fps=30)
    server = ZmqPolicyServer(config)
    server.start_background()

    port = server.bound_port
    client_ctx = zmq.Context()
    client_sock = client_ctx.socket(zmq.REQ)
    client_sock.setsockopt(zmq.RCVTIMEO, 25000)
    client_sock.setsockopt(zmq.SNDTIMEO, 5000)
    client_sock.connect(f"tcp://127.0.0.1:{port}")

    try:
        # Load policy
        cfg_bytes = serialize_policy_config(dummy_policy_config)
        client_sock.send(
            msgpack.packb({"type": "policy_config", "data": cfg_bytes}, use_bin_type=True)
        )
        assert msgpack.unpackb(client_sock.recv(), raw=False)["status"] == "ok"

        # Pass 6D state instead of 12D
        bad_obs_dict = dict(dummy_timed_observation.get_observation())
        bad_obs_dict["observation.state"] = np.zeros(6, dtype=np.float32)
        from lerobot.async_inference.helpers import TimedObservation
        bad_obs = TimedObservation(
            timestamp=dummy_timed_observation.get_timestamp(),
            timestep=dummy_timed_observation.get_timestep(),
            observation=bad_obs_dict,
        )

        bad_bytes = serialize_observation(bad_obs)
        client_sock.send(msgpack.packb({"type": "infer", "data": bad_bytes}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "error"
        assert "SmolVLA requires 12D observation.state" in rep["message"]

    finally:
        client_sock.close(linger=0)
        client_ctx.term()
        server.stop()

# Copyright 2026 LeKiwi Robotics Team

import time
import zmq
import pytest

from lerobot_policy_server.config import ZmqServerConfig
from lerobot_policy_server.protocol import (
    deserialize_actions,
    msgpack,
    serialize_observation,
    serialize_policy_config,
)
from lerobot_policy_server.server import ZmqPolicyServer


def test_zmq_server_full_integration(dummy_policy_config, dummy_timed_observation):
    # Bind to port 0 (OS picks an available ephemeral port)
    config = ZmqServerConfig(host="127.0.0.1", port=0, fps=50, device="cpu")
    server = ZmqPolicyServer(config)
    server.start_background()

    port = server.bound_port
    assert port > 0
    assert server.running is True

    # Setup client
    client_ctx = zmq.Context()
    client_sock = client_ctx.socket(zmq.REQ)
    client_sock.setsockopt(zmq.RCVTIMEO, 2000)  # 2s timeout
    client_sock.setsockopt(zmq.SNDTIMEO, 2000)
    client_sock.connect(f"tcp://127.0.0.1:{port}")

    try:
        # 1. Test Handshake
        client_sock.send(msgpack.packb({"type": "handshake"}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok"

        # 2. Test Policy Config
        cfg_bytes = serialize_policy_config(dummy_policy_config)
        client_sock.send(msgpack.packb({"type": "policy_config", "data": cfg_bytes}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok"

        # 3. Test Infer Request
        obs_bytes = serialize_observation(dummy_timed_observation)
        client_sock.send(msgpack.packb({"type": "infer", "data": obs_bytes}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok"
        assert "actions" in rep

        actions = deserialize_actions(rep["actions"])
        assert len(actions) == dummy_policy_config.actions_per_chunk

        # 4. Test Disconnect
        client_sock.send(msgpack.packb({"type": "disconnect"}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "ok"

        # 5. Test Unknown message type
        client_sock.send(msgpack.packb({"type": "invalid_ping"}, use_bin_type=True))
        rep = msgpack.unpackb(client_sock.recv(), raw=False)
        assert rep["status"] == "error"
        assert "Unknown message type" in rep["message"]

    finally:
        client_sock.close(linger=0)
        client_ctx.term()
        server.stop()
        assert server.running is False

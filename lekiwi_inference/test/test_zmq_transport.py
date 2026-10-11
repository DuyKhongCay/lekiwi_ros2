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

"""Unit tests for ZeroMQ transport client against an in-process mock REP server."""

import threading
import time

import msgpack
import numpy as np
import pytest
import zmq

from lekiwi_inference.protocol import (
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
    serialize_actions,
)
from lekiwi_inference.transport.zmq_transport import ZmqPolicyTransport


class MockZmqServer:
    """Simulates basic LeRobot policy server responses for transport verification."""

    def __init__(self, port: int = 5566) -> None:
        """Initializes mock server context and bind endpoint."""
        self.port = port
        self.endpoint = f"tcp://127.0.0.1:{port}"
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REP)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(self.endpoint)
        self.running = True

    def run(self) -> None:
        """Processes request envelopes matching the production policy server protocol."""
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

    def stop(self) -> None:
        """Stops the server loop and closes sockets."""
        self.running = False
        try:
            self.socket.close(linger=0)
            self.ctx.term()
        except Exception:
            pass


def test_zmq_transport_lifecycle():
    """Validates complete client-server transport lifecycle including infer request-reply."""
    port = 5578
    server = MockZmqServer(port=port)
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()

    transport = ZmqPolicyTransport(server_address=f"127.0.0.1:{port}", timeout_ms=500)

    try:
        assert transport.connect()
        assert transport.is_connected()

        assert transport.send_handshake()

        cfg = RemotePolicyConfig(repo_id="test_repo", policy_type="act", fps=50.0)
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


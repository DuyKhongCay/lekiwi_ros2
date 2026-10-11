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

"""ZeroMQ REQ client implementation providing robust retry and lazy-pirate socket recovery."""

import logging
import threading
import time
from typing import Optional

import msgpack
import zmq

from lekiwi_inference.protocol import (
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
    deserialize_actions,
    serialize_observation,
    serialize_policy_config,
)
from lekiwi_inference.transport.base import PolicyTransport

logger = logging.getLogger(__name__)


class ZmqPolicyTransport(PolicyTransport):
    """Manages ZeroMQ REQ transport with automatic reconnection and non-blocking timeout handling."""

    def __init__(
        self,
        server_address: str = "127.0.0.1:8090",
        timeout_ms: int = 500,
        max_retries: int = 3,
    ) -> None:
        """Configures transport address and communication timeout deadlines."""
        self.server_address = server_address
        self.timeout_ms = timeout_ms
        self.max_retries = max_retries

        self._ctx: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._lock = threading.Lock()
        self._connected = False
        self._last_rtt_ms = -1.0

    def connect(self) -> bool:
        """Creates socket resources and establishes connection to the remote policy server."""
        with self._lock:
            return self._connect_internal()

    def _connect_internal(self) -> bool:
        """Internal helper handling raw socket allocation without deadlock."""
        try:
            if self._socket is not None:
                self._socket.close(linger=0)
            if self._ctx is None:
                self._ctx = zmq.Context()

            self._socket = self._ctx.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.LINGER, 0)

            endpoint = (
                f"tcp://{self.server_address}"
                if not self.server_address.startswith("tcp://")
                else self.server_address
            )
            self._socket.connect(endpoint)
            self._connected = True
            logger.info("Connected ZMQ REQ socket to %s", endpoint)
            return True
        except Exception as exc:
            logger.error("Failed to connect ZMQ transport: %s", exc)
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Sends clean disconnect notification and tears down all allocated socket contexts."""
        with self._lock:
            if self._socket is not None and self._connected:
                try:
                    payload = msgpack.packb({"type": "disconnect"}, use_bin_type=True)
                    self._socket.send(payload)
                    self._socket.recv()
                except Exception:
                    pass
                self._socket.close(linger=0)
                self._socket = None

            if self._ctx is not None:
                self._ctx.term()
                self._ctx = None

            self._connected = False
            logger.info("Disconnected ZMQ transport cleanly.")

    def is_connected(self) -> bool:
        """Reports live connectivity status to calling health monitors."""
        return self._connected

    def send_handshake(self) -> bool:
        """Verifies server responsiveness and resets session buffers."""
        req = {"type": "handshake"}
        resp = self._send_request(req)
        return resp is not None and resp.get("status") == "ok"

    def send_policy_config(self, config: RemotePolicyConfig) -> bool:
        """Transmits neural policy specification to initiate remote weights loading."""
        cfg_bytes = serialize_policy_config(config)
        req = {"type": "policy_config", "data": cfg_bytes}
        resp = self._send_request(req)
        return resp is not None and resp.get("status") == "ok"

    def infer(self, observation: TimedObservation) -> Optional[list[TimedAction]]:
        """Sends binary observation frame and parses predicted action horizon chunk."""
        obs_bytes = serialize_observation(observation)
        req = {"type": "infer", "data": obs_bytes}
        t0 = time.perf_counter()
        resp = self._send_request(req)
        if resp is None:
            return None

        self._last_rtt_ms = (time.perf_counter() - t0) * 1000.0
        if resp.get("status") != "ok":
            logger.error("Inference server returned error: %s", resp.get("message"))
            return None

        actions_bytes = resp.get("actions")
        if not actions_bytes:
            return None
        return deserialize_actions(actions_bytes)

    def ping(self) -> float:
        """Returns the most recent measured round-trip latency in milliseconds."""
        return self._last_rtt_ms

    def _send_request(self, request_envelope: dict) -> Optional[dict]:
        """Executes request-reply transaction with Lazy Pirate recovery on timeout."""
        with self._lock:
            if not self._connected or self._socket is None:
                if not self._connect_internal():
                    return None

            try:
                msg_bytes = msgpack.packb(request_envelope, use_bin_type=True)
                self._socket.send(msg_bytes)
                reply_bytes = self._socket.recv()
                return msgpack.unpackb(reply_bytes, raw=False)
            except zmq.Again:
                logger.warning("ZMQ timeout on %s; resetting REQ socket.", self.server_address)
                self._connect_internal()
                return None
            except Exception as exc:
                logger.error("ZMQ transmission failed: %s; resetting socket.", exc)
                self._connect_internal()
                return None

